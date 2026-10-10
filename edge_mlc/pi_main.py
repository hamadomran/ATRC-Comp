"""Car-side main program (runs on the Raspberry Pi, as root).

  sudo python3 pi_main.py --decider baseline --run r01
  sudo python3 pi_main.py --decider proposed --run r02
  sudo python3 pi_main.py --decider proposed --features predict=0 --run r03   # ablation
  python3 pi_main.py --sim ...   # laptop test: loopback links, fake cameras, fake ESP32

Threads: per-link receivers, heartbeats, radio-stat pollers, two cameras,
ESP32 serial, and the 10 Hz decision loop.
"""
import argparse
import json
import threading
import time

import yaml

from common import protocol as P
from common.links import Link
from common.logger import Logger
from common.monitor import Monitor
from common.video import Camera
from deciders.baseline import Baseline
from deciders.proposed import Proposed


# ------------------------------------------------------------------ ESP32
class ESP32:
    """Serial line protocol (115200 baud, one line per message):
       Pi -> ESP32   C,<seq>,<thr>,<steer>,<cap>   thr/steer -1000..1000, cap 0..1000
                     K,<cap>                       speed cap update only
       ESP32 -> Pi   T,<ms>,<thr_out>,<steer_out>,<src>,<cap>,<failsafe>
    """

    def __init__(self, port, baud, log):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.log = log
        self.telem = {"thr": 0.0, "steer": 0.0, "src": "NONE", "failsafe": 1}
        threading.Thread(target=self._rx, daemon=True).start()

    def send_cmd(self, seq, thr, steer, cap):
        self.ser.write(f"C,{seq},{int(thr*1000)},{int(steer*1000)},{int(cap*1000)}\n".encode())

    def send_cap(self, cap):
        self.ser.write(f"K,{int(cap*1000)}\n".encode())

    def _rx(self):
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith("T,"):
                f = line.split(",")
                if len(f) >= 7:
                    self.telem = {"thr": int(f[2]) / 1000, "steer": int(f[3]) / 1000,
                                  "src": f[4], "cap": int(f[5]) / 1000, "failsafe": int(f[6])}
                    self.log.log("esp32", **self.telem)


class FakeESP32:
    """Stand-in for --sim: same interface, failsafe after 500 ms."""

    def __init__(self, log):
        self.log, self.last, self.cap = log, 0.0, 1.0
        self.telem = {"thr": 0.0, "steer": 0.0, "src": "NONE", "failsafe": 1}
        threading.Thread(target=self._loop, daemon=True).start()

    def send_cmd(self, seq, thr, steer, cap):
        self.last, self.cap = time.monotonic(), cap
        self.telem.update(thr=max(-cap, min(cap, thr)), steer=steer, src="NET", failsafe=0)

    def send_cap(self, cap):
        self.cap = cap

    def _loop(self):
        while True:
            if time.monotonic() - self.last > 0.5:
                self.telem.update(thr=0.0, src="NONE", failsafe=1)
            self.log.log("esp32", **self.telem)
            time.sleep(0.05)


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/run.yaml")
    ap.add_argument("--decider", choices=["baseline", "proposed"], required=True)
    ap.add_argument("--features", default="", help="e.g. predict=0,degrade=0 (ablations)")
    ap.add_argument("--run", default=time.strftime("run_%H%M%S"))
    ap.add_argument("--sim", action="store_true", help="loopback test without hardware")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(a.config))
    log = Logger(f"logs/{a.run}/car.jsonl")
    log.log("start", decider=a.decider, features=a.features, config=cfg)

    hub = (cfg["hub"]["host"], cfg["hub"]["car_port"])
    links = {l["name"]: Link(l["name"], l["id"], l["iface"], hub, bind_device=not a.sim)
             for l in cfg["links"]}
    mon = Monitor(cfg["links"], cfg["timing"]["hb_timeout_ms"])
    if not a.sim:
        mon.start_pollers(cfg["links"])
    esp = FakeESP32(log) if a.sim else ESP32(cfg["esp32"]["port"], cfg["esp32"]["baud"], log)

    if a.decider == "baseline":
        dec = Baseline(cfg)
    else:
        feats = {k: v != "0" for k, v in (kv.split("=") for kv in a.features.split(",") if kv)}
        dec = Proposed(cfg, feats)

    # ---- shared state updated by receivers
    st = {"last_cmd_seq": 0, "thr": 0.0, "steer": 0.0, "op_mode": None, "cap": 1.0, "hb_fast": False}

    def on_packet(m):
        s, ln = m["stream"], m["rx_link"]
        if s == P.HB_ECHO:
            mon.hb_echo(ln, m["seq"])
        elif s == P.REPORT:
            r = P.jload(m["payload"])
            mon.report(ln, r["kbps"], r["loss"])
        elif s == P.CMD:
            if m["seq"] <= st["last_cmd_seq"]:
                return                                   # duplicate copy or out of order
            c = P.jload(m["payload"])
            age_ms = (time.time() - c["t_op"]) * 1000
            stale = dec.reject_stale and age_ms > cfg["proposed"].get("stale_cmd_ms", 200)
            st["last_cmd_seq"] = m["seq"]
            st["op_mode"] = c.get("mode") or None
            log.log("cmd_rx", seq=m["seq"], link=ln, age_ms=round(age_ms, 1), stale=stale,
                    thr=c["thr"], steer=c["steer"])
            if stale:
                return                                   # Feature 3: never execute late commands
            st["thr"], st["steer"] = c["thr"], c["steer"]
            esp.send_cmd(m["seq"], c["thr"], c["steer"], st["cap"])

    for L in links.values():
        L.start_rx(on_packet)

    def send(link_name, stream, seq, payload, flags=0):
        links[link_name].send(stream, seq, payload, flags)

    cams = {n: Camera(n, "synthetic" if a.sim else cfg["cameras"][n]["device"], cfg["ladder"], send)
            for n in ("front", "rear")}
    for c in cams.values():
        c.start()

    # ---- heartbeats: every link, always (10 Hz, 20 Hz while preparing a switch)
    def heartbeats():
        seq = 0
        while True:
            seq += 1
            for ln, L in links.items():
                mon.hb_sent(ln, seq)
                L.send(P.HB, seq, b"")
            time.sleep(1 / (cfg["timing"]["hb_hz"] * (2 if st["hb_fast"] else 1)))
    threading.Thread(target=heartbeats, daemon=True).start()

    # ---- 10 Hz decision loop
    period = 1 / cfg["timing"]["loop_hz"]
    telem_seq = 0
    prev_plan = None
    while True:
        t0 = time.monotonic()
        if a.sim:                                        # laptop test: read fake impairments
            try:
                imp = json.load(open("sim_impair.json"))
            except Exception:
                imp = {}
            for ln, L in links.items():
                L.sim_loss = imp.get(ln, 0.0)
        snap = mon.snapshot()
        thr = esp.telem["thr"] if esp.telem["src"] != "NONE" else st["thr"]
        state = {"now": t0, "links": snap, "thr": thr, "steer": esp.telem["steer"],
                 "op_mode": st["op_mode"], "clock": time.time()}
        plan = dec.decide(state)

        # apply the plan
        for n, v in plan["video"].items():
            cams[n].set_target(v["links"], v["level"])
        st["cap"], st["hb_fast"] = plan["cap"], plan["hb_fast"]
        esp.send_cap(plan["cap"])
        for L in links.values():                         # tell the hub where to send commands
            L.send(P.CTRL, 0, P.jdump({"cmd_links": plan["cmd_links"]}))
        telem_seq += 1
        telem = {"mode": plan["mode"], "status": plan["status"], "cap": plan["cap"],
                 "video": plan["video"], "cmd_links": plan["cmd_links"], "esp32": esp.telem,
                 "links": {k: {"rtt": round(v["rtt_ms"]), "loss": round(v["loss"], 2),
                               "cap": round(v["cap_kbps"]), "alive": v["alive"]} for k, v in snap.items()}}
        links[plan["telem_link"]].send(P.TELEM, telem_seq, P.jdump(telem))

        log.log("tick", links=snap, plan=plan,
                cam_kbps={n: round(c.actual_kbps()) for n, c in cams.items()},
                changed=(prev_plan is None or plan["video"] != prev_plan["video"]
                         or plan["cmd_links"] != prev_plan["cmd_links"]))
        prev_plan = plan
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


if __name__ == "__main__":
    main()
