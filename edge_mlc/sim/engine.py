"""Discrete-event simulator for one mission run.

What is REAL code here:  the decision modules (deciders/) and the link
measurement code (common/monitor.py: heartbeat loss/RTT, capacity estimator).
What is SIMULATED:       the links (sim/scenario.py), packets in flight,
                         queues, the operator's commands, the ESP32 failsafe.

  run(cfg, decider, variant, seed) -> (metrics dict, timeline list)
"""
import heapq
import itertools
import math

import numpy as np

from common.monitor import Monitor
from sim import scenario as S

HUB_TO_OP_MS = 5.0          # hub <-> operator over the internet (A)
FRESH_S = 0.5               # a view is "live" if its newest frame is < 0.5 s old (A)
FAILSAFE_S = 0.5            # ESP32 stops if no fresh command for 0.5 s (firmware value)
VMAX = 5.0                  # m/s at full throttle (~18 km/h, THeMIS-class) (A)
TELEM_KBPS = 30.0
CHECK_AHEAD_S = 5.0         # a switch is "false" if the old link stayed fine this long


class Sim:
    def __init__(self, cfg, decider, variant="nominal", seed=0, overrides=None):
        self.cfg, self.dec = cfg, decider
        self.rng = np.random.default_rng(seed + 1000)
        self.truth = S.build(variant, seed, overrides)
        self.links = [l["name"] for l in cfg["links"]]
        self.ladder = cfg["ladder"]
        self.now = 0.0
        self.q, self.cnt = [], itertools.count()
        self.mon = Monitor(cfg["links"], cfg["timing"]["hb_timeout_ms"], clock=lambda: self.now)
        self.stale_ms = cfg["proposed"]["stale_cmd_ms"]

        # uplink queue + per-report counters per link
        self.backlog = {l: 0.0 for l in self.links}           # kbit waiting
        self.ovf = {l: 0.0 for l in self.links}               # recent overflow fraction
        self.win = {l: {"sent": 0.0, "got": 0.0} for l in self.links}
        self.kbit_sent = {l: 0.0 for l in self.links}
        self.kbit_dup = 0.0

        # plan state
        self.plan = None
        self.cam = {c: {"links": [], "level": 0} for c in ("front", "rear")}
        self.cam_gen = {c: 0 for c in self.cam}
        self.cmd_links = self.links[:2]
        self.cap = 1.0
        self.hb_fast = False

        # car / operator state
        self.cmd_seq = 0
        self.car_last_seq = 0
        self.car_cmd = (0.0, 0.0)
        self.car_last_exec = -math.inf
        self.view = {c: (-math.inf, 0) for c in ("front", "rear")}   # (capture time, level)
        self.c = dict(cmd_sent=0, cmd_exec=0, cmd_fresh=0, stale_exec=0, stale_rejected=0)
        self.exec_times = []
        self.failsafe, self.fs_stops, self.fs_time = False, 0, 0.0
        self.rows = []

    # ------------------------------------------------------------ helpers
    def at(self, t, fn, *args):
        heapq.heappush(self.q, (t, next(self.cnt), fn, args))

    def tr(self, link):
        """Ground truth of a link right now."""
        L, i = self.truth[link], min(int(self.now / S.DT), len(self.truth["t"]) - 1)
        g = lambda k: L[k][i]
        return dict(up=bool(g("up")), loss=g("loss"), owd=g("owd"), jit=g("jit"), cap=g("cap"),
                    signal=None if L["signal"] is None else g("signal"), buffer_ms=L["buffer_ms"])

    def owd(self, T):
        return T["owd"] + abs(self.rng.normal(0, T["jit"]))

    def qdelay(self, link, T):
        return 1000 * self.backlog[link] / max(T["cap"], 1.0)

    def lost(self, T, extra=0.0, n=1):
        if not T["up"]:
            return True
        p = min(1.0, T["loss"] + extra)
        return self.rng.random() > (1 - p) ** n

    # ------------------------------------------------------------ physics (uplink queues, ESP32)
    def physics(self):
        dt = 0.02
        for ln in self.links:
            T = self.tr(ln)
            offered = TELEM_KBPS * (ln in self.cmd_links[:1])
            for c, v in self.cam.items():
                if ln in v["links"]:
                    offered += self.ladder[v["level"]]["kbps"]
            inn = offered * dt
            self.kbit_sent[ln] += inn
            if not T["up"]:                      # down: everything lost (via `up`), queue flushed
                self.backlog[ln], self.ovf[ln] = 0.0, 0.0
                self.win[ln]["sent"] += inn
                continue
            self.backlog[ln] += inn - T["cap"] * dt
            limit = T["cap"] * T["buffer_ms"] / 1000
            drop = max(0.0, self.backlog[ln] - limit)
            self.backlog[ln] = min(max(self.backlog[ln], 0.0), limit)
            frac = drop / inn if inn > 0 else 0.0
            self.ovf[ln] = 0.7 * self.ovf[ln] + 0.3 * frac
            self.win[ln]["sent"] += inn
            self.win[ln]["got"] += max(0.0, inn - drop) * (1 - T["loss"])
        # ESP32 failsafe
        fs = self.now - self.car_last_exec > FAILSAFE_S
        if fs and not self.failsafe and self.now > 3:
            self.fs_stops += 1
        if fs and self.now > 3:
            self.fs_time += dt
        self.failsafe = fs
        self.at(self.now + dt, self.physics)

    # ------------------------------------------------------------ heartbeats + hub reports
    def heartbeat(self, ln, seq):
        self.mon.hb_sent(ln, seq)
        T = self.tr(ln)
        if not self.lost(T, self.ovf[ln], n=2):
            rtt = (self.owd(T) + self.qdelay(ln, T) + self.owd(T)) / 1000
            self.at(self.now + rtt, self.mon.hb_echo, ln, seq)
        period = 0.05 if self.hb_fast else 1 / self.cfg["timing"]["hb_hz"]
        self.at(self.now + period, self.heartbeat, ln, seq + 1)

    def report(self, ln):
        w = self.win[ln]
        kbps = w["got"] / 0.2
        loss = 1 - w["got"] / w["sent"] if w["sent"] > 0 else 0.0
        self.win[ln] = {"sent": 0.0, "got": 0.0}
        T = self.tr(ln)
        if not self.lost(T):
            self.at(self.now + self.owd(T) / 1000, self.mon.report, ln, kbps, loss)
        self.at(self.now + 0.2, self.report, ln)

    # ------------------------------------------------------------ video
    def camera(self, cam, gen):
        if gen != self.cam_gen[cam]:
            return                                   # superseded by a newer target
        v = self.cam[cam]
        lv = self.ladder[v["level"]]
        if v["level"] == 0 or not v["links"]:
            return                                   # paused: woken again by the next target change
        nfrag = max(1, math.ceil(lv["kbps"] / lv["fps"] * 1000 / 8 / 1100))
        best = None
        for k, ln in enumerate(v["links"]):
            if k:
                self.kbit_dup += lv["kbps"] / lv["fps"]
            T = self.tr(ln)
            if self.lost(T, self.ovf[ln], nfrag):
                continue
            arr = self.now + (self.owd(T) + self.qdelay(ln, T) + HUB_TO_OP_MS) / 1000
            best = arr if best is None else min(best, arr)          # first copy wins
        if best is not None:
            self.at(best, self.frame_arrives, cam, self.now, v["level"])
        self.at(self.now + 1 / lv["fps"], self.camera, cam, gen)

    def frame_arrives(self, cam, t_cap, level):
        if t_cap > self.view[cam][0]:
            self.view[cam] = (t_cap, level)

    # ------------------------------------------------------------ commands
    def operator(self):
        thr, steer, _ = S.driver(self.now)
        self.cmd_seq += 1
        self.c["cmd_sent"] += 1
        for ln in self.cmd_links:
            T = self.tr(ln)
            if not self.lost(T):
                arr = self.now + (HUB_TO_OP_MS + self.owd(T)) / 1000
                self.at(arr, self.car_rx, self.cmd_seq, self.now, thr, steer)
        self.at(self.now + 1 / self.cfg["timing"]["cmd_hz"], self.operator)

    def car_rx(self, seq, t_send, thr, steer):
        if seq <= self.car_last_seq:
            return                                   # duplicate or older: ignore
        self.car_last_seq = seq
        age = (self.now - t_send) * 1000
        if self.dec.reject_stale and age > self.stale_ms:
            self.c["stale_rejected"] += 1
            return
        self.car_cmd = (thr, steer)
        self.car_last_exec = self.now
        self.exec_times.append(self.now)
        self.c["cmd_exec"] += 1
        if age > self.stale_ms:
            self.c["stale_exec"] += 1
        else:
            self.c["cmd_fresh"] += 1

    # ------------------------------------------------------------ decision loop
    def decide(self):
        snap = self.mon.snapshot()
        for ln in self.links:
            T = self.tr(ln)
            if T["signal"] is None or not T["up"]:
                snap[ln]["signal"] = None
            elif ln == "lte":                               # modem polled once per second
                if int(self.now * 10) % 10 == 0 or not hasattr(self, "_lteq"):
                    self._lteq = round(T["signal"] + self.rng.normal(0, 2))
                snap[ln]["signal"] = self._lteq
            else:
                snap[ln]["signal"] = T["signal"] + self.rng.normal(0, 1.0)
        thr, steer = self.car_cmd
        plan = self.dec.decide({"now": self.now, "links": snap, "thr": thr, "steer": steer,
                                "op_mode": None, "clock": self.now})
        self.plan = plan
        for c in self.cam:
            new = {"links": list(plan["video"][c]["links"]), "level": plan["video"][c]["level"]}
            if new != self.cam[c]:              # camera wakes at once on a new target (as video.py)
                self.cam_gen[c] += 1
                self.at(self.now, self.camera, c, self.cam_gen[c])
            self.cam[c] = new
        self.cmd_links = list(plan["cmd_links"])
        self.cap = plan["cap"]
        self.hb_fast = plan.get("hb_fast", False)
        self.at(self.now + 1 / self.cfg["timing"]["loop_hz"], self.decide)

    # ------------------------------------------------------------ measurement (0.1 s grid)
    def sample(self):
        thr_int, _, man = S.driver(self.now)
        need = self.cfg["modes"][man]
        prot = min(need, key=lambda c: need[c]["prio"])
        tcap, level = self.view[prot]
        fresh = self.now - tcap <= FRESH_S
        thr_exec = 0.0 if self.failsafe else self.car_cmd[0] * self.cap
        p = self.plan or {"video": {prot: {"links": []}}, "status": "N/A", "mode": "N/A"}
        self.rows.append(dict(
            t=round(self.now, 2), man=man, prot=prot,
            useful=fresh and level >= need[prot]["levels"][0],
            live=fresh and level >= 2, level=level if fresh else 0,
            link=(p["video"][prot]["links"] or [None])[0],
            dup=len(p["video"][prot]["links"]) > 1,
            thr_int=thr_int, thr_exec=thr_exec, cap=self.cap, fs=self.failsafe,
            status=p.get("status"), mode=p.get("mode"),
            up={ln: self.tr(ln)["up"] for ln in self.links},
        ))
        self.at(self.now + 0.1, self.sample)

    # ------------------------------------------------------------ run
    def run(self):
        self.at(0.0, self.physics)
        for k, ln in enumerate(self.links):
            self.at(0.003 * k, self.heartbeat, ln, 1)
            self.at(0.2 + 0.001 * k, self.report, ln)
        self.at(0.0, self.operator)
        self.at(0.1, self.decide)
        self.at(0.15, self.sample)
        while self.q:
            t, _, fn, args = heapq.heappop(self.q)
            if t > S.T_END:
                break
            self.now = t
            fn(*args)
        return self.metrics(), self.rows

    # ------------------------------------------------------------ metrics
    def link_usable(self, ln, t0, t1, req):
        L = self.truth[ln]
        i0, i1 = int(t0 / S.DT), int(min(t1, S.T_END) / S.DT)
        return bool(np.all(L["up"][i0:i1]) and np.all(L["loss"][i0:i1] <= req["max_loss"])
                    and np.all(2 * L["owd"][i0:i1] <= req["max_rtt_ms"]))

    def metrics(self):
        R = [r for r in self.rows if r["t"] >= 3.0]
        dt = 0.1
        moving = [r for r in R if abs(r["thr_int"]) > 0.05]
        useful = np.mean([r["useful"] for r in R])
        live = np.mean([r["live"] for r in R])
        blind_m = sum(abs(r["thr_exec"]) * VMAX * dt for r in R if not r["useful"])
        dist = sum(abs(r["thr_exec"]) * VMAX * dt for r in R)
        dist_int = sum(abs(r["thr_int"]) * VMAX * dt for r in R)

        # control: longest gap between executed commands while the driver wants to move
        ex = np.array(self.exec_times)
        gaps = np.diff(ex[ex > 3.0]) if len(ex) > 1 else np.array([S.T_END])
        starts = ex[ex > 3.0][:-1]
        gaps_mv = [g for a, g in zip(starts, gaps) if abs(S.driver(a)[0]) > 0.05 or g > 1]

        # switching of the protected camera's link
        sw = fs = pp = 0
        last = {}
        req = self.cfg["limits"]["video"]
        for a, b in zip(R, R[1:]):
            if a["prot"] != b["prot"] or not a["link"] or not b["link"] or a["link"] == b["link"]:
                continue
            sw += 1
            if b["link"] != self.links[0] and self.link_usable(a["link"], b["t"], b["t"] + CHECK_AHEAD_S, req):
                # left a link that stayed fine (returning to the preferred link does not count)
                fs += 1
            pv = last.get(b["prot"])
            if pv and pv[1] == b["link"] and b["t"] - pv[0] < 5.0:
                pp += 1
            last[b["prot"]] = (b["t"], a["link"])
        tot = sum(self.kbit_sent.values())
        phase = {}
        for a, b, name in S.PHASES:
            rr = [r for r in R if a <= r["t"] < b]
            phase[f"useful_{name}"] = 100 * np.mean([r["useful"] for r in rr]) if rr else np.nan

        # radio fade: when did the view leave the radio, relative to the moment the
        # radio truly became unusable for video? (negative = before = good)
        L = self.truth["radio"]
        t = self.truth["t"]
        bad = (~L["up"]) | (L["loss"] > req["max_loss"]) | (2 * L["owd"] > req["max_rtt_ms"])
        k = np.argmax(bad & (t > 30) & (t < 80))
        t_fail = t[k] if bad[k] else np.nan
        t_leave = next((b["t"] for a, b in zip(R, R[1:]) if 30 < b["t"] < 80
                        and a["link"] == "radio" and b["link"] != "radio"), np.nan)
        # view interruptions: protected view stops being useful (count + longest)
        runs, cur = [], 0.0
        for r in R:
            if not r["useful"]:
                cur += dt
            elif cur:
                runs.append(cur)
                cur = 0.0
        if cur:
            runs.append(cur)
        return dict(**phase, radio_leave_minus_fail_s=t_leave - t_fail,
            prot_level_mean=float(np.mean([r["level"] for r in R])),
            interruptions=len(runs), longest_interruption_s=max(runs, default=0.0),
            useful_view_pct=100 * useful,
            live_view_pct=100 * live,
            blind_m=blind_m,
            progress_pct=100 * dist / max(dist_int, 1e-9),
            max_cmd_gap_s=float(max(gaps_mv) if gaps_mv else 0.0),
            fresh_cmd_pct=100 * self.c["cmd_fresh"] / max(self.c["cmd_sent"], 1),
            stale_exec=self.c["stale_exec"],
            stale_rejected=self.c["stale_rejected"],
            failsafe_stops=self.fs_stops,
            failsafe_s=self.fs_time,
            switches=sw, false_switches=fs, pingpongs=pp,
            dup_overhead_pct=100 * self.kbit_dup / max(tot, 1e-9),
            lte_mbit=self.kbit_sent.get("lte", 0) / 1000,
            total_mbit=tot / 1000,
        )


def run(cfg, decider, variant="nominal", seed=0, overrides=None):
    return Sim(cfg, decider, variant, seed, overrides).run()
