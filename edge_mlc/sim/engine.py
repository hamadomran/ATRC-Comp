"""Discrete-event simulator for one mission run (v2: position-based links).

What is REAL code here:  the decision modules (deciders/) and the link
measurement code (common/monitor.py: heartbeat loss/RTT, capacity estimator).
What is SIMULATED:       the links (sim/scenario.py), the vehicle's position,
                         packets in flight, queues (video + telemetry + bulk
                         uploads share each link's modem FIFO), the operator's
                         commands and the ESP32 failsafe.

  run(cfg, decider, variant, seed) -> (metrics dict, timeline list)
"""
import heapq
import itertools
import math
from bisect import bisect_right
from collections import deque

import numpy as np

from common.monitor import Monitor
from sim import scenario as S

HUB_TO_OP_MS = 5.0          # hub <-> operator over the internet (A)
FRESH_S = 0.5               # a view is "live" if its newest frame is < 0.5 s old (A)
FAILSAFE_S = 0.5            # ESP32 stops if no fresh command for 0.5 s (firmware value)
VMAX = 5.0                  # m/s at full throttle (~18 km/h, THeMIS-class) (A)
CREEP_MPS = 0.5             # (A) lost-link creep speed
CREEP_AFTER_S = 3.0         # (A) creep once no command executed for this long
TELEM_KBPS = 30.0
CHECK_AHEAD_S = 5.0         # a switch is "false" if the old link stayed fine this long
STALE_MS = 200.0            # (A) informational only: counts commands executed late


class Sim:
    def __init__(self, cfg, decider, variant="nominal", seed=0, overrides=None):
        self.cfg, self.dec = cfg, decider
        self.rng = np.random.default_rng(seed + 1000)
        self.ctx = S.make_ctx(variant, seed, overrides)
        self.driver = S.Driver()
        self.links = [l["name"] for l in cfg["links"]]
        self.ladder = cfg["ladder"]
        self.reqs = cfg["requirements"]      # operator usability per TRUE manoeuvre (metrics only)
        self.now = 0.0
        self.q, self.cnt = [], itertools.count()
        self.mon = Monitor(cfg["links"], cfg["timing"]["hb_timeout_ms"], clock=lambda: self.now)

        # vehicle
        self.d, self.v_mps, self.d_max = 0.0, 0.0, 0.0
        self.man, self.thr_int = "STOPPED", 0.0   # true manoeuvre label + asked throttle
        self.done, self.mission_time = False, None
        self.creep_s = 0.0

        # uplink queue + per-report counters per link
        self.backlog = {l: 0.0 for l in self.links}           # kbit waiting
        self.ovf = {l: 0.0 for l in self.links}               # recent overflow fraction
        self.win = {l: {"sent": 0.0, "got": 0.0} for l in self.links}
        self.kbit_sent = {l: 0.0 for l in self.links}
        self.kbit_dup = 0.0
        self.qdelay_max = {l: 0.0 for l in self.links}

        # vehicle upload queue (bulk data)
        self.uploads = []
        self.bg_n = 0
        self.bulk_sent_kbit = 0.0
        self.bulk_active = False
        self.bulk_req_kbps = self.bulk_bg_kbps = 0.0
        self.qdelay_bulk_max = {l: 0.0 for l in self.links}   # queue delay while bulk moved
        self.qd_bulk_n = self.qd_bulk_hi = 0                  # physics steps with bulk; of those, > 40 ms
        self.guard_ms = cfg.get("uploads", {}).get("queue_guard_ms", 30.0)   # (A) router AQM

        # video frame log: (latency_ms, capture_t, bulk_at_capture, cam, manoeuvre_at_capture)
        self.frames = []
        self.arrivals = {c: deque() for c in ("front", "rear")}   # (arrival_t) last ~1 s per camera
        self.bulk_item = None                                     # id of the item being sent

        # plan state (v2: no graceful degradation, the speed cap is always 1.0)
        self.plan = None
        self.cam = {c: {"links": [], "level": 0} for c in ("front", "rear")}
        self.cam_gen = {c: 0 for c in self.cam}
        self.cmd_links = self.links[:2]
        self.hb_fast = False

        # car / operator state
        self.cmd_seq = 0
        self.car_last_seq = 0
        self.car_cmd = (0.0, 0.0)
        self.car_last_exec = -math.inf
        self.view = {c: (-math.inf, 0) for c in ("front", "rear")}   # (capture time, level)
        self.c = dict(cmd_sent=0, cmd_exec=0, cmd_fresh=0, stale_exec=0)
        self.exec_times = []
        self.failsafe, self.fs_stops, self.fs_time = False, 0, 0.0
        self.rows = []

    # ------------------------------------------------------------ helpers
    def at(self, t, fn, *args):
        heapq.heappush(self.q, (t, next(self.cnt), fn, args))

    def tr(self, link):
        """Ground truth of a link right now, at the vehicle's true position."""
        return S.link_state(link, self.now, self.d, self.ctx)

    def owd(self, T):
        return T["owd_ms"] + abs(self.rng.normal(0, T["jit_ms"]))

    def qdelay(self, link, T):
        return 1000 * self.backlog[link] / max(T["cap_kbps"], 1.0)

    def lost(self, T, extra=0.0, n=1):
        if not T["up"]:
            return True
        p = min(1.0, T["loss"] + extra)
        return self.rng.random() > (1 - p) ** n

    # ------------------------------------------------------------ physics (kinematics, queues, ESP32)
    def physics(self):
        dt = 0.02
        # vehicle kinematics: position d (km along the route, 0 = base)
        dirn = self.driver.direction
        if self.now - self.car_last_exec >= CREEP_AFTER_S and self.now > 3:
            v = CREEP_MPS * dirn                  # lost-link rule: creep until a command executes
            self.creep_s += dt
        else:
            thr = 0.0 if self.failsafe else self.car_cmd[0]
            v = dirn * thr * VMAX                 # negative throttle moves against `direction`
        self.v_mps = v
        self.d = max(0.0, self.d + v * dt / 1000.0)
        self.d_max = max(self.d_max, self.d)
        if self.ctx["blackout_t"] is None and self.d >= self.ctx["v"]["blackout_at"]:
            self.ctx["blackout_t"] = self.now     # jammer blackout: once per mission

        # uplink queues: video + telemetry + bulk share each link's modem FIFO
        bp = (self.plan or {}).get("bulk")
        pending = [u for u in self.uploads if u["delivered_kbit"] < u["size_kbit"]]
        order, target = [], None
        if bp and pending:
            if bp.get("mode") == "rate":          # shaped: the decider names the item
                target = next((u for u in pending if u["id"] == bp.get("item")), None)
            else:                                 # greedy: plain FIFO by creation
                order = sorted(pending, key=lambda u: u["created_t"])
                target = order[0]
        self.bulk_req_kbps = self.bulk_bg_kbps = 0.0
        self.bulk_active, self.bulk_item = False, None
        for ln in self.links:
            T = self.tr(ln)
            offered = TELEM_KBPS * (ln in self.cmd_links[:1])
            for c, vc in self.cam.items():
                if ln in vc["links"]:
                    offered += self.ladder[vc["level"]]["kbps"]
            bulk_rate = 0.0
            if target is not None and bp["link"] == ln:
                if bp.get("mode") == "greedy":    # TCP-like: fill the bottleneck and the buffer
                    bulk_rate = max(0.0, T["cap_kbps"] - offered)
                    if self.backlog[ln] < 0.9 * T["cap_kbps"] * T["buffer_ms"] / 1000:
                        bulk_rate += 0.1 * T["cap_kbps"]
                else:
                    bulk_rate = max(0.0, bp.get("rate_kbps") or 0.0)
                    # router queue management (A): once the backlog passes the
                    # guard, bulk gets at most the drain rate minus other traffic
                    if self.backlog[ln] > self.guard_ms / 1000 * T["cap_kbps"]:
                        bulk_rate = min(bulk_rate, max(0.0, T["cap_kbps"] - offered))
            bulk_in = bulk_rate * dt
            inn = offered * dt + bulk_in
            self.kbit_sent[ln] += inn
            if bulk_in > 0:
                self.bulk_sent_kbit += bulk_in
                target["sent_kbit"] += bulk_in
                self.bulk_active, self.bulk_item = True, target["id"]
                if target["requested"]:
                    self.bulk_req_kbps = bulk_rate
                else:
                    self.bulk_bg_kbps = bulk_rate
            if not T["up"]:                       # down: everything lost, queue flushed
                self.backlog[ln], self.ovf[ln] = 0.0, 0.0
                self.win[ln]["sent"] += inn
                continue
            self.backlog[ln] += inn - T["cap_kbps"] * dt
            limit = T["cap_kbps"] * T["buffer_ms"] / 1000
            drop = max(0.0, self.backlog[ln] - limit)
            self.backlog[ln] = min(max(self.backlog[ln], 0.0), limit)
            frac = drop / inn if inn > 0 else 0.0
            self.ovf[ln] = 0.7 * self.ovf[ln] + 0.3 * frac
            self.win[ln]["sent"] += inn
            self.win[ln]["got"] += max(0.0, inn - drop) * (1 - T["loss"])
            self.qdelay_max[ln] = max(self.qdelay_max[ln], self.qdelay(ln, T))
            if bulk_in > 0:
                qd = self.qdelay(ln, T)
                self.qdelay_bulk_max[ln] = max(self.qdelay_bulk_max[ln], qd)
                self.qd_bulk_n += 1
                self.qd_bulk_hi += qd > 40.0
                # goodput; lost bulk data is resent, so `remaining` falls by delivered only
                rem = bulk_in * (1 - frac) * (1 - T["loss"])
                for u in ([target] if bp.get("mode") == "rate" else order):
                    if rem <= 0:
                        break
                    x = min(rem, u["size_kbit"] - u["delivered_kbit"])
                    u["delivered_kbit"] += x
                    rem -= x
                    if u["delivered_kbit"] >= u["size_kbit"]:
                        u["done_t"], u["done_d"] = self.now, self.d

        # ESP32 failsafe (the speed cap is gone in v2; the failsafe stays)
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
            self.at(best, self.frame_arrives, cam, self.now, v["level"], self.bulk_active, self.man)
        self.at(self.now + 1 / lv["fps"], self.camera, cam, gen)

    def frame_arrives(self, cam, t_cap, level, bulk, man):
        self.frames.append(((self.now - t_cap) * 1000.0, t_cap, bulk, cam, man))
        self.arrivals[cam].append(self.now)
        if t_cap > self.view[cam][0]:
            self.view[cam] = (t_cap, level)

    # ------------------------------------------------------------ background recording
    def bg_chunks(self):
        """Every 60 s the vehicle queues a 4K recording chunk (10 MB) and a
        route-LiDAR chunk (5 MB) as background uploads (A)."""
        self.bg_n += 1
        for id_, kind, mb in ((f"rec_{self.bg_n}", "recording", 10), (f"lidar_{self.bg_n}", "lidar", 5)):
            self.uploads.append(dict(id=id_, kind=kind, size_kbit=mb * 8000.0, requested=False,
                                     created_t=self.now, created_d=self.d,
                                     sent_kbit=0.0, delivered_kbit=0.0, done_t=None, done_d=None))
        self.at(self.now + 60.0, self.bg_chunks)

    # ------------------------------------------------------------ commands (driver -> car)
    def operator(self):
        thr, steer, man, events, done = self.driver.step(self.now, self.d)
        self.man, self.thr_int = man, thr
        self.ctx["in_crater"] = self.driver.in_crater
        for ev in events:
            self.uploads.append(dict(ev, created_t=self.now, created_d=self.d,
                                     sent_kbit=0.0, delivered_kbit=0.0, done_t=None, done_d=None))
        if done:
            self.done, self.mission_time = True, self.now
            return
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
        self.car_cmd = (thr, steer)
        self.car_last_exec = self.now
        self.exec_times.append(self.now)
        self.c["cmd_exec"] += 1
        if (self.now - t_send) * 1000 > STALE_MS:
            self.c["stale_exec"] += 1                # informational only: never rejected in v2
        else:
            self.c["cmd_fresh"] += 1

    # ------------------------------------------------------------ decision loop
    def decide(self):
        snap = self.mon.snapshot()
        for ln in self.links:
            T = self.tr(ln)
            if T["signal"] is None or not T["up"]:
                snap[ln]["signal"] = None
            elif ln == "lte":                               # modem RSRP polled once per second
                if int(self.now * 10) % 10 == 0 or not hasattr(self, "_lteq"):
                    self._lteq = round(T["signal"] + self.rng.normal(0, 2))
                snap[ln]["signal"] = self._lteq
            else:
                snap[ln]["signal"] = T["signal"] + self.rng.normal(0, 1.0)
        thr, steer = self.car_cmd
        up_list = [{"id": u["id"], "requested": u["requested"],
                    "remaining_kbit": u["size_kbit"] - u["delivered_kbit"]}
                   for u in self.uploads if u["delivered_kbit"] < u["size_kbit"]]
        plan = self.dec.decide({"now": self.now, "links": snap, "thr": thr, "steer": steer,
                                "op_mode": None, "clock": self.now, "uploads": up_list})
        self.plan = plan
        for c in self.cam:
            new = {"links": list(plan["video"][c]["links"]), "level": plan["video"][c]["level"]}
            if new != self.cam[c]:              # camera wakes at once on a new target (as video.py)
                self.cam_gen[c] += 1
                self.at(self.now, self.camera, c, self.cam_gen[c])
            self.cam[c] = new
        self.cmd_links = list(plan["cmd_links"])
        self.hb_fast = plan.get("hb_fast", False)
        self.at(self.now + 1 / self.cfg["timing"]["loop_hz"], self.decide)

    # ------------------------------------------------------------ measurement (0.1 s grid)
    def sample(self):
        # usability is defined by the operator requirements for the TRUE
        # manoeuvre (config `requirements:`), independent of the controller:
        # the required camera must meet frame age, delivered frame rate over
        # the last 1 s, and resolution (from the newest frame's level)
        man = self.man
        rq = self.reqs[man]
        prot = rq["cam"]
        arr = self.arrivals[prot]
        while arr and self.now - arr[0] > 1.0:
            arr.popleft()
        fps = len(arr)                            # frames that ARRIVED in the last 1 s
        tcap, level = self.view[prot]
        age_ok = self.now - tcap <= rq["max_age_s"]
        useful = age_ok and fps >= rq["min_fps"] and self.ladder[level]["w"] >= rq["min_w"]
        thr_exec = 0.0 if self.failsafe else self.car_cmd[0]
        p = self.plan or {"video": {prot: {"links": []}}, "mode": "N/A"}
        self.rows.append(dict(
            t=round(self.now, 2), man=man, prot=prot,
            d=self.d, dir=self.driver.direction, state=self.driver.state,
            seg=S.segment(self.d, self.driver.direction, self.driver.state),
            crater=self.driver.in_crater, v_mps=abs(self.v_mps),
            useful=useful, fps=fps,
            live=self.now - tcap <= FRESH_S and level >= 2,
            level=level if age_ok else 0,
            link=(p["video"][prot]["links"] or [None])[0],
            dup=len(p["video"][prot]["links"]) > 1,
            thr_int=self.thr_int, thr_exec=thr_exec, fs=self.failsafe,
            mode=p.get("mode"),
            bulk=self.bulk_active, bulk_req_kbps=self.bulk_req_kbps, bulk_bg_kbps=self.bulk_bg_kbps,
            bulk_item=self.bulk_item,
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
        self.at(60.0, self.bg_chunks)
        while self.q:
            t, _, fn, args = heapq.heappop(self.q)
            if t > S.T_MAX or self.done:
                break                                # mission complete or 40 min hard stop
            self.now = t
            fn(*args)
        return self.metrics(), self.rows

    # ------------------------------------------------------------ metrics
    def truth_at(self, ln, row):
        """Ground truth at a recorded sample (uses the run's own trajectory)."""
        return S.link_state(ln, row["t"], row["d"], {**self.ctx, "in_crater": row["crater"]})

    def link_usable(self, ln, t0, t1, req):
        rs = [r for r in self.rows if t0 <= r["t"] <= t1]
        for r in rs:
            T = self.truth_at(ln, r)
            if not T["up"] or T["loss"] > req["max_loss"] or 2 * T["owd_ms"] > req["max_rtt_ms"]:
                return False
        return bool(rs)

    def transition_events(self, R):
        """Link-change events from ground truth along this run's trajectory:
        any link's `up` flips, or the best available true capacity changes by
        >= 30% within 1 s. Events closer than 5 s are merged (first kept)."""
        best = []
        for r in R:
            caps = []
            for ln in self.links:
                T = self.truth_at(ln, r)
                if T["up"]:
                    caps.append(T["cap_kbps"])
            best.append(max(caps) if caps else 0.0)
        events = [(b["t"], "up/down") for a, b in zip(R, R[1:]) if a["up"] != b["up"]]
        for i in range(10, len(R)):               # 10 samples = 1 s
            a, b = best[i - 10], best[i]
            if min(a, b) < 0.7 * max(a, b) - 1e-9:
                events.append((R[i]["t"], "capacity"))
        merged = []
        for t, kind in sorted(events):
            if not merged or t - merged[-1][0] >= 5.0:
                merged.append((t, kind))
        return merged

    def metrics(self):
        R = [r for r in self.rows if r["t"] >= 3.0]
        dt = 0.1
        useful = np.mean([r["useful"] for r in R])
        live = np.mean([r["live"] for r in R])
        blind_m = sum(r["v_mps"] * dt for r in R if not r["useful"])

        # control: longest gap between executed commands while the driver wants to move
        ex = np.array(self.exec_times)
        gaps = np.diff(ex[ex > 3.0]) if len(ex) > 1 else np.array([S.T_MAX])
        starts = ex[ex > 3.0][:-1]
        rt = np.array([r["t"] for r in self.rows])
        ri = np.array([abs(r["thr_int"]) for r in self.rows])

        def want_move(a):
            k = min(np.searchsorted(rt, a), len(ri) - 1)
            return ri[k] > 0.05
        gaps_mv = [g for a, g in zip(starts, gaps) if want_move(a) or g > 1]

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
        seg = {}
        for name in S.SEGMENTS:
            rr = [r for r in R if r["seg"] == name]
            seg[f"useful_{name}"] = 100 * np.mean([r["useful"] for r in rr]) if rr else np.nan

        # radio fade: when did the view leave the radio, relative to the moment the
        # radio truly became unusable for video? (negative = before = good)
        t_fail = np.nan
        for r in R:
            if r["dir"] > 0 and 1.0 < r["d"] < 1.7:
                T = self.truth_at("radio", r)
                if not T["up"] or T["loss"] > req["max_loss"] or 2 * T["owd_ms"] > req["max_rtt_ms"]:
                    t_fail = r["t"]
                    break
        t_leave = next((b["t"] for a, b in zip(R, R[1:]) if b["dir"] > 0 and 1.0 < b["d"] < 1.7
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

        # video frame delay, overall and while bulk data was being sent
        lat = np.array([f[0] for f in self.frames])
        latb = np.array([f[0] for f in self.frames if f[2]])
        Rb = [r for r in R if r["bulk"]]

        # uploads (Experiment B)
        up = {}
        reqd = [u for u in self.uploads if u["requested"]]
        for u in reqd:
            up[f"{u['id']}_s"] = (u["done_t"] - u["created_t"]) if u["done_t"] is not None else np.nan
        req_ts = list(up.values())
        up["req_mean_s"] = float(np.mean(req_ts)) if reqd else np.nan
        up["req_max_s"] = float(np.max(req_ts)) if reqd else np.nan
        bg = [u for u in self.uploads if not u["requested"]]
        up["bg_backlog_mb_at_end"] = sum(u["size_kbit"] - u["delivered_kbit"] for u in bg) / 8000
        up["bg_delivered_mb"] = sum(u["delivered_kbit"] for u in bg) / 8000
        up["bulk_mbit_total"] = self.bulk_sent_kbit / 1000
        # the video cost of pushing urgent data, in both situations
        Rd = [r for r in R if r["bulk_item"] == "scan_drive"]
        up["needed_fps_while_req_drive"] = float(np.mean([r["fps"] for r in Rd])) if Rd else np.nan
        Rs = [r for r in R if r["bulk_item"] in ("lidar_crater", "still", "clip")]
        up["needed_level_while_req_stopped"] = float(np.mean([r["level"] for r in Rs])) if Rs else np.nan

        # neutral measures per true manoeuvre
        for man, rq in self.reqs.items():
            rr = [r for r in R if r["man"] == man]
            up[f"fps_delivered_mean_{man}"] = float(np.mean([r["fps"] for r in rr])) if rr else np.nan
            up[f"level_mean_{man}"] = float(np.mean([r["level"] for r in rr])) if rr else np.nan
            ls = [f[0] for f in self.frames if f[4] == man and f[3] == rq["cam"] and f[1] >= 3.0]
            up[f"frame_delay_p95_ms_{man}"] = float(np.percentile(ls, 95)) if ls else np.nan

        # Experiment A: behaviour around ground-truth link changes
        events = self.transition_events(R)
        wins, trans = [], {}
        for t, _ in events:                       # merge overlapping [t-2, t+10] windows
            if wins and t - 2.0 <= wins[-1][1]:
                wins[-1] = (wins[-1][0], t + 10.0)
            else:
                wins.append((t - 2.0, t + 10.0))
        starts = [w[0] for w in wins]

        def inwin(t):
            i = bisect_right(starts, t) - 1
            return i >= 0 and t <= wins[i][1]
        Rw = [r for r in R if inwin(r["t"])]
        trans["n_transition_events"] = len(events)
        trans["n_trans_updown"] = sum(1 for _, k in events if k == "up/down")
        trans["n_trans_capacity"] = sum(1 for _, k in events if k == "capacity")
        trans["useful_transition_pct"] = 100 * np.mean([r["useful"] for r in Rw]) if Rw else np.nan
        trans["interruptions_transition"] = sum(
            1 for a, b in zip(R, R[1:]) if a["useful"] and not b["useful"] and inwin(b["t"]))

        return dict(**seg, **up, **trans,
            radio_leave_minus_fail_s=t_leave - t_fail,
            mission_time_s=float(self.mission_time if self.mission_time is not None else min(self.now, S.T_MAX)),
            mission_completed=float(self.mission_time is not None),
            lost_link_creep_s=self.creep_s,
            final_d_km=self.d, max_d_km=self.d_max,
            prot_level_mean=float(np.mean([r["level"] for r in R])),
            interruptions=len(runs), longest_interruption_s=max(runs, default=0.0),
            useful_view_pct=100 * useful,
            live_view_pct=100 * live,
            useful_view_pct_bulk=100 * np.mean([r["useful"] for r in Rb]) if Rb else np.nan,
            video_delay_p95_ms=float(np.percentile(lat, 95)) if len(lat) else np.nan,
            video_delay_p95_bulk_ms=float(np.percentile(latb, 95)) if len(latb) else np.nan,
            blind_m=blind_m,
            max_cmd_gap_s=float(max(gaps_mv) if gaps_mv else 0.0),
            stale_exec=self.c["stale_exec"],
            failsafe_stops=self.fs_stops,
            failsafe_s=self.fs_time,
            switches=sw, false_switches=fs, pingpongs=pp,
            dup_overhead_pct=100 * self.kbit_dup / max(tot, 1e-9),
            **{f"qdelay_max_{ln}_ms": self.qdelay_max[ln] for ln in self.links},
            **{f"qdelay_bulk_max_{ln}_ms": self.qdelay_bulk_max[ln] for ln in self.links},
            bulk_qdelay_over40_pct=100 * self.qd_bulk_hi / max(self.qd_bulk_n, 1),
            total_mbit=tot / 1000,
        )


def run(cfg, decider, variant="nominal", seed=0, overrides=None):
    return Sim(cfg, decider, variant, seed, overrides).run()
