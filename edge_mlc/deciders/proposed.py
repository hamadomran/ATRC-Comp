"""Proposed decision module.

Feature 1  Mission-aware allocation
  Part A   camera allocation by manoeuvre   (ModeDetector + Allocator)
  Part B   data upload scheduling by vehicle state (rate-limited bulk uploads)
Feature 2  Predictive switching              (Predictor: trend rule + handover schedule)

Part B is an upload_policy; all three share the same traffic shaping, queue
guard, bulk-link choice and camera allocation, only the order inside the
room differs:
  "video_first"   cameras (full quality) always before requested uploads
  "urgent_first"  requested uploads always before camera-above-minimum
  "adaptive"      ours: urgent_first while STOPPED/PRECISION, video_first
                  while DRIVE/REVERSE

Each feature can be switched off for ablation runs:
  Proposed(cfg, features={"mission": True, "uploads": True, "predict": True})
(uploads off forces upload_policy="video_first".)

Link names come from the config (first link = preferred). The decision
module never sees ground truth: only the measured snapshot it is given.
"""
import math
from collections import deque

import numpy as np

INF = math.inf
FIXED_NEEDS = {   # used when "mission" is OFF: fixed priorities (front first), full ladder for both
    "front": {"prio": 1, "levels": [2, 3, 4, 5], "max_rtt_ms": 300, "max_loss": 0.05},
    "rear": {"prio": 2, "levels": [2, 3, 4, 5], "max_rtt_ms": 300, "max_loss": 0.05},
}


# ===================================================================
# Feature 1a: what is the vehicle doing?
# ===================================================================
class ModeDetector:
    def __init__(self, p):
        self.p = p
        self.speed = 0.0              # smoothed |throttle| as a speed proxy (A)
        self.mode = "DRIVE"
        self.candidate, self.cand_since = "DRIVE", 0.0
        self.slow_since = None        # when speed first dropped below "stopped"
        self.last_stopped = -INF      # last time the vehicle was (nearly) still

    def update(self, now, thr, steer, op_mode):
        p = self.p
        self.speed = 0.8 * self.speed + 0.2 * abs(thr)          # ~0.5 s smoothing at 10 Hz
        if self.speed < p["stopped_speed"]:
            self.slow_since = self.slow_since or now
            self.last_stopped = now
        else:
            self.slow_since = None

        if op_mode:                                             # operator button wins
            cand = op_mode
        elif thr < p["reverse_thr"] and (now - self.last_stopped < 1.5 or self.mode == "REVERSE"):
            cand = "REVERSE"            # backwards after a stop = reverse, not braking
        elif self.slow_since and now - self.slow_since > p["stopped_for_s"]:
            cand = "STOPPED"
        elif abs(steer) > p["precision_steer"] or (0 < self.speed < p["precision_speed"]):
            cand = "PRECISION"
        else:
            cand = "DRIVE"

        if cand != self.candidate:
            self.candidate, self.cand_since = cand, now
        if cand == "REVERSE" or now - self.cand_since >= p["mode_debounce_s"]:
            self.mode = cand                                    # reverse applies instantly
        return self.mode


# ===================================================================
# Feature 2: predictive switching
# ===================================================================
METRICS = {"rtt_ms": +1, "loss": +1, "signal": -1}   # +1: higher is worse, -1: lower is worse


class Predictor:
    def __init__(self, cfg):
        self.p = cfg["proposed"]
        self.linkcfg = {l["name"]: l for l in cfg["links"]}
        self.hist = {l["name"]: deque() for l in cfg["links"]}
        self.trend = {}               # (link, metric) -> (current value, worsening rate per s)
        self.ttu = {}                 # (link, stream) -> seconds until unusable
        self.avoid_until = {}         # (link, stream) -> time; set by SWITCH

    def observe(self, now, links):
        """Store the newest readings and compute each metric's trend once per tick."""
        need, out_of = self.p["trend_consistency"]
        for ln, L in links.items():
            h = self.hist[ln]
            h.append((now, {m: L.get(m) for m in METRICS}))
            while h and now - h[0][0] > self.p["trend_window_s"] + 0.2:
                h.popleft()
            for m, worse in METRICS.items():
                vals = [x[1][m] for x in h]
                if len(h) < 8 or any(v is None for v in vals):
                    self.trend[(ln, m)] = None
                    continue
                t = np.array([x[0] for x in h])
                sm = np.convolve(np.array(vals, float), np.ones(3) / 3, mode="valid")   # light smoothing
                steps = np.diff(sm[-(out_of + 1):]) * worse
                rate = 0.0
                if np.sum(steps > 0) >= need:                       # steadily worsening
                    rate = max(0.0, np.polyfit(t[-len(sm):] - t[0], sm, 1)[0] * worse)
                self.trend[(ln, m)] = (sm[-1], rate)

    def _limit(self, metric, link, req):
        if metric == "rtt_ms":
            return req["max_rtt_ms"]
        if metric == "loss":
            return req["max_loss"]
        return self.linkcfg[link].get("signal_unusable")          # None if the link reports no signal

    def time_until_unusable(self, link, req):
        """Shortest time, over all metrics, until one crosses its limit."""
        best = INF
        for m, worse in METRICS.items():
            tr, lim = self.trend.get((link, m)), self._limit(m, link, req)
            if tr is None or lim is None:
                continue
            cur, rate = tr
            if worse * (cur - lim) > 0:                            # already past the limit
                return 0.0
            if rate > 0:
                best = min(best, abs(lim - cur) / rate)
        return best

    def scheduled_risk(self, link, clock):
        """Known handover schedule (e.g. Starlink every 15 s at fixed seconds):
        True from `pre` s before a handover until `post` s after it."""
        sch = self.linkcfg[link].get("handover_schedule")
        if not sch or clock is None:
            return False
        s = clock % sch["period_s"]
        to_next = min((o - s) % sch["period_s"] for o in sch["offsets_s"])
        since = min((s - o) % sch["period_s"] for o in sch["offsets_s"])
        return to_next <= sch["pre_s"] or since <= sch["post_s"]

    def margin(self, speed):
        p = self.p
        return p["margin_base_s"] + p["margin_per_speed_s"] * speed + p["switch_cost_s"]

    def avoided(self, now, link, stream):
        return now < self.avoid_until.get((link, stream), -INF)


# ===================================================================
# Feature 1b: share capacity by what matters in this mode
# ===================================================================
class Allocator:
    def __init__(self, cfg, predictor):
        self.cfg, self.p = cfg, cfg["proposed"]
        self.ladder = cfg["ladder"]
        self.pred = predictor
        self.links = [l["name"] for l in cfg["links"]]
        self.pref = self.links[0]                     # preferred link = first in config
        self.current = {"front": self.pref, "rear": self.pref}
        self.usable_since = {}        # (link, stream) -> time it became usable
        self.switched_at = {"front": -INF, "rear": -INF}

    def usable(self, now, L, ln, stream, req, predict_on):
        ok = (L["alive"] and L["rtt_p90"] <= req["max_rtt_ms"] and L["loss"] <= req["max_loss"]
              and not self.pred.avoided(now, ln, stream))
        if ok and predict_on and self.pred.ttu.get((ln, stream), INF) <= 0:
            ok = False
        key = (ln, stream)
        if ok:
            self.usable_since.setdefault(key, now)
        else:
            self.usable_since.pop(key, None)
        return ok

    def return_ok(self, now, stream, mode):
        """May this stream move back to the preferred link?"""
        since = self.usable_since.get((self.pref, stream))
        return (since is not None and now - since >= self.p["return_stable_s"]
                and self.pred.ttu.get((self.pref, stream), INF) == INF
                and now - self.switched_at[stream] >= self.p["hold_s"]
                and mode not in ("REVERSE", "PRECISION"))

    def allocate(self, now, links, mode, needs, predict_on, reasons, reserve=None):
        """reserve: kbps per link already promised to bulk uploads that outrank
        camera quality above the minimum (requested items, or any bulk while
        stopped). The protected camera's MINIMUM level is never reduced by it."""
        room = {ln: self.p["headroom"] * links[ln]["cap_kbps"] for ln in self.links}
        reserve = reserve or {}
        cmd_req = self.cfg["limits"]["cmd"]
        unmet, degraded = [], []

        # 1) commands (+telemetry): two best usable links, in preference order
        cmd_ok = [ln for ln in self.links if self.usable(now, links[ln], ln, "cmd", cmd_req, predict_on)]
        cmd_links = cmd_ok[:2] if cmd_ok else list(self.links)    # none meets limits: commands are tiny, send on all
        for ln in cmd_links:
            room[ln] -= 30                        # ~20 Hz commands + telemetry (A)
        if not cmd_ok:
            (degraded if any(links[l]["alive"] for l in self.links) else unmet).append("cmd")

        # 2) cameras in priority order
        video = {}
        for cam, need in sorted(needs.items(), key=lambda kv: kv[1]["prio"]):
            levels = need["levels"]
            if levels == [0]:
                video[cam] = {"links": [], "level": 0}
                continue
            req = {"max_rtt_ms": need.get("max_rtt_ms", 600), "max_loss": need.get("max_loss", 0.2)}
            min_kbps = self.ladder[levels[0]]["kbps"]
            # room left after the bulk reservation; the protected (prio 1)
            # camera's minimum outranks bulk, so it ignores the reservation
            free = {ln: room[ln] - (0.0 if need["prio"] <= 1 else reserve.get(ln, 0.0))
                    for ln in self.links}
            cands = [ln for ln in self.links
                     if self.usable(now, links[ln], ln, cam, req, predict_on) and free[ln] >= min_kbps]
            cur = self.current[cam]
            if not cands:
                alive = [ln for ln in self.links if links[ln]["alive"]]
                if need["prio"] <= 1 and alive:
                    # protected view: keep its minimum level on the least-bad link (degraded)
                    b = min(alive, key=lambda l: (links[l]["loss"], links[l]["rtt_p90"]))
                    if cur in alive and links[cur]["loss"] <= links[b]["loss"] + 0.05:
                        b = cur                                  # not clearly better: stay put
                    video[cam] = {"links": [b], "level": levels[0]}
                    room[b] -= min_kbps
                    if b != cur:
                        self.current[cam], self.switched_at[cam] = b, now
                    degraded.append(cam)
                    reasons.append(f"{cam}: no link meets limits -> minimum level on {b} (degraded)")
                    continue
                if need["prio"] <= 1:
                    # nothing looks alive: keep the minimum on the current link anyway
                    # (costs little if it is really dead; recovers instantly if not)
                    video[cam] = {"links": [cur], "level": levels[0]}
                    unmet.append(cam)
                    reasons.append(f"{cam}: no link alive -> minimum level kept on {cur}")
                    continue
                trickle = [ln for ln in self.links if links[ln]["alive"] and free[ln] >= self.ladder[1]["kbps"]]
                if trickle and levels[0] > 0:
                    video[cam] = {"links": [trickle[0]], "level": 1}
                    room[trickle[0]] -= self.ladder[1]["kbps"]
                else:
                    video[cam] = {"links": [], "level": 0}
                if need["prio"] <= 1:
                    unmet.append(cam)
                reasons.append(f"{cam}: no link can carry minimum -> {'trickle' if video[cam]['level'] else 'paused'}")
                continue
            if self.pref in cands and cur != self.pref and self.return_ok(now, cam, mode):
                choice = self.pref
            elif cur in cands:
                choice = cur
            else:
                choice = min(cands, key=self.links.index)        # best by preference order
            if choice != cur:
                reasons.append(f"{cam}: {cur}->{choice}")
                self.current[cam] = choice
                self.switched_at[cam] = now
            # levels above the minimum come after reserved bulk (requested uploads)
            fit = [lv for lv in levels if self.ladder[lv]["kbps"] <= room[choice] - reserve.get(choice, 0.0)]
            level = fit[-1] if fit else levels[0]
            room[choice] -= self.ladder[level]["kbps"]
            video[cam] = {"links": [choice], "level": level}
        return cmd_links, video, unmet, degraded


# ===================================================================
# The module the main loop calls
# ===================================================================
# Part B thresholds, used if the config has no `uploads:` block; all (A)
UPLOAD_DEFAULTS = {"bulk_budget_share": 0.9, "queue_guard_ms": 30.0}


class Proposed:
    name = "proposed"
    reject_stale = False

    def __init__(self, cfg, features=None, upload_policy="adaptive"):
        self.cfg, self.p = cfg, cfg["proposed"]
        self.f = {"mission": True, "uploads": True, "predict": True, **(features or {})}
        self.upload_policy = upload_policy if self.f["uploads"] else "video_first"
        self.up = {**UPLOAD_DEFAULTS, **cfg.get("uploads", {})}
        self.mode_det = ModeDetector(self.p)
        self.pred = Predictor(cfg)
        self.alloc = Allocator(cfg, self.pred)
        self.links = [l["name"] for l in cfg["links"]]

    def _backup(self, now, links, cur, stream, req, kbps, margin):
        """Best other link for this stream: meets its limits now, is not itself
        predicted to fail soon, has room; lowest delay wins. None if no such link."""
        c = [l for l in self.links if l != cur and links[l]["alive"]
             and links[l]["rtt_p90"] <= req["max_rtt_ms"] and links[l]["loss"] <= req["max_loss"]
             and self.pred.ttu.get((l, stream), INF) > 2 * margin
             and not self.pred.avoided(now, l, stream)
             and links[l]["cap_kbps"] * self.p["headroom"] >= kbps]
        return min(c, key=lambda l: links[l]["rtt_ms"]) if c else None

    def _bulk_link(self, links):
        """Link for bulk: alive, meets the video loss limit, highest estimated
        capacity. May differ from the video link."""
        vreq = self.cfg["limits"]["video"]
        cands = [l for l in self.links if links[l]["alive"] and links[l]["loss"] <= vreq["max_loss"]]
        return max(cands, key=lambda l: links[l]["cap_kbps"]) if cands else None

    def decide(self, s):
        now, links, reasons = s["now"], s["links"], []
        uploads = s.get("uploads") or []

        # Feature 1a (Part A): mode + needs. The detector always runs: Part B
        # keys its rate rules on the real mode even when Part A is off.
        det_mode = self.mode_det.update(now, s["thr"], s["steer"], s.get("op_mode"))
        if self.f["mission"]:
            mode, needs = det_mode, self.cfg["modes"][det_mode]
        else:
            mode, needs = "FIXED", FIXED_NEEDS

        # Feature 2a: time-until-unusable for every link x stream
        self.pred.observe(now, links)
        self.pred.ttu = {}
        if self.f["predict"]:
            reqs = {"cmd": self.cfg["limits"]["cmd"]}
            for c, n in needs.items():
                reqs[c] = {"max_rtt_ms": n.get("max_rtt_ms", 600), "max_loss": n.get("max_loss", 0.2)}
            for ln in self.links:
                for st, req in reqs.items():
                    self.pred.ttu[(ln, st)] = self.pred.time_until_unusable(ln, req)

        # Feature 1 Part B: upload scheduling. Requested items go first (FIFO),
        # then background (FIFO). When the policy says urgent data outranks
        # camera quality above the minimum, it reserves its rate BEFORE
        # allocation; otherwise the cameras come first and bulk gets what is
        # left (computed after allocation).
        bulk, reserve, head, bln = None, {}, None, None
        if uploads:
            bln = self._bulk_link(links)
            if bln is not None:
                head = next((u for u in uploads if u["requested"]), uploads[0])
                urgent = (self.upload_policy == "urgent_first"
                          or (self.upload_policy == "adaptive"
                              and det_mode in ("STOPPED", "PRECISION")))
                if head["requested"] and urgent:
                    used = 30.0                   # ~20 Hz commands + telemetry (A)
                    prot = min(needs, key=lambda c: needs[c]["prio"])
                    if self.alloc.current.get(prot) == bln and needs[prot]["levels"][0] > 0:
                        used += self.cfg["ladder"][needs[prot]["levels"][0]]["kbps"]
                    rate = max(0.0, self.up["bulk_budget_share"] * links[bln]["cap_kbps"] - used)
                    bulk = {"link": bln, "mode": "rate", "rate_kbps": rate, "item": head["id"]}
                    if rate > 0:
                        reserve = {bln: rate}

        # Feature 1b (Part A): allocation
        cmd_links, video, unmet, degraded = self.alloc.allocate(
            now, links, mode, needs, self.f["predict"], reasons, reserve)

        # Feature 2b: graded actions on protected streams
        actions = {}
        if self.f["predict"]:
            m = self.pred.margin(self.mode_det.speed)
            for cam, n in needs.items():
                if n["prio"] != 1 or not video[cam]["links"]:
                    continue
                cur = video[cam]["links"][0]
                t = self.pred.ttu.get((cur, cam), INF)
                kbps = self.cfg["ladder"][video[cam]["level"]]["kbps"]
                req = {"max_rtt_ms": n.get("max_rtt_ms", 600), "max_loss": n.get("max_loss", 0.2)}
                b = self._backup(now, links, cur, cam, req, kbps, m)
                if t < m:
                    actions[cam] = "SWITCH"
                    if b:           # move now rather than waiting for next cycle
                        self.pred.avoid_until[(cur, cam)] = now + self.p["hold_s"]
                        reasons.append(f"{cam}: predicted failure of {cur} in {t:.1f}s -> switch to {b}")
                        video[cam]["links"] = [b]
                        self.alloc.current[cam] = b
                        self.alloc.switched_at[cam] = now
                elif t < 2 * m:
                    actions[cam] = "DUPLICATE"
                    if b:
                        video[cam]["links"] = [cur, b]
                        reasons.append(f"{cam}: {cur} has {t:.1f}s left -> duplicate on {b}")
                elif self.pred.scheduled_risk(cur, s.get("clock")):
                    actions[cam] = "DUPLICATE"           # clock rule: known handover moment
                    if b:
                        video[cam]["links"] = [cur, b]
                        reasons.append(f"{cam}: scheduled handover on {cur} -> duplicate on {b}")
                elif t < 3 * m:
                    actions[cam] = "PREPARE"
                else:
                    actions[cam] = "NORMAL"
            # commands are tiny: always on two links when two exist (as baseline)

        # Part B, video-first / background case: bulk gets what is left after
        # the cameras, from the same budget and engine queue guard as always
        if uploads and bln is not None and bulk is None:
            used = 30.0 + sum(self.cfg["ladder"][v["level"]]["kbps"]
                              for v in video.values() if bln in v["links"])
            rate = max(0.0, self.up["bulk_budget_share"] * links[bln]["cap_kbps"] - used)
            bulk = {"link": bln, "mode": "rate", "rate_kbps": rate, "item": head["id"]}

        return {"mode": mode, "cmd_links": cmd_links, "telem_link": cmd_links[0], "video": video,
                "bulk": bulk,                     # optional: the car ignores it
                "hb_fast": any(a != "NORMAL" for a in actions.values()),
                "actions": actions, "reasons": reasons,
                "cap": 1.0, "status": "N/A",      # graceful degradation removed in v2
                "ttu": {f"{k[0]}:{k[1]}": (round(v, 2) if v != INF else None) for k, v in self.pred.ttu.items()}}
