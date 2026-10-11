"""The simulated mission, v2: link conditions follow the vehicle's POSITION.

Everything here is GROUND TRUTH. The decision modules never see it directly;
they only see what the measurement code (the real Monitor class) reports.

Mission (THeMIS-class UGV leaving base on an evacuation/resupply run, ~20 min):
  0.0-1.3 km  leave base          radio strong, 4G from the base tower near base
  1.3-1.6 km  radio range edge    radio fades over the ridge, 4G long gone
  1.6-2.35 km transit             Starlink only; tree cover kills it at 2.10-2.13;
                                  4G comes back near the village tower at 2.0 km
  2.35-2.45   crater              slow approach, LiDAR scan, REVERSE out, turn;
                                  Starlink partly obstructed: ~1.5 Mbps
  2.45-2.80   contested area      jamming ramps up on Starlink; one 6 s blackout
                                  the first time the vehicle passes 2.75 km
  3.0 km      inspection site     25 s stop; stills/clips requested, 4K recording
                                  and route LiDAR queued as background uploads
  return      same route home     mission ends at 1.4 km on the way back

The driver is a state machine over position (and time, for the stops): class
Driver below.  Link ground truth is a function link_state(link, t, d, ctx):
position decides the radio and 4G signal and the jamming; time decides the
Starlink handovers and the congestion burst; vehicle state decides the crater
obstruction and the jammer blackout.

Randomness (shadowing, handover outages) is pre-generated from the run seed so
both decision modules see identical conditions and re-runs are identical.
"""
import argparse
import math

import numpy as np

DT = 0.01                     # time grid for shadowing + Starlink schedule (s)
T_MAX = 2400.0                # hard stop: 40 min of sim time (A)

# ---------------------------------------------------------------- variants
VARIANTS = {
    "nominal":          {},
    "harsh_handovers":  {"ho_p_out": 0.8, "ho_out_s": (0.5, 2.5)},
    "heavy_jamming":    {"jam_zone": (2.40, 2.85), "jam_ramp_km": 0.25, "blackout_s": 12.0},
    "no_lte":           {"lte": False},
    "fast_fade":        {"fade": (1.50, 1.60)},
}
DEFAULTS = {
    "ho_p_out": 0.35, "ho_out_s": (0.2, 1.2),   # (A) chance + length of an outage at a Starlink handover
    "jam_zone": (2.45, 2.80),                   # (A) jamming zone (km), both directions
    "jam_ramp_km": 0.30,                        # (A) jamming ramps up over this distance
    "blackout_at": 2.75,                        # (A) jammer blackout: first time d >= this (km)
    "blackout_s": 6.0,                          # (A) blackout length, once per mission
    "lte": True,
    "fade": (1.30, 1.60),                       # (A) radio -65 -> -100 dBm over these km (terrain)
    "crater_cap": 1000.0,                       # (A) Starlink kbps in the crater: genuinely
                                                #     too little for two cameras
    "buffer_scale": 1.0,                        # (A) scales Starlink + 4G modem buffers (sensitivity)
}

# ---------------------------------------------------------------- driver
CRATER_STATES = {"CRATER_APPROACH", "CRATER_STOP1", "CRATER_REVERSE", "CRATER_STOP2", "CRATER_TURN"}


class Driver:
    """The operator's driving script as a position-driven state machine.
    The engine calls step(t, d) every 0.05 s with the TRUE position d (km).
    Returns (throttle, steer, manoeuvre label for scoring, upload events, done)."""

    # state -> (throttle, steer sine amplitude, steer offset, manoeuvre label)
    OUT = {
        "START":           (0.0,  0.0, 0.0,  "STOPPED"),
        "CRUISE_OUT":      (0.8,  0.3, 0.0,  "DRIVE"),
        "CRATER_APPROACH": (0.15, 0.0, 0.6,  "PRECISION"),
        "CRATER_STOP1":    (0.0,  0.0, 0.0,  "PRECISION"),
        "CRATER_REVERSE":  (-0.4, 0.2, 0.0,  "REVERSE"),
        "CRATER_STOP2":    (0.0,  0.0, 0.0,  "PRECISION"),
        "CRATER_TURN":     (0.25, 0.0, -0.6, "PRECISION"),
        "CRUISE_TO_ZONE":  (0.8,  0.3, 0.0,  "DRIVE"),
        "CONTESTED_OUT":   (0.6,  0.3, 0.0,  "DRIVE"),
        "CRUISE_TO_SITE":  (0.8,  0.3, 0.0,  "DRIVE"),
        "INSPECT":         (0.0,  0.0, 0.0,  "STOPPED"),
        "TURN_AROUND":     (0.0,  0.0, 0.0,  "STOPPED"),
        "RETURN":          (0.8,  0.3, 0.0,  "DRIVE"),
        "END":             (0.0,  0.0, 0.0,  "STOPPED"),
    }

    # upload items REQUESTED on entering a state: (id, kind, MB, requested).
    # Background data is not event-driven: the vehicle adds a 10 MB recording
    # chunk and a 5 MB LiDAR chunk every 60 s (sim/engine.py).
    # sizes (A); 1 MB = 8000 kbit
    EVENTS = {
        "CRATER_STOP1": [("lidar_crater", "lidar", 20, True)],
        "INSPECT":      [("still", "still", 4, True), ("clip", "clip", 30, True)],
    }

    def __init__(self):
        self.state, self.since = "START", 0.0
        self.direction = 1              # +1 outbound, -1 on the return leg
        self.in_crater = False
        self.scan_fired = False         # one-shot scan_drive request at 1.70 km

    def _next(self, t, d):
        s, dt_ = self.state, t - self.since
        if s == "START" and dt_ >= 2:            return "CRUISE_OUT"
        if s == "CRUISE_OUT" and d >= 2.35:      return "CRATER_APPROACH"
        if s == "CRATER_APPROACH" and d >= 2.40: return "CRATER_STOP1"
        if s == "CRATER_STOP1" and dt_ >= 1:     return "CRATER_REVERSE"
        if s == "CRATER_REVERSE" and dt_ >= 12:  return "CRATER_STOP2"
        if s == "CRATER_STOP2" and dt_ >= 1:     return "CRATER_TURN"
        if s == "CRATER_TURN" and dt_ >= 10:     return "CRUISE_TO_ZONE"
        if s == "CRUISE_TO_ZONE" and d >= 2.45:  return "CONTESTED_OUT"
        if s == "CONTESTED_OUT" and d >= 2.80:   return "CRUISE_TO_SITE"
        if s == "CRUISE_TO_SITE" and d >= 3.00:  return "INSPECT"
        if s == "INSPECT" and dt_ >= 25:         return "TURN_AROUND"
        if s == "TURN_AROUND" and dt_ >= 3:      return "RETURN"
        if s == "RETURN" and d <= 1.40:          return "END"
        return None

    def step(self, t, d):
        ev = []
        # requested scan WHILE DRIVING: first time past 1.70 km outbound,
        # where Starlink is the only link (A)
        if not self.scan_fired and self.direction > 0 and d >= 1.70:
            self.scan_fired = True
            ev.append({"id": "scan_drive", "kind": "scan", "size_kbit": 20 * 8000.0, "requested": True})
        nxt = self._next(t, d)
        if nxt:
            self.state, self.since = nxt, t
            if nxt == "RETURN":
                self.direction = -1               # turn-around done: head home
            for (i, k, mb, req) in self.EVENTS.get(nxt, []):
                ev.append({"id": i, "kind": k, "size_kbit": mb * 8000.0, "requested": req})
        self.in_crater = self.state in CRATER_STATES
        thr, amp, off, man = self.OUT[self.state]
        if self.state == "RETURN" and 2.45 <= d <= 2.80:
            thr = 0.6                             # slower back through the contested zone
        steer = off + amp * math.sin(2 * math.pi * t / 7.0)
        return thr, steer, man, ev, self.state == "END"


# ---------------------------------------------------------------- segments for scoring
SEGMENTS = ["leave base", "radio edge", "transit", "crater", "contested out",
            "inspection", "contested back", "transit back", "radio back"]


def segment(d, direction, state):
    """Scoring segment of a sample, from position, direction and driver state.
    Samples between the named stretches (e.g. 2.35-2.45 outside the crater
    states, or beyond 2.80 outbound) belong to no segment."""
    if state in CRATER_STATES:
        return "crater"
    if state in ("INSPECT", "TURN_AROUND"):
        return "inspection"
    if direction > 0:
        if d < 1.3:
            return "leave base"
        if d < 1.6:
            return "radio edge"
        if d < 2.35:
            return "transit"
        if 2.45 <= d <= 2.80:
            return "contested out"
        return None
    if 2.45 <= d <= 2.80:
        return "contested back"
    if 1.6 < d < 2.45:
        return "transit back"
    if 1.4 <= d <= 1.6:
        return "radio back"
    return None


# ---------------------------------------------------------------- ground truth
def _ar1(rng, n, sd, tau_s):
    """Slowly varying noise (shadowing), standard deviation sd, correlation time tau."""
    a = np.exp(-DT / tau_s)
    e = rng.normal(0, sd * np.sqrt(1 - a * a), n)
    x = np.zeros(n)
    for i in range(1, n):
        x[i] = a * x[i - 1] + e[i]
    return x


def make_ctx(variant="nominal", seed=0, overrides=None, shadow=True):
    """Per-run ground-truth context: variant parameters, pre-generated random
    processes (so both modules see identical conditions for the same seed) and
    the mutable vehicle state (in_crater, blackout) the engine keeps updated."""
    v = {**DEFAULTS, **VARIANTS[variant], **(overrides or {})}
    n = int(T_MAX / DT) + 1
    if shadow:
        sh_radio = _ar1(np.random.default_rng([seed, 1]), n, 2.0, 3.0)   # (A) sd 2 dB, tau 3 s
        sh_lte = _ar1(np.random.default_rng([seed, 2]), n, 3.0, 2.0)     # (A) sd 3 dB, tau 2 s
    else:
        sh_radio = sh_lte = np.zeros(n)
    # Starlink handovers at seconds 12, 27, 42, 57 of every minute: +40 ms owd
    # for 0.5 s, sometimes a short outage (pre-generated from the seed)
    ho_owd = np.zeros(n)
    ho_down = np.zeros(n, bool)
    rng = np.random.default_rng([seed, 3])
    for m in range(int(T_MAX // 60) + 1):
        for off in (12, 27, 42, 57):
            h = m * 60 + off
            i0 = int(h / DT)
            if i0 >= n:
                continue
            ho_owd[i0:int((h + 0.5) / DT)] += 40.0
            if rng.random() < v["ho_p_out"]:
                dur = rng.uniform(*v["ho_out_s"])
                ho_down[i0:int((h + dur) / DT)] = True
    ho_owd[int(430.0 / DT):int(433.0 / DT)] += 325.0   # (A) gateway congestion burst at t=430 s
    return {"v": v, "sh_radio": sh_radio, "sh_lte": sh_lte, "ho_owd": ho_owd, "ho_down": ho_down,
            "in_crater": False, "blackout_t": None}


def link_state(link, t, d, ctx):
    """Ground truth of one link at time t (s) and position d (km):
    {up, signal, loss, owd_ms, jit_ms, cap_kbps, buffer_ms}."""
    v = ctx["v"]
    i = min(int(t / DT), len(ctx["sh_radio"]) - 1)

    if link == "radio":
        f0, f1 = v["fade"]
        if d <= 1.3:
            s = -55.0 - 10.0 * d / 1.3
        elif d <= f0:
            s = -65.0
        elif d <= f1:
            s = -65.0 - 35.0 * (d - f0) / (f1 - f0)
        else:
            s = -105.0                             # (A) terrain beyond the ridge
        s += ctx["sh_radio"][i]
        return dict(
            up=s > -97, signal=s,
            loss=0.002 + 0.5 / (1 + math.exp((s + 92) / 1.5)),
            owd_ms=10 + max(0.0, -82 - s) * 4,
            jit_ms=2 + max(0.0, -85 - s) * 2,
            cap_kbps=8000 * min(max((s + 96) / 20, 0.05), 1.0),
            buffer_ms=150.0)

    if link == "lte":
        if not v["lte"]:
            return dict(up=False, signal=None, loss=1.0, owd_ms=9999.0, jit_ms=8.0,
                        cap_kbps=0.0, buffer_ms=600.0)
        r1 = -80.0 - 39.0 * math.log10(max(d, 0.1) / 0.1)   # (A) base tower
        r2 = -90.0 - 128.0 * abs(d - 2.0)                   # (A) village tower at 2.0 km
        rsrp = max(r1, r2) + ctx["sh_lte"][i]
        # (A) noise floor ~ -125 dBm per resource element; one quiet tower, no
        # interference, so RSRQ is near-constant and not modelled
        sinr = rsrp + 125.0
        return dict(
            up=rsrp > -122, signal=rsrp,
            loss=0.002 + 0.4 / (1 + math.exp((sinr - 6) / 1.2)),
            owd_ms=30 + max(0.0, 15 - sinr) * 3,
            jit_ms=8.0,
            cap_kbps=5000 * min(max((sinr - 3) / 17, 0.05), 1.0),
            buffer_ms=600.0 * v["buffer_scale"])

    # ---- starlink (no signal reported); effects applied in this order:
    up, loss, owd, cap = True, 0.003, 22.0, 8000.0
    owd += ctx["ho_owd"][i]                       # handovers + congestion burst (time)
    if ctx["ho_down"][i]:
        up = False
    if 2.10 <= d <= 2.13:
        up = False                                # (A) dense tree cover (position)
    if ctx["in_crater"]:                          # crater obstruction (vehicle state)
        cap, loss, owd = v["crater_cap"], 0.01, owd + 10
    z0, z1 = v["jam_zone"]
    if z0 <= d <= z1:                             # jamming (position, both directions)
        x = min(max((d - z0) / v["jam_ramp_km"], 0.0), 1.0)
        loss = 0.003 + 0.15 * x
        owd += 150 * x
        cap = 8000 - 7200 * x
    bt = ctx["blackout_t"]                        # jammer blackout (position-triggered, once)
    if bt is not None and bt <= t < bt + v["blackout_s"]:
        up = False
    return dict(up=up, signal=None, loss=loss, owd_ms=owd, jit_ms=4.0, cap_kbps=cap,
                buffer_ms=300.0 * v["buffer_scale"])


# ---------------------------------------------------------------- self check
def _table():
    """Print each link at reference positions with shadowing set to 0."""
    ctx = make_ctx("nominal", 0, shadow=False)
    spots = [0.1, 0.5, 0.9, 1.2, 1.4, 1.5, 1.55, 1.6, 1.8, 1.9, 2.0, 2.11, 2.2,
             (2.4, True), 2.55, 2.65, 2.77, 3.0]
    print(f"{'pos km':>8}  {'radio':<34}{'4G (RSRP)':<34}{'starlink':<34}")
    for spot in spots:
        d, crater = spot if isinstance(spot, tuple) else (spot, False)
        ctx["in_crater"] = crater
        cols = [f"{d:5.2f}" + ("*" if crater else " ")]
        for ln in ("radio", "lte", "starlink"):
            L = link_state(ln, 5.0, d, ctx)       # t=5 s: no handover, no burst
            if not L["up"]:
                cols.append("down")
            else:
                sig = "" if L["signal"] is None else f"{L['signal']:.0f} dBm, "
                cols.append(f"{sig}{100 * L['loss']:.1f}%, {2 * L['owd_ms']:.0f} ms, "
                            f"{L['cap_kbps'] / 1000:.1f} Mbps")
        print(f"{cols[0]:>8}  {cols[1]:<34}{cols[2]:<34}{cols[3]:<34}")
    print("* = in crater (Starlink obstructed). Jammer blackout not shown: it is")
    print("position-triggered (first time d >= 2.75) and lasts 6 s.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--table", action="store_true", help="print link truth at reference positions")
    if ap.parse_args().table:
        _table()
