"""The simulated mission: what the driver asks for, and what each link really does.

Everything here is GROUND TRUTH. The decision modules never see it directly;
they only see what the measurement code (the real Monitor class) reports.

Mission (240 s, THeMIS-class UGV leaving base on an evacuation/resupply run):
   0- 35  leave base             radio strong, Starlink, 4G near base
  35- 65  radio range edge       radio fades gradually (beyond ~1.5 km), 4G gone by ~50 s
  65-105  transit                Starlink only; delay burst ~75 s; 4G patch 80-96 s;
                                 tree line blocks Starlink at ~88 s (sudden, 5 s)
 104-130  crater                 slow approach, REVERSE out (107-119), precision turn;
                                 Starlink partly obstructed: ~1.5 Mbps
 130-160  contested area         jamming ramps up on Starlink, then full outage 152-158 s
 160-185  inspection stop        vehicle still; operator studies the front view
 185-240  return                 4G back at ~215 s, radio back from ~200 s

Assumption (A): link conditions follow TIME along the route (as if replaying a
recorded run). If a vehicle slows down it does not change what the links do;
we measure the lost progress separately.

Variants (VARIANTS below) make one thing harder each. "overlap" and
"overlap_harsh" add 4G along the whole route at cell-edge quality, so more than
one link is usable most of the time and the choice of link at a handover matters.
"""
import numpy as np

DT = 0.01                     # ground-truth resolution (s)
T_END = 240.0

# ---------------------------------------------------------------- variants
VARIANTS = {
    "nominal":          {},
    "harsh_handovers":  {"ho_p_out": 0.8, "ho_out_s": (0.5, 2.5)},
    "heavy_jamming":    {"jam_start": 122.0, "jam_out": (148.0, 162.0)},
    "no_lte":           {"lte": False},
    "fast_fade":        {"fade": (50.0, 60.0)},
    # coverage overlaps: 4G along the whole route at cell-edge quality, so the
    # choice of link matters for handovers (the other variants are mostly
    # single-link once the radio fades)
    "overlap":          {"lte_route": True},
    "overlap_harsh":    {"lte_route": True, "ho_p_out": 0.8, "ho_out_s": (0.5, 2.5)},
}
DEFAULTS = {
    "ho_p_out": 0.35, "ho_out_s": (0.2, 1.2),     # (A) chance + length of an outage at a handover
    "jam_start": 130.0, "jam_out": (152.0, 158.0),
    "lte": True,
    "lte_route": False,                           # 4G at cell edge along the whole route (A)
    "fade": (35.0, 65.0),                         # radio fade start/end (s)
    "crater_cap": 1500.0,                         # Starlink kbps in the crater (A)
}

# ---------------------------------------------------------------- driver
# (start, end, maneuver, throttle, steer amplitude, steer offset)
DRIVER = [
    (0.0,   2.0,   "STOPPED",   0.0,  0.0, 0.0),
    (2.0,   65.0,  "DRIVE",     0.8,  0.3, 0.0),
    (65.0,  104.0, "DRIVE",     0.8,  0.3, 0.0),
    (104.0, 106.0, "PRECISION", 0.15, 0.0, 0.6),
    (106.0, 107.0, "PRECISION", 0.0,  0.0, 0.0),    # stop before reversing
    (107.0, 119.0, "REVERSE",  -0.4,  0.2, 0.0),
    (119.0, 130.0, "PRECISION", 0.25, 0.0, -0.6),
    (130.0, 160.0, "DRIVE",     0.6,  0.3, 0.0),
    (160.0, 185.0, "STOPPED",   0.0,  0.0, 0.0),
    (185.0, 240.0, "DRIVE",     0.8,  0.3, 0.0),
]


def driver(t):
    """What the operator commands at time t: (throttle, steer, maneuver)."""
    for a, b, man, thr, amp, off in DRIVER:
        if a <= t < b:
            return thr, off + amp * np.sin(2 * np.pi * t / 7.0), man
    return 0.0, 0.0, "STOPPED"


def _ramp(t, t0, t1, v0, v1):
    return np.clip(v0 + (v1 - v0) * (t - t0) / (t1 - t0), min(v0, v1), max(v0, v1))


def _ar1(rng, n, sd, tau_s):
    """Slowly varying noise (shadowing), standard deviation sd, correlation time tau."""
    a = np.exp(-DT / tau_s)
    e = rng.normal(0, sd * np.sqrt(1 - a * a), n)
    x = np.zeros(n)
    for i in range(1, n):
        x[i] = a * x[i - 1] + e[i]
    return x


PHASES = [(3, 35, "leave base"), (35, 65, "radio edge"), (65, 104, "transit"),
          (104, 130, "crater"), (130, 160, "contested"), (160, 185, "inspection"), (185, 240, "return")]


def build(variant="nominal", seed=0, overrides=None):
    """Return ground truth arrays for every link on a DT grid."""
    v = {**DEFAULTS, **VARIANTS[variant], **(overrides or {})}
    rng = np.random.default_rng(seed)
    t = np.arange(0, T_END + 1, DT)
    n = len(t)
    out = {"t": t}

    # ---------------- radio (mesh to base): everything follows signal strength
    f0, f1 = v["fade"]
    s = np.full(n, -62.0)
    s = np.where(t >= f0, _ramp(t, f0, f1, -65, -102), s)
    s = np.where(t >= f1, -105.0, s)
    s = np.where(t >= 200, _ramp(t, 200, 215, -102, -66), s)
    s = s + _ar1(rng, n, 2.0, 3.0)                       # shadowing (A)
    loss = 0.002 + 0.5 / (1 + np.exp((s + 92) / 1.5))    # (A) loss vs signal
    out["radio"] = dict(
        up=s > -97, signal=s, loss=loss,
        owd=10 + np.maximum(0, -82 - s) * 4,             # one-way delay ms (A)
        jit=2 + np.maximum(0, -85 - s) * 2,
        cap=8000 * np.clip((s + 96) / 20, 0.05, 1.0),
        buffer_ms=150.0,
    )

    # ---------------- Starlink
    up = np.ones(n, bool)
    owd = np.full(n, 22.0)
    jit = np.full(n, 4.0)
    sl_loss = np.full(n, 0.003)
    cap = np.full(n, 8000.0)
    # handovers every 15 s: delay spike, sometimes a short outage (moving vehicle)
    for k in range(0, int(T_END) + 15, 15):
        h = 12 + k
        m = (t >= h) & (t < h + 0.5)
        owd[m] += 40
        if rng.random() < v["ho_p_out"]:
            d = rng.uniform(*v["ho_out_s"])
            up[(t >= h) & (t < h + d)] = False
    m = (t >= 75) & (t < 78)                             # gateway congestion: delay burst
    owd[m] += 325
    up[(t >= 88) & (t < 93)] = False                     # tree line: sudden blockage
    m = (t >= 104) & (t < 130)                           # crater: partial obstruction
    cap[m] = v["crater_cap"]
    sl_loss[m] = 0.01
    owd[m] += 10
    j0, (o0, o1) = v["jam_start"], v["jam_out"]          # jamming ramp, then outage
    m = (t >= j0) & (t < o0)
    x = (t[m] - j0) / (o0 - j0)
    sl_loss[m] = 0.003 + 0.15 * x
    owd[m] += 150 * x
    cap[m] = 8000 - 7200 * x
    up[(t >= o0) & (t < o1)] = False
    out["starlink"] = dict(up=up, signal=None, loss=sl_loss, owd=owd, jit=jit, cap=cap, buffer_ms=300.0)

    # ---------------- 4G (quality % drives everything)
    q = np.zeros(n)
    if v["lte"]:
        q = np.where(t < 40, 65.0, q)
        q = np.where((t >= 40) & (t < 50), _ramp(t, 40, 50, 65, 0), q)
        q = np.where((t >= 80) & (t < 96), np.minimum(_ramp(t, 80, 83, 0, 55), _ramp(t, 93, 96, 55, 0)), q)
        q = np.where(t >= 215, _ramp(t, 215, 220, 0, 60), q)
        q = np.clip(q + _ar1(rng, n, 3.0, 2.0) * (q > 0), 0, 100)
    if v["lte_route"]:
        # (A) slow fades in and out of usability (quality 8-20 %), good near base
        r2 = np.random.default_rng(seed + 77)     # own stream: other links unchanged
        q = 30 + _ar1(r2, n, 14.0, 6.0) + 8 * np.sin(2 * np.pi * t / 45 + r2.uniform(0, 6))
        q = np.where((t < 40) | (t >= 215), np.maximum(q, 60), q)
        q = np.clip(q, 0, 100)
    out["lte"] = dict(
        up=q > 8, signal=q,
        loss=0.002 + 0.4 / (1 + np.exp((q - 15) / 3)),
        owd=30 + np.maximum(0, 30 - q) * 3,
        jit=np.full(n, 8.0),
        cap=5000 * np.clip(q / 50, 0.05, 1.0),
        buffer_ms=600.0,
    )
    return out
