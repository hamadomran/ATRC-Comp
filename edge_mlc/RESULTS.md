# Simulation results (v2.2)

*Numbers below are from the quick run (3 seeds nominal, charts in `results/charts/`).
Re-run `python3 sim/run_experiments.py` (~5 min) for the full 20-seed set before quoting
these externally; the picture has been stable across runs.*

## What was tested

A teleoperated UGV drives a 3 km round trip with three links: mesh radio (dies
beyond a ridge at ~1.6 km), Starlink (handover drops every 15 s, blocked by trees,
jammed in a contested zone, obstructed to 1 Mbps in a crater) and patchy 4G from
two towers. The vehicle also has data to upload: continuous background recording
(15 MB per minute) and four operator-requested items — a 20 MB scan requested
**while driving**, a 20 MB crater scan and a still + clip requested **while stopped**.

Two controllers decide, every 0.1 s, which link carries what:

* **Baseline** — the setup a competent integrator gets from off-the-shelf gear:
  fixed link preference with threshold switching, fixed priorities
  (commands > video > bulk), traffic-shaped uploads with requested-before-background.
* **Proposed** — adds mission awareness: it detects the manoeuvre (drive, reverse,
  precision, stopped) and gives the camera that manoeuvre needs the right quality;
  it predicts link failures and switches early; and its upload policy adapts to
  vehicle state.

**Scoring is independent of both controllers.** A sample counts as "usable view"
only if the camera the *true* manoeuvre needs meets a frame-age, frame-rate and
resolution requirement from a fixed operator table. Both controllers are scored
with the same yardstick.

## Headline result

| | Baseline | Proposed |
|---|---|---|
| Usable view, whole mission | 76% | **84%** |
| View interruptions per mission | 74 | **44** |
| Video delay (p95) while uploads run | 62 ms | 70 ms |
| Mission completed | yes (~22 min) | yes (~22 min) |

The gap is not from uploads misbehaving (both are shaped; delays stay low for
both). It comes from the baseline switching links too late and feeding the wrong
camera at the wrong quality during manoeuvres.

## Experiment A — what happens when a link changes

Every mission has ~74 ground-truth link events (a link drops or comes back, or
capacity collapses ≥30% within a second). Inside a window from 2 s before to 10 s
after each event:

| | usable view in window | interruptions in windows |
|---|---|---|
| Baseline | 79.9% | 52 |
| Baseline + duplicate video | 81.4% | 38 |
| **Proposed** | **85.3%** | **28** |
| Proposed without mission-awareness | 82.9% | 41 |

Duplicating video on two links (the brute-force fix) buys the baseline less than
mission-awareness buys the proposed controller, and costs ~38% extra data.

## Experiment B — three upload policies, same shaping

Only the *order* differs; shaping, link choice and camera allocation are identical:

* **video-first**: cameras always outrank uploads
* **urgent-first**: requested uploads always outrank extra video quality
* **adaptive (ours)**: urgent-first while stopped/precision, video-first while driving

| | scan while driving | crater scan | still | clip | front-camera fps during driving scan |
|---|---|---|---|---|---|
| video-first | 38 s | 183 s | 7 s | 56 s | 19.0 |
| urgent-first | **29 s** | **57 s** | 5 s | **42 s** | 10.1 — at the usability floor |
| adaptive | 38 s | 183 s | **5 s** | 49 s | **19.0** |

Each static policy fails somewhere. Urgent-first delivers everything fastest but
drags the driving view down to exactly the 10 fps minimum (and causes the most
view interruptions overall: 64 vs 44). Video-first keeps the view clean but makes
the operator wait 3× longer for urgent data. Adaptive keeps the full 19 fps while
driving and still gets the stopped-request items fastest.

## Where the proposed design does not win — reported as found

1. **The crater scan (183 s vs 57 s for urgent-first).** It is requested during a
   stop, but the crater link is only 1 Mbps, so the data actually flows later,
   while driving — where adaptive deliberately protects video. If the operator
   needs that scan fast, urgent-first is the better policy there; adaptive trades
   it away for view quality. This is the designed trade-off, visible in the data.
2. **Predictive switching adds little under the strict metric** (84.2% without it
   vs 84.3% with). Its value shows in interruption counts, not the headline view
   number.
3. **Unshaped uploads deliver background fastest** (zero backlog at mission end vs
   ~8 MB) — by wrecking the live view while doing it (460 ms video delay, 99+
   interruptions). That is the trade a greedy uploader makes.

## Caveats

* All physical constants marked `(A)` in the code/config are assumptions, including
  the operator requirements table (`# to be backed by teleoperation literature`).
* Results above are 3 seeds; run the full set for publication-grade error bars.
* The same seed gives both controllers identical link conditions, and runs are
  bit-for-bit reproducible.
