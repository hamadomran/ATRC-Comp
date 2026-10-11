# EDGE multi-link controller (ATRC Challenge, EDGE track)

Baseline (3GPP ATSSS / Peplink SpeedFusion-style threshold steering) vs proposed
(mission-aware allocation + upload scheduling, predictive switching) for a
teleoperated THeMIS-class UGV with mesh radio, Starlink and 4G.

## Phase 1: simulation (what we submit)

Needs Python 3.10+ with numpy, pandas, matplotlib, pyyaml (`pip install -r requirements.txt`).

    python3 sim/run_experiments.py            # all runs, ~6 min on 4 cores (~12 on 2) -> results/*.csv
    python3 sim/run_experiments.py --seeds 3  # quick check (~1-2 min)
    python3 sim/plots.py                      # charts -> results/charts/
    python3 sim/scenario.py --table           # link ground truth at reference positions

What is real and what is simulated:
* REAL code, unchanged from the car version: `deciders/baseline.py`, `deciders/proposed.py`,
  `common/monitor.py` (heartbeat delay/loss, capacity estimator). The simulator only feeds them
  measurements and applies their decisions.
* SIMULATED: the links (`sim/scenario.py`), the vehicle's position, packets, queues, the
  operator's commands and the car's failsafe (`sim/engine.py`).
* Settings for the simulated vehicle: `config/sim_themis.yaml`.

The v2 scenario is position-based: link conditions follow where the vehicle IS
(radio fades beyond the ridge at 1.3-1.6 km, 4G is RSRP from two towers, Starlink
is jammed at 2.45-2.80 km), not mission time. The driver is a state machine over
position; a slower vehicle really does spend longer in the bad spots. The mission
also creates bulk upload work: recording and route-LiDAR chunks every 60 s as
background, plus a crater LiDAR scan and inspection stills/clips on request.
All of it shares each link's modem uplink FIFO with the video and telemetry.
Both modules shape their uploads (rate-limited below the capacity estimate with
router queue management, like CAKE autorate) so the comparison is fair. What is
compared is the ORDER inside the shaped budget: `video_first` (cameras always
win), `urgent_first` (requested data always beats extra video quality) and the
proposed `adaptive` (urgent-first while stopped or in a precision manoeuvre,
video-first while driving). `Baseline (no shaping)` keeps a greedy TCP-like
uploader as a reference. Usability is scored against an operator `requirements:`
table (frame age, delivered fps, resolution per true manoeuvre) that only the
simulator reads, never the deciders. There is no graceful degradation (speed
caps, stale-command rejection) in v2.

Files:
    sim/scenario.py         driver state machine + position-based link models + 5 harder variants
    sim/engine.py           discrete-event simulator (kinematics, queues, uploads) + metrics
    sim/run_experiments.py  configs x variants x seeds -> results/runs.csv, summary.csv, capacity.csv
    sim/plots.py            charts (timeline, headline, ablation, segments, uploads, variants, capacity)

## Phase 2 (later): the RC car

The same decision modules run on the Raspberry Pi car (`pi_main.py`, `hub.py`,
`operator_station.py`, `esp32/`, `config/run.yaml`). The car ignores the plan's
`bulk` key. Laptop loop-back test:

    python3 hub.py --config config/sim.yaml --bind 127.0.0.1 &
    python3 pi_main.py --config config/sim.yaml --decider proposed --sim --run test1 &
    python3 operator_station.py --config config/sim.yaml --run test1
    echo '{"wifi": 0.5}' > sim_impair.json     # fake 50% loss on "wifi"; '{}' to clear
