# EDGE multi-link controller (ATRC Challenge, EDGE track)

Baseline (3GPP ATSSS / Peplink SpeedFusion-style threshold steering) vs proposed
(mission-aware allocation, predictive switching, graceful degradation) for a
teleoperated THeMIS-class UGV with mesh radio, Starlink and 4G.

## Phase 1: simulation (what we submit)

Needs Python 3.10+ with numpy, pandas, matplotlib, pyyaml (`pip install -r requirements.txt`).

    python3 sim/run_experiments.py            # all runs, ~20 min on 2 cores -> results/*.csv
    python3 sim/run_experiments.py --seeds 3  # quick check (~3 min)
    python3 sim/plots.py                      # charts -> results/charts/

What is real and what is simulated:
* REAL code, unchanged from the car version: `deciders/baseline.py`, `deciders/proposed.py`,
  `common/monitor.py` (heartbeat delay/loss, capacity estimator). The simulator only feeds them
  measurements and applies their decisions.
* SIMULATED: the links (`sim/scenario.py`: a scripted 240 s mission with ground-truth link
  behaviour), packets, queues, the operator's commands and the car's failsafe (`sim/engine.py`).
* Settings for the simulated vehicle: `config/sim_themis.yaml`.

Files:
    sim/scenario.py         mission script + link models + 5 harder variants
    sim/engine.py           discrete-event simulator + metrics
    sim/run_experiments.py  configs x variants x seeds -> results/runs.csv, summary.csv, capacity.csv, caps.csv
    sim/plots.py            charts

## Phase 2 (later): the RC car

The same decision modules run on the Raspberry Pi car (`pi_main.py`, `hub.py`,
`operator_station.py`, `esp32/`, `config/run.yaml`). Laptop loop-back test:

    python3 hub.py --config config/sim.yaml --bind 127.0.0.1 &
    python3 pi_main.py --config config/sim.yaml --decider proposed --sim --run test1 &
    python3 operator_station.py --config config/sim.yaml --run test1
    echo '{"wifi": 0.5}' > sim_impair.json     # fake 50% loss on "wifi"; '{}' to clear
