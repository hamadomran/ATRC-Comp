"""Run every experiment and write the results tables.

  python3 sim/run_experiments.py                 # full set: 20 seeds nominal, 10 per variant
  python3 sim/run_experiments.py --seeds 3       # quick check

It first times one nominal run per module and prints the seconds per run; if
the estimated total exceeds 2.5 h on the available cores it drops to 10 / 5
seeds and says so.

Outputs (results/):
  runs.csv          one row per run (config x variant x seed)
  summary.csv       mean and 95% confidence interval per config x variant
  capacity.csv      crater capacity sweep (baseline vs proposed)
"""
import argparse
import copy
import os
import sys
import time
from multiprocessing import Pool

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from deciders.baseline import Baseline           # noqa: E402
from deciders.proposed import Proposed           # noqa: E402
from sim import engine                           # noqa: E402
from sim.scenario import VARIANTS                # noqa: E402

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "sim_themis.yaml")))

# name -> (kind, options)
CONFIGS = {
    "Baseline":                   ("baseline", {}),
    "Baseline + duplicate video": ("baseline", {"dup_video": True}),
    "Proposed":                   ("proposed", {}),
    "Proposed - mission":         ("proposed", {"mission": False}),   # Part A off
    "Proposed - uploads":         ("proposed", {"uploads": False}),   # Part B off
    "Proposed - predict":         ("proposed", {"predict": False}),
}


def make(name):
    kind, opt = CONFIGS[name]
    cfg = copy.deepcopy(CFG)
    if kind == "baseline":
        return cfg, Baseline(cfg, dup_video=opt.get("dup_video", False))
    return cfg, Proposed(cfg, features=dict(opt))


def one(job):
    name, variant, seed, overrides = job
    cfg, dec = make(name)
    m, _ = engine.run(cfg, dec, variant, seed, overrides)
    return {"config": name, "variant": variant, "seed": seed, **(overrides or {}), **m}


def summarise(df, keys):
    num = [c for c in df.columns if c not in keys + ["seed"] and pd.api.types.is_numeric_dtype(df[c])]
    rows = []
    for k, g in df.groupby(keys, sort=False):
        r = dict(zip(keys, k if isinstance(k, tuple) else (k,)))
        r["n"] = len(g)
        for c in num:
            x = g[c].dropna()
            r[c] = x.mean()
            r[c + "_ci"] = 1.96 * x.std(ddof=1) / np.sqrt(len(x)) if len(x) > 1 else 0.0
        rows.append(r)
    return pd.DataFrame(rows)


def build_jobs(nom_seeds, var_seeds, cap_seeds):
    jobs = [(c, v, s, None) for v in VARIANTS for c in CONFIGS
            for s in range(nom_seeds if v == "nominal" else var_seeds)]
    cjobs = [(c, "nominal", s, {"crater_cap": float(k)}) for k in (750, 1000, 1500, 2500, 4000)
             for c in ("Baseline", "Proposed", "Proposed - mission") for s in range(cap_seeds)]
    return jobs, cjobs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20,
                    help="seeds for nominal (other variants get half, capacity sweep at most 5)")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    # time one nominal run of each module first
    secs = []
    for name in ("Baseline", "Proposed"):
        t0 = time.perf_counter()
        one((name, "nominal", 0, None))
        secs.append(time.perf_counter() - t0)
        print(f"{name}: {secs[-1]:.1f} s per nominal run")

    nom_seeds, var_seeds, cap_seeds = a.seeds, max(1, (a.seeds + 1) // 2), min(5, a.seeds)
    jobs, cjobs = build_jobs(nom_seeds, var_seeds, cap_seeds)
    cores = os.cpu_count() or 1
    est_h = (len(jobs) + len(cjobs)) * np.mean(secs) / cores / 3600
    print(f"{len(jobs) + len(cjobs)} runs, estimated {est_h:.1f} h on {cores} cores")
    if est_h > 2.5 and a.seeds == 20:
        nom_seeds, var_seeds = 10, 5
        jobs, cjobs = build_jobs(nom_seeds, var_seeds, cap_seeds)
        print(f"estimate above 2.5 h: dropping to {nom_seeds} seeds nominal / {var_seeds} per variant "
              f"({len(jobs) + len(cjobs)} runs)")

    with Pool(cores) as p:
        runs = pd.DataFrame(p.map(one, jobs, chunksize=1))
        cap = pd.DataFrame(p.map(one, cjobs, chunksize=1))
    runs.to_csv(os.path.join(a.out, "runs.csv"), index=False)
    summarise(runs, ["config", "variant"]).to_csv(os.path.join(a.out, "summary.csv"), index=False)
    summarise(cap, ["config", "crater_cap"]).to_csv(os.path.join(a.out, "capacity.csv"), index=False)
    cols = ["config", "useful_view_pct", "video_delay_p95_bulk_ms", "req_mean_s", "bg_done_before_end",
            "mission_time_s", "interruptions", "blind_m", "dup_overhead_pct"]
    print("--- nominal")
    print(summarise(runs[runs.variant == "nominal"], ["config"])[cols].round(1).to_string(index=False))


if __name__ == "__main__":
    main()
