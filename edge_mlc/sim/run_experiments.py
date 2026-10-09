"""Run every experiment and write the results tables.

  python3 sim/run_experiments.py                 # full set (~10-15 min on 2 cores)
  python3 sim/run_experiments.py --seeds 3       # quick check

Outputs (results/):
  runs.csv          one row per run (config x variant x seed)
  summary.csv       mean and 95% confidence interval per config x variant
  capacity.csv      crater capacity sweep (baseline vs proposed)
  caps.csv          speed-cap sensitivity (progress vs driving without the needed view)
"""
import argparse
import copy
import os
import sys
from multiprocessing import Pool

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from deciders.baseline import Baseline           # noqa: E402
from deciders.proposed import Proposed           # noqa: E402
from sim import engine                           # noqa: E402
from sim.oracles import OracleAllUp, OracleLinkChoice  # noqa: E402
from sim.scenario import VARIANTS                # noqa: E402

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "sim_themis.yaml")))

# name -> (kind, options)
CONFIGS = {
    "Baseline":                 ("baseline", {"bad_after": 3}),
    "Baseline (fast, 1 check)": ("baseline", {"bad_after": 1}),
    "Baseline (5 checks)":      ("baseline", {"bad_after": 5}),
    "Baseline (slow, 8 checks)": ("baseline", {"bad_after": 8}),
    "Baseline + duplicate video": ("baseline", {"bad_after": 3, "dup_video": True}),
    "Proposed":                 ("proposed", {}),
    "Proposed - mission":       ("proposed", {"mission": False}),
    "Proposed - predict":       ("proposed", {"predict": False}),
    "Proposed - degrade":       ("proposed", {"degrade": False}),
    # upper bounds (read ground truth, not deployable): what a perfect handover could gain
    "Oracle: perfect link choice": ("oracle", {"cls": OracleLinkChoice}),
    "Oracle: every up link":    ("oracle", {"cls": OracleAllUp}),
}


def make(name):
    kind, opt = CONFIGS[name]
    cfg = copy.deepcopy(CFG)
    if kind == "baseline":
        cfg["baseline"]["bad_after"] = opt["bad_after"]
        return cfg, Baseline(cfg, dup_video=opt.get("dup_video", False))
    if kind == "oracle":
        return cfg, opt["cls"](cfg)
    if "caps" in opt:                                  # speed-cap sensitivity runs
        cfg["proposed"]["caps"] = dict(opt["caps"])
        return cfg, Proposed(cfg)
    return cfg, Proposed(cfg, features={k: v for k, v in opt.items()})


# speed caps (CAUTION, DEGRADED) tried in the sensitivity sweep; default is (0.6, 0.3)
CAP_SWEEP = [(0.6, 0.3), (0.8, 0.5), (0.9, 0.7), (1.0, 1.0)]
for c, d in CAP_SWEEP:
    CONFIGS[f"Proposed caps {c}/{d}"] = ("proposed", {"caps": {"OK": 1.0, "CAUTION": c, "DEGRADED": d,
                                                                "UNAVAILABLE": 0.0}})
MAIN = [k for k in CONFIGS if not k.startswith("Proposed caps")]


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--out", default="results")
    ap.add_argument("--part", default="all", choices=["all", "main", "caps"])
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.part in ("all", "caps"):
        sj = [(c, "nominal", s, None) for c in CONFIGS if c.startswith("Proposed caps") for s in range(a.seeds)]
        with Pool(os.cpu_count()) as p:
            sw = pd.DataFrame(p.map(one, sj, chunksize=4))
        summarise(sw, ["config"]).to_csv(os.path.join(a.out, "caps.csv"), index=False)
        if a.part == "caps":
            return

    jobs = [(c, v, s, None) for v in VARIANTS for c in MAIN for s in range(a.seeds)]
    caps = [750, 1000, 1500, 2500, 4000]
    cjobs = [(c, "nominal", s, {"crater_cap": float(k)}) for k in caps
             for c in ("Baseline", "Proposed", "Proposed - mission") for s in range(max(3, a.seeds // 2))]
    with Pool(os.cpu_count()) as p:
        runs = pd.DataFrame(p.map(one, jobs, chunksize=4))
        cap = pd.DataFrame(p.map(one, cjobs, chunksize=4))
    runs.to_csv(os.path.join(a.out, "runs.csv"), index=False)
    summarise(runs, ["config", "variant"]).to_csv(os.path.join(a.out, "summary.csv"), index=False)
    summarise(cap, ["config", "crater_cap"]).to_csv(os.path.join(a.out, "capacity.csv"), index=False)
    cols = ["config", "useful_view_pct", "blind_m", "progress_pct", "stale_exec", "interruptions",
            "false_switches", "dup_overhead_pct"]
    for v in ("nominal", "overlap"):
        print(f"--- {v}")
        print(summarise(runs[runs.variant == v], ["config"])[cols].round(1).to_string(index=False))


if __name__ == "__main__":
    main()
