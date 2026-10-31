"""Check the redundancy schemes against the saved two-stream study.

The original study (EDGE Multi-Link Video.md, part 2) ran in a scratch copy of
the simulator that was not kept; only its per-run results survive in
../results_mdc_runs.csv. This prints those saved results next to the rebuilt
schemes (results/redundancy_runs.csv, same scenarios and seeds).

    python3 sim/compare_redundancy.py
    python3 sim/compare_redundancy.py --rerun     # re-run the rebuilt schemes first (~30 min on 2 cores)
"""
import argparse
import os
import sys
from multiprocessing import Pool

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from sim import run_experiments as R  # noqa: E402

SAVED = os.path.join(HERE, "..", "..", "results_mdc_runs.csv")
OURS = os.path.join(HERE, "..", "results", "redundancy_runs.csv")
NAMES = {  # rebuilt config -> name in the saved study
    "Proposed": "Proposed",
    "Proposed + base-layer copy": "Base-layer copy",
    "Proposed + full copy": "Full copy on every link",
    "Proposed + FEC": "FEC: survive one link loss",
    "Proposed + two-stream FEC": "Two-stream FEC (full survives one loss)",
    "Proposed + graded two-stream": "Two-stream graded FEC",
}
VARIANTS = ["nominal", "harsh_handovers", "fast_fade", "overlap", "overlap_harsh"]
COLS = ["useful_view_pct", "longest_interruption_s", "blind_m", "full_level_pct",
        "dup_overhead_pct", "video_delay_p95_ms"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rerun", action="store_true")
    a = ap.parse_args()
    if a.rerun:
        jobs = [(n, v, s, None) for n in NAMES for v in VARIANTS for s in range(5)]
        with Pool(os.cpu_count() or 1) as p:
            pd.DataFrame(p.map(R.one, jobs, chunksize=1)).to_csv(OURS, index=False)
    saved, ours = pd.read_csv(SAVED), pd.read_csv(OURS)
    ours["config"] = ours["config"].map(NAMES)
    for label, vs in (("overlapping coverage (overlap, overlap_harsh)", ["overlap", "overlap_harsh"]),
                      ("repo scenarios (nominal, harsh_handovers, fast_fade)", VARIANTS[:3])):
        s = saved[saved.variant.isin(vs) & saved.config.isin(NAMES.values())].groupby("config")[COLS].mean()
        o = ours[ours.variant.isin(vs)].groupby("config")[COLS].mean()
        t = pd.concat({"saved": s, "rebuilt": o}, axis=1).swaplevel(axis=1)
        print(f"--- {label}, 5 seeds each")
        print(t[[(c, k) for c in COLS for k in ("saved", "rebuilt")]].loc[list(NAMES.values())].round(1).to_string())


if __name__ == "__main__":
    main()
