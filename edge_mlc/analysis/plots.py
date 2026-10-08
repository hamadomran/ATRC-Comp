"""Charts for the deck.

  python3 analysis/plots.py timeline logs/r01 logs/r02     # one baseline + one proposed run
  python3 analysis/plots.py boxes logs/results.csv          # spread across all runs
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def load(p):
    return [json.loads(l) for l in open(p) if l.strip()]


def timeline(folders):
    fig, axes = plt.subplots(len(folders), 1, figsize=(11, 3.2 * len(folders)), sharex=True)
    axes = [axes] if len(folders) == 1 else axes
    for ax, f in zip(axes, folders):
        car = load(os.path.join(f, "car.jsonl"))
        start = next(e for e in car if e["ev"] == "start")
        ticks = [e for e in car if e["ev"] == "tick"]
        t0 = ticks[0]["t"]
        t = [e["t"] - t0 for e in ticks]
        ax.plot(t, [min(e["links"]["wifi"]["rtt_ms"], 400) for e in ticks], label="Wi-Fi delay (ms)", color="#3b6fb6")
        ax.plot(t, [100 * e["links"]["wifi"]["loss"] * 4 for e in ticks], label="Wi-Fi loss (x4, %)", color="#9bb7e0")
        on_lte = [1 if e["plan"]["video"]["front"]["links"][:1] == ["lte"] else 0 for e in ticks]
        ax.fill_between(t, 0, [400 * v for v in on_lte], color="#e8a33d", alpha=0.18, step="post", label="front video on 4G")
        rx = [e["t"] for e in car if e["ev"] == "cmd_rx"]
        first, last = (rx[0], rx[-1]) if rx else (t0, t0)
        esp = [e for e in car if e["ev"] == "esp32" and first <= e["t"] <= last]
        for a, b in zip(esp, esp[1:]):
            if a["failsafe"]:
                ax.axvspan(a["t"] - t0, b["t"] - t0, color="#c0392b", alpha=0.35, lw=0)
        ax.set_ylim(0, 420)
        ax.set_ylabel("ms")
        ax.set_title(f"{start['decider']} {start['features']}  (red = vehicle stopped by failsafe)", loc="left", fontsize=10)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time since start (s)")
    fig.tight_layout()
    fig.savefig("timeline.png", dpi=160)
    print("wrote timeline.png")


def boxes(csv):
    df = pd.read_csv(csv)
    metrics = ["max_cmd_gap_ms", "failsafe_time_s", "useful_view_pct", "wrong_view_pct",
               "video_switches", "dup_overhead_pct"]
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.5))
    for ax, m in zip(axes.flat, metrics):
        groups = sorted(df["config"].unique())
        ax.boxplot([df[df.config == g][m] for g in groups])
        ax.set_xticks(range(1, len(groups) + 1), groups)
        ax.set_title(m, fontsize=10)
        ax.tick_params(axis="x", labelsize=7, rotation=20)
    fig.tight_layout()
    fig.savefig("boxes.png", dpi=160)
    print("wrote boxes.png")


if __name__ == "__main__":
    {"timeline": lambda: timeline(sys.argv[2:]), "boxes": lambda: boxes(sys.argv[2])}[sys.argv[1]]()
