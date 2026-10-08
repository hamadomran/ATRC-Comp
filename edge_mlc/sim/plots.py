"""Charts for the report / deck.

  python3 sim/plots.py --res results --out results/charts
"""
import argparse
import copy
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt               # noqa: E402
import matplotlib.ticker                      # noqa: E402
import numpy as np                            # noqa: E402
import pandas as pd                           # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim import engine                        # noqa: E402
from sim.run_experiments import CFG, make     # noqa: E402
from sim.scenario import PHASES, build        # noqa: E402

C = {"Baseline": "#8a8f98", "Baseline + duplicate video": "#c3b28f", "Proposed": "#1764c0",
     "Proposed - mission": "#9cc0ea", "Proposed - predict": "#6fa3de", "Proposed - degrade": "#3f83d1",
     "Baseline (fast, 1 check)": "#b5b9bf", "Baseline (5 checks)": "#70757d", "Baseline (slow, 8 checks)": "#4c5057"}
LINKC = {"radio": "#5b8c5a", "starlink": "#7a5ea8", "lte": "#d08a3c"}
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.titleweight": "bold", "axes.titlesize": 11, "figure.dpi": 150})


def bars(ax, df, configs, col, title, ylabel, fmt="{:.0f}", better=None):
    d = df.set_index("config").loc[configs]
    x = np.arange(len(configs))
    ax.bar(x, d[col], yerr=d[col + "_ci"], color=[C[c] for c in configs], capsize=3, width=0.65,
           error_kw={"elinewidth": 1, "ecolor": "#333"})
    for i, v in enumerate(d[col]):
        ax.text(i, v + d[col + "_ci"].iloc[i] + 0.02 * max(d[col].max(), 1e-9), fmt.format(v),
                ha="center", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels([c.replace("Baseline + duplicate video", "Baseline\n+ dup. video")
                        .replace("Proposed - ", "without\n") for c in configs], fontsize=8.5)
    ax.set_title(title + (f"  ({better})" if better else ""), loc="left")
    ax.set_ylabel(ylabel)
    ax.set_ylim(0, (d[col] + d[col + "_ci"]).max() * 1.18 + 1e-9)


def headline(S, out, configs, name, sup):
    nom = S[S.variant == "nominal"]
    panels = [("useful_view_pct", "Needed view available", "% of mission", "{:.1f}", "higher is better"),
              ("interruptions", "View interruptions", "count per mission", "{:.1f}", "lower"),
              ("blind_m", "Driven without needed view", "metres", "{:.1f}", "lower"),
              ("stale_exec", "Old commands executed (>200 ms)", "count", "{:.0f}", "lower"),
              ("progress_pct", "Mission progress", "% of commanded distance", "{:.0f}", "higher"),
              ("dup_overhead_pct", "Extra data sent (duplication)", "% of all data", "{:.1f}", "lower")]
    fig, axs = plt.subplots(2, 3, figsize=(12, 6.6))
    for ax, (col, t, yl, f, b) in zip(axs.flat, panels):
        bars(ax, nom, configs, col, t, yl, f, b)
    fig.suptitle(sup, x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.text(0.01, 0.005, "Nominal mission, 20 runs each (different random seeds). Error bars: 95% confidence interval.",
             fontsize=8, color="#555")
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    fig.savefig(os.path.join(out, name))
    plt.close(fig)


def variants(S, out):
    cfgs = ["Baseline", "Baseline + duplicate video", "Proposed"]
    vs = ["nominal", "harsh_handovers", "heavy_jamming", "no_lte", "fast_fade"]
    fig, axs = plt.subplots(1, 2, figsize=(12, 3.8))
    for ax, col, t in [(axs[0], "useful_view_pct", "Needed view available (%)"),
                       (axs[1], "blind_m", "Metres driven without needed view")]:
        w = 0.26
        for i, c in enumerate(cfgs):
            d = S[S.config == c].set_index("variant").loc[vs]
            ax.bar(np.arange(len(vs)) + (i - 1) * w, d[col], w, yerr=d[col + "_ci"], color=C[c], label=c,
                   capsize=2, error_kw={"elinewidth": 0.8})
        ax.set_xticks(np.arange(len(vs)))
        ax.set_xticklabels([v.replace("_", "\n").replace("lte", "4G") for v in vs])
        ax.set_title(t, loc="left")
        if col == "useful_view_pct":
            ax.set_ylim(60, 100)
    axs[1].legend(frameon=False, fontsize=8, loc="upper right")
    fig.suptitle("Same result under harder conditions", x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(out, "variants.png"))
    plt.close(fig)


def capacity(Cp, out):
    fig, ax = plt.subplots(figsize=(6.5, 3.8))
    for c in ["Baseline", "Proposed - mission", "Proposed"]:
        d = Cp[Cp.config == c].sort_values("crater_cap")
        ax.errorbar(d.crater_cap / 1000, d["useful_crater"], yerr=d["useful_crater_ci"], marker="o",
                    color=C[c], label=c.replace("Proposed - mission", "Proposed without mission-awareness"),
                    capsize=3, lw=2)
    ax.set_xscale("log")
    ax.set_xticks([0.75, 1, 1.5, 2.5, 4])
    ax.set_xticklabels(["0.75", "1", "1.5", "2.5", "4"])
    ax.xaxis.set_minor_locator(matplotlib.ticker.NullLocator())
    ax.set_xlabel("Satellite capacity in the crater (Mbps)")
    ax.set_ylabel("Needed view available in crater (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Thin links: giving capacity to the camera that matters", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "capacity.png"))
    plt.close(fig)


def cost(S, out):
    nom = S[S.variant == "nominal"]
    fig, ax = plt.subplots(figsize=(6.5, 4))
    for _, r in nom.iterrows():
        ax.errorbar(r.dup_overhead_pct, r.interruptions, xerr=r.dup_overhead_pct_ci, yerr=r.interruptions_ci,
                    fmt="o", color=C[r.config], ms=8, capsize=2)
        ax.annotate(r.config.replace("Baseline + duplicate video", "Baseline + dup."), (r.dup_overhead_pct, r.interruptions),
                    xytext=(6, 4), textcoords="offset points", fontsize=7.5)
    ax.set_xlabel("Extra data sent for duplication (% of all data)")
    ax.set_ylabel("View interruptions per mission")
    ax.set_title("Continuity per unit of extra data", loc="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "cost_vs_continuity.png"))
    plt.close(fig)


def phases(S, out):
    nom = S[S.variant == "nominal"].set_index("config")
    names = [p[2] for p in PHASES]
    fig, ax = plt.subplots(figsize=(10, 3.6))
    w = 0.38
    for i, c in enumerate(["Baseline", "Proposed"]):
        v = [nom.loc[c, f"useful_{n}"] for n in names]
        e = [nom.loc[c, f"useful_{n}_ci"] for n in names]
        ax.bar(np.arange(len(names)) + (i - 0.5) * w, v, w, yerr=e, color=C[c], label=c, capsize=2)
    ax.set_xticks(np.arange(len(names)))
    ax.set_xticklabels(names)
    ax.set_ylim(40, 103)
    ax.set_ylabel("Needed view available (%)")
    ax.set_title("Where the difference comes from (by mission phase)", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "phases.png"))
    plt.close(fig)


def caps(K, S, out):
    nom = S[S.variant == "nominal"].set_index("config")
    fig, ax = plt.subplots(figsize=(6.5, 4))
    K = K.sort_values("progress_pct")
    ax.plot(K.progress_pct, K.blind_m, "-o", color=C["Proposed"], lw=2, label="Proposed (speed caps varied)")
    for _, r in K.iterrows():
        lab = r.config.replace("Proposed caps ", "")
        ax.annotate(lab + (" (default)" if lab == "0.6/0.3" else ""), (r.progress_pct, r.blind_m),
                    xytext=(0, -14 if lab in ("0.6/0.3", "0.9/0.7") else 8), textcoords="offset points",
                    fontsize=7.5, ha="center")
    ax.text(0.02, 0.62, "labels: speed cap when CAUTION / DEGRADED\n(1.0/1.0 = never slow down)",
            transform=ax.transAxes, fontsize=7.5, color="#555")
    b = nom.loc["Baseline"]
    ax.errorbar(b.progress_pct, b.blind_m, xerr=b.progress_pct_ci, yerr=b.blind_m_ci, fmt="s", ms=8,
                color=C["Baseline"], label="Baseline", capsize=2)
    ax.set_xlabel("Mission progress (% of commanded distance)")
    ax.set_ylabel("Metres driven without the needed view")
    ax.set_title("Speed cap is a dial: safety vs speed", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "speed_caps.png"))
    plt.close(fig)


def timeline(out, seed=0):
    truth = build("nominal", seed)
    runs = {}
    for name in ("Baseline", "Proposed"):
        cfg, dec = make(name)
        runs[name] = engine.run(copy.deepcopy(cfg), dec, "nominal", seed)[1]
    fig, axs = plt.subplots(4, 1, figsize=(12, 7.2), sharex=True,
                            gridspec_kw={"height_ratios": [1.1, 1, 1, 1.2]})
    t = truth["t"]
    ax = axs[0]
    for i, ln in enumerate(["radio", "starlink", "lte"]):
        up = truth[ln]["up"] & (truth[ln]["loss"] < 0.05) & (2 * truth[ln]["owd"] < 250)
        ax.fill_between(t, i - 0.35, i + 0.35, where=up, color=LINKC[ln], step="mid", lw=0)
        weak = truth[ln]["up"] & ~up
        ax.fill_between(t, i - 0.35, i + 0.35, where=weak, color=LINKC[ln], alpha=0.3, step="mid", lw=0)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["Radio", "Starlink", "4G"])
    ax.set_title("Links (ground truth: solid = good, faint = up but lossy or slow, blank = down)", loc="left")
    for a, b, n in PHASES:
        for x in axs:
            x.axvline(a, color="#ddd", lw=0.8, zorder=0)
        ax.text((a + b) / 2, 2.75, n, ha="center", fontsize=8, color="#444")
    ax.set_ylim(-0.6, 3.1)
    for ax, name in zip(axs[1:3], ["Baseline", "Proposed"]):
        R = runs[name]
        tt = np.array([r["t"] for r in R])
        lv = np.array([r["level"] for r in R])
        ok = np.array([r["useful"] for r in R])
        ax.fill_between(tt, 0, 5.5, where=~ok, color="#e35d5d", alpha=0.25, step="mid", lw=0,
                        label="needed view missing")
        ax.step(tt, lv, where="mid", color=C[name], lw=1.2)
        ax.set_ylim(0, 5.6)
        ax.set_yticks([0, 2, 4])
        ax.set_ylabel("level")
        ax.set_title(f"{name}: quality of the camera the manoeuvre needs", loc="left")
    axs[1].legend(frameon=False, fontsize=8, loc="lower left")
    ax = axs[3]
    R = runs["Baseline"]
    ax.plot([r["t"] for r in R], [r["thr_int"] for r in R], color="#bbb", lw=3, label="driver asks")
    for name in ("Baseline", "Proposed"):
        R = runs[name]
        ax.plot([r["t"] for r in R], [r["thr_exec"] for r in R], color=C[name], lw=1.1, label=name)
    ax.set_ylabel("throttle")
    ax.set_title("What the vehicle actually does (speed cap, failsafe stops)", loc="left")
    ax.legend(frameon=False, fontsize=8, ncol=3, loc="lower left")
    ax.set_xlabel("mission time (s)")
    ax.set_xlim(0, 240)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "timeline.png"))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--res", default="results")
    ap.add_argument("--out", default="results/charts")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    S = pd.read_csv(os.path.join(a.res, "summary.csv"))
    Cp = pd.read_csv(os.path.join(a.res, "capacity.csv"))
    headline(S, a.out, ["Baseline", "Baseline + duplicate video", "Proposed"], "headline.png",
             "Proposed vs real-world-style baselines")
    headline(S, a.out, ["Proposed", "Proposed - mission", "Proposed - predict", "Proposed - degrade"],
             "ablation.png", "What each feature contributes (one switched off at a time)")
    variants(S, a.out)
    capacity(Cp, a.out)
    cost(S, a.out)
    phases(S, a.out)
    if os.path.exists(os.path.join(a.res, "caps.csv")):
        caps(pd.read_csv(os.path.join(a.res, "caps.csv")), S, a.out)
    timeline(a.out)
    print("charts in", a.out)


if __name__ == "__main__":
    main()
