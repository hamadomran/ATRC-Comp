"""Charts for the report / deck.

  python3 sim/plots.py --res results --out results/charts
"""
import argparse
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
from sim.run_experiments import make          # noqa: E402
from sim.scenario import SEGMENTS             # noqa: E402

C = {"Baseline": "#8a8f98", "Baseline + duplicate video": "#c3b28f", "Proposed": "#1764c0",
     "Proposed - mission": "#9cc0ea", "Proposed - uploads": "#8fd0c9", "Proposed - predict": "#6fa3de",
     "Baseline (no shaping)": "#4c5057", "Proposed (urgent_first)": "#0e3f7e"}
# the three upload policies of Experiment B (same shaping, different order)
POLICIES = [("Proposed - uploads", "video-first"), ("Proposed (urgent_first)", "urgent-first"),
            ("Proposed", "adaptive (ours)")]
LINKC = {"radio": "#5b8c5a", "starlink": "#7a5ea8", "lte": "#d08a3c"}
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.titleweight": "bold", "axes.titlesize": 11, "figure.dpi": 150})



def simulate(seed=0):
    """One nominal run per module; the Sim objects keep rows + frame delays."""
    sims = {}
    for name in ("Baseline", "Proposed"):
        cfg, dec = make(name)
        sim = engine.Sim(cfg, dec, "nominal", seed)
        sim.run()
        sims[name] = sim
    return sims


def bars(ax, df, configs, col, title, ylabel, fmt="{:.0f}", better=None):
    d = df.set_index("config").loc[configs]
    x = np.arange(len(configs))
    v = np.nan_to_num(d[col].to_numpy(float))
    e = np.nan_to_num(d[col + "_ci"].to_numpy(float))
    ax.bar(x, v, yerr=e, color=[C[c] for c in configs], capsize=3, width=0.65,
           error_kw={"elinewidth": 1, "ecolor": "#333"})
    for i, val in enumerate(v):
        ax.text(i, val + e[i] + 0.02 * max(v.max(), 1e-9), fmt.format(val),
                ha="center", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels([c.replace("Baseline + duplicate video", "Baseline\n+ dup. video")
                        .replace("Proposed - ", "without\n") for c in configs], fontsize=8.5)
    ax.set_title(title + (f"  ({better})" if better else ""), loc="left")
    ax.set_ylabel(ylabel)
    ax.set_ylim(0, (v + e).max() * 1.18 + 1e-9)


def headline(S, out, configs, name, sup):
    nom = S[S.variant == "nominal"]
    panels = [("useful_view_pct", "Needed view available", "% of mission", "{:.1f}", "higher is better"),
              ("interruptions", "View interruptions", "count per mission", "{:.1f}", "lower"),
              ("video_delay_p95_bulk_ms", "Video delay while uploading (p95)", "ms", "{:.0f}", "lower"),
              ("req_mean_s", "Requested upload delivered", "mean s after request", "{:.0f}", "lower"),
              ("mission_time_s", "Mission time", "seconds", "{:.0f}", "lower"),
              ("dup_overhead_pct", "Extra data sent (duplication)", "% of all data", "{:.1f}", "lower")]
    fig, axs = plt.subplots(2, 3, figsize=(12, 6.6))
    for ax, (col, t, yl, f, b) in zip(axs.flat, panels):
        bars(ax, nom, configs, col, t, yl, f, b)
    fig.suptitle(sup, x=0.01, ha="left", fontsize=12, fontweight="bold")
    n = int(nom["n"].max())
    fig.text(0.01, 0.005, f"Nominal mission, {n} runs each (different random seeds). "
             "Error bars: 95% confidence interval.", fontsize=8, color="#555")
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
    ax.axvline(1.0, color="#999", lw=1, ls=":")
    ax.text(1.0, 3, " scenario default (1.0)", fontsize=7.5, color="#666")
    ax.set_xlabel("Satellite capacity in the crater (Mbps)")
    ax.set_ylabel("Needed view available in crater (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Thin links: giving capacity to the camera that matters", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "capacity.png"))
    plt.close(fig)


def segments(S, out):
    nom = S[S.variant == "nominal"].set_index("config")
    fig, ax = plt.subplots(figsize=(10.5, 3.6))
    w = 0.38
    for i, c in enumerate(["Baseline", "Proposed"]):
        v = [nom.loc[c, f"useful_{n}"] for n in SEGMENTS]
        e = [nom.loc[c, f"useful_{n}_ci"] for n in SEGMENTS]
        ax.bar(np.arange(len(SEGMENTS)) + (i - 0.5) * w, v, w, yerr=e, color=C[c], label=c, capsize=2)
    ax.set_xticks(np.arange(len(SEGMENTS)))
    ax.set_xticklabels([n.replace(" ", "\n") for n in SEGMENTS])
    ax.set_ylim(40, 103)
    ax.set_ylabel("Needed view available (%)")
    ax.set_title("Where the difference comes from (by mission segment)", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "segments.png"))
    plt.close(fig)


def expA_transitions(S, out):
    """Experiment A: behaviour around ground-truth link changes."""
    nom = S[S.variant == "nominal"].set_index("config")
    cfgs = ["Baseline", "Baseline + duplicate video", "Proposed", "Proposed - mission"]
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    v = [nom.loc[c, "useful_transition_pct"] for c in cfgs]
    e = [nom.loc[c, "useful_transition_pct_ci"] for c in cfgs]
    ax.bar(np.arange(len(cfgs)), v, 0.6, yerr=e, color=[C[c] for c in cfgs], capsize=3)
    for i, (val, err) in enumerate(zip(v, e)):
        ax.text(i, val + err + 0.5, f"{val:.1f}", ha="center", fontsize=9)
    ax.set_xticks(np.arange(len(cfgs)))
    ax.set_xticklabels([c.replace("Baseline + duplicate video", "Baseline\n+ dup. video")
                        .replace("Proposed - mission", "Proposed\nwithout mission") for c in cfgs],
                       fontsize=8.5)
    ax.set_ylabel("needed view available (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Needed view inside transition windows [-2 s, +10 s]", loc="left")
    n_ev = nom.loc["Proposed", "n_transition_events"]
    n_ud = nom.loc["Proposed", "n_trans_updown"]
    n_cap = nom.loc["Proposed", "n_trans_capacity"]
    ax.text(0.99, 0.04, f"events per mission (mean): {n_ev:.0f}\n"
            f"  link up/down: {n_ud:.0f}\n  capacity ≥ 30%/s: {n_cap:.0f}",
            transform=ax.transAxes, ha="right", fontsize=8, color="#444",
            bbox=dict(fc="#f6f6f6", ec="#ddd"))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "expA_transitions.png"))
    plt.close(fig)


def expB_uploads(S, out):
    """Experiment B: the three upload policies. Each static policy should lose
    in one situation; plot whatever comes out."""
    nom = S[S.variant == "nominal"].set_index("config")
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
    ax = axs[0]
    w = 0.26
    groups = [("scan_drive_s", "20 MB scan\nwhile DRIVING"), ("lidar_crater_s", "20 MB scan\nwhile STOPPED")]
    for i, (cfg, lab) in enumerate(POLICIES):
        v = [nom.loc[cfg, k] for k, _ in groups]
        e = [nom.loc[cfg, k + "_ci"] for k, _ in groups]
        ax.bar(np.arange(len(groups)) + (i - 1) * w, v, w, yerr=e, color=C[cfg], label=lab, capsize=2)
    ax.set_xticks(np.arange(len(groups)))
    ax.set_xticklabels([lab for _, lab in groups], fontsize=8.5)
    ax.set_ylabel("seconds from request to delivered")
    ax.set_title("Urgent scan delivery time, driving vs stopped", loc="left")
    ax.legend(frameon=False, fontsize=8)
    ax = axs[1]
    v = [nom.loc[cfg, "needed_fps_while_req_drive"] for cfg, _ in POLICIES]
    e = [nom.loc[cfg, "needed_fps_while_req_drive_ci"] for cfg, _ in POLICIES]
    ax.bar(np.arange(len(POLICIES)), v, 0.55, yerr=e, color=[C[c] for c, _ in POLICIES], capsize=3)
    ax.axhline(10, color="#b33", lw=1, ls="--")
    ax.text(0.02, 10.25, "DRIVE needs 10 fps", fontsize=8, color="#b33")
    ax.set_xticks(np.arange(len(POLICIES)))
    ax.set_xticklabels([lab for _, lab in POLICIES], fontsize=8.5)
    ax.set_ylabel("delivered fps (front camera)")
    ax.set_title("Video frame rate while the driving scan uploads", loc="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "expB_uploads.png"))
    plt.close(fig)


def appendix_video_delay(out, sims):
    fig, ax = plt.subplots(figsize=(6, 3.6))
    for name, sim in sims.items():
        lat = np.sort([f[0] for f in sim.frames if f[2]])
        if len(lat):
            ax.plot(lat, np.arange(1, len(lat) + 1) / len(lat), color=C[name], lw=1.8, label=name)
    ax.set_xscale("log")
    ax.set_xlabel("video frame delay while bulk data was moving (ms)")
    ax.set_ylabel("fraction of frames")
    ax.set_ylim(0, 1.02)
    ax.set_title("Appendix: frame delay CDF while uploading", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "appendix_video_delay.png"))
    plt.close(fig)


def _seg_spans(rows):
    spans, cur, t0 = [], None, 0.0
    for r in rows:
        if r["seg"] != cur:
            if cur:
                spans.append((cur, t0, r["t"]))
            cur, t0 = r["seg"], r["t"]
    if cur:
        spans.append((cur, t0, rows[-1]["t"]))
    return spans


def timeline(out, sims):
    fig, axs = plt.subplots(5, 1, figsize=(12, 11), sharex=True,
                            gridspec_kw={"height_ratios": [1.1, 0.9, 1, 1, 0.9]})
    rp = sims["Proposed"].rows
    tp = np.array([r["t"] for r in rp])
    t_end = max(s.rows[-1]["t"] for s in sims.values())

    # 1) link ground truth along the Proposed run's trajectory
    ax = axs[0]
    for i, ln in enumerate(["radio", "starlink", "lte"]):
        st = [sims["Proposed"].truth_at(ln, r) for r in rp]
        up = np.array([x["up"] for x in st])
        good = up & np.array([x["loss"] < 0.05 and 2 * x["owd_ms"] < 250 for x in st])
        ax.fill_between(tp, i - 0.35, i + 0.35, where=good, color=LINKC[ln], step="mid", lw=0)
        ax.fill_between(tp, i - 0.35, i + 0.35, where=up & ~good, color=LINKC[ln], alpha=0.3, step="mid", lw=0)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["Radio", "Starlink", "4G"])
    ax.set_title("Links (ground truth along the Proposed run: solid = good, faint = up but lossy or slow)",
                 loc="left")
    for name, a, b in _seg_spans(rp):
        for x in axs:
            x.axvline(a, color="#ddd", lw=0.8, zorder=0)
        ax.text((a + b) / 2, 2.75, name, ha="center", fontsize=7.5, color="#444")
    ax.set_ylim(-0.6, 3.1)

    # 2) vehicle position
    ax = axs[1]
    for name, sim in sims.items():
        ax.plot([r["t"] for r in sim.rows], [r["d"] for r in sim.rows], color=C[name], lw=1.4, label=name)
    ax.set_ylabel("position (km)")
    ax.set_title("Vehicle position along the route", loc="left")
    ax.legend(frameon=False, fontsize=8, loc="upper left")

    # 3+4) needed-camera level with "view missing" bands
    for ax, name in zip(axs[2:4], ["Baseline", "Proposed"]):
        R = sims[name].rows
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
    axs[2].legend(frameon=False, fontsize=8, loc="lower left")

    # 5) bulk upload rate, stacked requested / background
    ax = axs[4]
    for name, sim in sims.items():
        R = sim.rows
        tt = np.array([r["t"] for r in R])
        rq = np.array([r["bulk_req_kbps"] for r in R])
        bg = np.array([r["bulk_bg_kbps"] for r in R])
        ax.fill_between(tt, 0, rq, color=C[name], alpha=0.55, step="mid", lw=0, label=f"{name}: requested")
        ax.fill_between(tt, rq, rq + bg, color=C[name], alpha=0.22, step="mid", lw=0,
                        label=f"{name}: background")
    ax.set_ylabel("bulk kbps")
    ax.set_title("Upload rate (stacked: requested, then background)", loc="left")
    ax.legend(frameon=False, fontsize=8, ncol=2, loc="upper left")
    ax.set_xlabel("mission time (s)")
    ax.set_xlim(0, t_end)
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
    sims = simulate()
    headline(S, a.out, ["Baseline", "Baseline + duplicate video", "Proposed"], "headline.png",
             "Proposed vs real-world-style baselines")
    headline(S, a.out, ["Proposed", "Proposed - mission", "Proposed - uploads", "Proposed - predict"],
             "ablation.png", "What each feature contributes (one switched off at a time)")
    variants(S, a.out)
    segments(S, a.out)
    capacity(Cp, a.out)
    expA_transitions(S, a.out)
    expB_uploads(S, a.out)
    appendix_video_delay(a.out, sims)
    timeline(a.out, sims)
    print("charts in", a.out)


if __name__ == "__main__":
    main()
