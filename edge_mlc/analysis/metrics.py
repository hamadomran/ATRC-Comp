"""Compute per-run metrics from the logs.

  python3 analysis/metrics.py logs/            # every run folder -> results.csv

Each run folder needs car.jsonl and operator.jsonl (copy the operator log from the
laptop into the same folder). Clocks must be synced (chrony) for latency numbers.
"""
import json
import os
import sys

import numpy as np
import pandas as pd

FRESH_S = 0.5          # a view is "live" if its newest frame is < 0.5 s old (A)
MIN_USEFUL_LEVEL = 2   # lowest ladder level you can drive with (A, from the ladder test)


def load(path):
    with open(path) as f:
        return [json.loads(l) for l in f if l.strip()]


def run_metrics(folder):
    car = load(os.path.join(folder, "car.jsonl"))
    op = load(os.path.join(folder, "operator.jsonl"))
    start = next(e for e in car if e["ev"] == "start")
    label = start["decider"] + (f"[{start['features']}]" if start["features"] else "")
    cmd_tx = [e for e in op if e["ev"] == "cmd_tx"]
    if not cmd_tx:
        return None
    t0, t1 = cmd_tx[0]["t"], cmd_tx[-1]["t"]

    # --- control continuity (commands executed on the car)
    rx = [e for e in car if e["ev"] == "cmd_rx" and t0 <= e["t"] <= t1 + 1]
    used = [e["t"] for e in rx if not e["stale"]]
    gaps = np.diff([t0] + used + [t1]) * 1000
    ages = np.array([e["age_ms"] for e in rx]) if rx else np.array([np.nan])

    # --- failsafe stops (ESP32)
    esp = [e for e in car if e["ev"] == "esp32" and t0 + 1 <= e["t"] <= t1]
    fs_time, stops, prev = 0.0, 0, 0
    for a, b in zip(esp, esp[1:]):
        if a["failsafe"]:
            fs_time += b["t"] - a["t"]
        if a["failsafe"] and not prev:
            stops += 1
        prev = a["failsafe"]

    # --- switching behaviour (from the decision log)
    ticks = [e for e in car if e["ev"] == "tick" and t0 <= e["t"] <= t1]
    switches, pingpong, last_switch = 0, 0, {}
    for a, b in zip(ticks, ticks[1:]):
        for cam in ("front", "rear"):
            la, lb = a["plan"]["video"][cam]["links"], b["plan"]["video"][cam]["links"]
            if la and lb and la[0] != lb[0]:
                switches += 1
                prev_sw = last_switch.get(cam)
                if prev_sw and prev_sw[1] == lb[0] and b["t"] - prev_sw[0] < 5.0:
                    pingpong += 1                    # went back within 5 s
                last_switch[cam] = (b["t"], la[0])
    sent = sum(sum(e["cam_kbps"].values()) for e in ticks)
    dup = sum(e["cam_kbps"][c] * (1 - 1 / len(e["plan"]["video"][c]["links"]))
              for e in ticks for c in ("front", "rear") if len(e["plan"]["video"][c]["links"]) > 1)

    # --- view quality at the operator (0.1 s grid)
    frames = {"front": [], "rear": []}
    for e in op:
        if e["ev"] == "frame":
            frames[e["cam"]].append((e["t"], e["level"]))
    thr_t = np.array([e["t"] for e in cmd_tx])
    thr_v = np.array([e["thr"] for e in cmd_tx])
    useful = wrong = freezes = 0
    frozen = False
    grid = np.arange(t0, t1, 0.1)
    idx = {c: 0 for c in frames}
    newest = {c: None for c in frames}
    for t in grid:
        for c in frames:
            while idx[c] < len(frames[c]) and frames[c][idx[c]][0] <= t:
                newest[c] = frames[c][idx[c]]
                idx[c] += 1
        thr = thr_v[max(0, np.searchsorted(thr_t, t) - 1)]
        need = "rear" if thr < -0.1 else "front"      # ground truth: which view the driver needs
        other = "front" if need == "rear" else "rear"
        ok = lambda c: newest[c] is not None and t - newest[c][0] < FRESH_S and newest[c][1] >= MIN_USEFUL_LEVEL
        if ok(need):
            useful += 1
            frozen = False
        else:
            if ok(other):
                wrong += 1
            if not frozen:
                freezes += 1
                frozen = True
    n = max(len(grid), 1)
    return {
        "run": os.path.basename(folder.rstrip("/")), "config": label,
        "duration_s": round(t1 - t0, 1),
        "max_cmd_gap_ms": round(float(gaps.max()), 0),
        "cmd_within_100ms_pct": round(100 * float(np.mean(ages <= 100)), 1),
        "cmd_within_200ms_pct": round(100 * float(np.mean(ages <= 200)), 1),
        "stale_rejected": sum(e["stale"] for e in rx),
        "stale_executed": sum(1 for e in rx if not e["stale"] and e["age_ms"] > 200),
        "failsafe_stops": stops, "failsafe_time_s": round(fs_time, 2),
        "video_switches": switches, "pingpongs": pingpong,
        "useful_view_pct": round(100 * useful / n, 1),
        "wrong_view_pct": round(100 * wrong / n, 1),
        "freezes": freezes,
        "dup_overhead_pct": round(100 * dup / sent, 1) if sent else 0.0,
    }


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else "logs"
    rows = []
    for d in sorted(os.listdir(root)):
        f = os.path.join(root, d)
        if os.path.exists(os.path.join(f, "car.jsonl")) and os.path.exists(os.path.join(f, "operator.jsonl")):
            r = run_metrics(f)
            if r:
                rows.append(r)
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(root, "results.csv"), index=False)
    pd.set_option("display.width", 200)
    print(df.to_string(index=False))
    if len(df):
        print("\nMedian per configuration:")
        print(df.drop(columns=["run"]).groupby("config").median(numeric_only=True).to_string())


if __name__ == "__main__":
    main()
