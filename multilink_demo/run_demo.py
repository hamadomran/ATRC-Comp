"""Run a video through the multi-link codec under three link conditions.

  python3 run_demo.py samples/clip.mp4                # writes output/
  python3 run_demo.py samples/clip.mp4 --seconds 20 --parity 0.5

The sender always sends on two links. What changes is how much of link 2
arrives:

  one_link            link 2 down (0% of its packets arrive)
  one_and_half_links  link 2 drops half of its packets at random
  two_links           both links deliver everything

Link 1 always delivers. Outputs one video per case, a side-by-side
comparison and stats.json.
"""
import argparse
import json
import os
import shutil
import subprocess

import cv2
import numpy as np

from multilink import Link, Receiver, Sender

CASES = [("one_link", "1 link", 0.0), ("one_and_half_links", "1.5 links", 0.5), ("two_links", "2 links", 1.0)]


def psnr(a, b):
    mse = np.mean((a.astype(np.float32) - b.astype(np.float32)) ** 2)
    return 99.0 if mse == 0 else 10 * np.log10(255 ** 2 / mse)


def label(img, text, sub):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 46), (0, 0, 0), -1)
    cv2.putText(out, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, sub, (10, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return out


def to_h264(path):
    """Re-encode to H.264 so browsers and phones play it (needs ffmpeg; skipped if absent)."""
    if not shutil.which("ffmpeg"):
        return
    tmp = path + ".tmp.mp4"
    r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", path, "-c:v", "libx264",
                        "-pix_fmt", "yuv420p", "-crf", "18", tmp])
    if r.returncode == 0:
        os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--out", default="output")
    ap.add_argument("--seconds", type=float, default=0, help="only the first N seconds (0 = all)")
    ap.add_argument("--parity", type=float, default=0.5,
                    help="parity symbols as a share of the stripes (0.5 = 50%%)")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {a.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
    frames = []
    while True:
        ok, img = cap.read()
        if not ok or (a.seconds and len(frames) >= a.seconds * fps):
            break
        frames.append(img)
    if not frames:
        raise SystemExit("no frames read")

    # full stream 640 wide at the clip's aspect ratio (stripes are 16 px), base half that
    h0, w0 = frames[0].shape[:2]
    W, H = 640, max(16, round(640 * h0 / w0 / 16) * 16)
    four = cv2.VideoWriter_fourcc(*"mp4v")
    side = cv2.VideoWriter(os.path.join(a.out, "comparison.mp4"), four, fps, (3 * W, H))
    writers, stats, recs, links, senders = {}, {}, {}, {}, {}
    for key, name, d in CASES:
        senders[key] = Sender(2, full_size=(W, H), base_size=(W // 2, H // 2), parity=a.parity)
        recs[key] = Receiver(full_size=(W, H))
        links[key] = [Link("link 1", 1.0, a.seed), Link("link 2", d, a.seed + 1)]
        writers[key] = cv2.VideoWriter(os.path.join(a.out, key + ".mp4"), four, fps, (W, H))
        stats[key] = {"psnr": [], "full": 0, "partial": 0, "base_only": 0, "held": 0, "stripes": []}

    for n, img in enumerate(frames):
        ref = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
        panels = []
        for key, name, _ in CASES:
            s = senders[key]
            per_link = s.encode(n, img)
            got = [pk for ln, pks in zip(links[key], per_link) for pk in ln.carry(pks)]
            out, info = recs[key].decode(got)
            st = stats[key]
            st["psnr"].append(psnr(out, ref))
            st["stripes"].append(info["stripes"] / s.n_stripes)
            if info["held"]:
                st["held"] += 1
            elif info["complete"]:
                st["full"] += 1
            elif info["stripes"]:
                st["partial"] += 1
            else:
                st["base_only"] += 1
            how = ("held" if info["held"] else "full picture" + (" (rebuilt by FEC)" if info["rebuilt"] else "")
                   if info["complete"] else f"base + {info['stripes']}/{s.n_stripes} full-res stripes")
            panel = label(out, name, how)
            writers[key].write(panel)
            panels.append(panel)
        side.write(np.hstack(panels))

    for w in list(writers.values()) + [side]:
        w.release()
    for f in [k for k, _, _ in CASES] + ["comparison"]:
        to_h264(os.path.join(a.out, f + ".mp4"))

    s = senders[CASES[0][0]]
    summary = {"frames": len(frames), "fps": fps, "stripes_per_frame": s.n_stripes, "parity": a.parity,
               "layout": {"link 1": "base + parity packets", "link 2": "base + full-resolution stripe packets"}}
    print(f"{len(frames)} frames at {fps:.0f} fps, {s.n_stripes} stripes per frame, parity {a.parity:.0%} of the data packets\n")
    print(f"{'case':<20}{'full':>7}{'partial':>9}{'base only':>11}{'held':>6}{'stripes':>9}{'PSNR dB':>9}"
          f"{'link1 kbps':>12}{'link2 kbps':>12}")
    for key, name, _ in CASES:
        st = stats[key]
        N = len(frames)
        kbps = [ln.bytes_sent * 8 / 1000 / (N / fps) for ln in links[key]]
        row = {"full_pct": 100 * st["full"] / N, "partial_pct": 100 * st["partial"] / N,
               "base_only_pct": 100 * st["base_only"] / N, "held_pct": 100 * st["held"] / N,
               "stripes_pct": 100 * float(np.mean(st["stripes"])), "psnr_db": float(np.mean(st["psnr"])),
               "link_kbps_sent": kbps,
               "link_delivery": [ln.delivered / max(ln.sent, 1) for ln in links[key]]}
        summary[key] = row
        print(f"{name:<20}{row['full_pct']:>6.0f}%{row['partial_pct']:>8.0f}%{row['base_only_pct']:>10.0f}%"
              f"{row['held_pct']:>5.0f}%{row['stripes_pct']:>8.0f}%{row['psnr_db']:>9.1f}"
              f"{kbps[0]:>12.0f}{kbps[1]:>12.0f}")
    json.dump(summary, open(os.path.join(a.out, "stats.json"), "w"), indent=2)
    print(f"\nvideos and stats.json in {a.out}/")


if __name__ == "__main__":
    main()
