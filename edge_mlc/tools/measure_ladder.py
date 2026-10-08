"""Measure the real bitrate of each video quality level on YOUR camera.

  python3 tools/measure_ladder.py --device 0 --seconds 5          # USB camera
  python3 tools/measure_ladder.py --device videos/front.mp4       # recorded clip

Point the camera at a typical scene (the course). Copy the printed kbps
numbers into `ladder:` in config/run.yaml.
"""
import argparse
import os
import sys
import time

import cv2
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ap = argparse.ArgumentParser()
ap.add_argument("--config", default="config/run.yaml")
ap.add_argument("--device", default="0")
ap.add_argument("--seconds", type=float, default=5)
a = ap.parse_args()
ladder = yaml.safe_load(open(a.config))["ladder"]
cap = cv2.VideoCapture(int(a.device) if a.device.isdigit() else a.device)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
for i, lv in enumerate(ladder):
    if lv["fps"] == 0:
        continue
    total, n, t0 = 0, 0, time.time()
    while time.time() - t0 < a.seconds:
        ok, img = cap.read()
        if not ok:                                   # video file ended: loop
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        img = cv2.resize(img, (lv["w"], lv["h"]))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, lv["q"]])
        total += len(buf)
        n += 1
    kbps = (total / max(n, 1)) * lv["fps"] * 8 / 1000
    print(f"level {i} {lv['name']:8s} {lv['w']}x{lv['h']} @{lv['fps']}fps q{lv['q']}: "
          f"{kbps:7.0f} kbps (+~5% packet overhead)")
