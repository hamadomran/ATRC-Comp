"""Cameras (car side) and frame reassembly (operator side).

Each camera runs in its own thread. The decision module sets
  target = (list of link names, ladder level)
level 0 = paused, 1 = trickle. Every frame is JPEG-encoded at the level's
size/quality, split into ~1100-byte fragments and sent on each listed link
(two links = duplicated).
"""
import threading
import time

import cv2
import numpy as np

from . import protocol as P


class Camera:
    def __init__(self, name, device, ladder, send_fn):
        self.name, self.stream = name, P.STREAM_ID[name]
        self.ladder = ladder
        self.send_fn = send_fn            # send_fn(link_name, stream, seq, payload, flags)
        self.links, self.level = [], 0
        self.seq = 0
        self.bytes_window = []            # (t, bytes) for actual bitrate
        self.cap = None
        self.is_file = isinstance(device, str) and device != "synthetic"
        if device == "synthetic":
            self.cap = None
        elif self.is_file:                # recorded clip, played in real time and looped
            self.cap = cv2.VideoCapture(device)
            if not self.cap.isOpened():
                raise FileNotFoundError(f"cannot open video file {device}")
            self.src_fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
            self.n_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
            self.pos, self.t_start = 0, time.monotonic()
        else:                             # USB camera number
            self.cap = cv2.VideoCapture(int(device))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.lock = threading.Lock()

    def set_target(self, links, level):
        with self.lock:
            self.links, self.level = list(links), int(level)

    def _grab(self):
        if self.cap is None:              # synthetic test pattern with moving bar
            img = np.full((480, 640, 3), 40, np.uint8)
            x = int(time.time() * 200) % 640
            cv2.rectangle(img, (x, 0), (x + 40, 480), (0, 200, 255), -1)
            cv2.putText(img, self.name, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
            return img
        if self.is_file:
            return self._grab_file()
        ok, img = self.cap.read()
        return img if ok else None

    def _grab_file(self):
        """Return the clip frame for 'now', so the clip plays at real speed
        whatever frame rate we send at, and loops at the end."""
        target = int((time.monotonic() - self.t_start) * self.src_fps)
        if target >= self.n_frames:                    # end of clip: loop
            self.t_start = time.monotonic()
            target = 0
        if target < self.pos or target - self.pos > 60:   # far away: jump directly
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, target)
            self.pos = target
        while self.pos < target:                       # nearby: skip frames cheaply
            self.cap.grab()
            self.pos += 1
        ok, img = self.cap.read()
        self.pos += 1
        if not ok:                                     # unexpected end: rewind
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.pos, self.t_start = 0, time.monotonic()
            return None
        return img

    def encode(self, img, lv):
        img = cv2.resize(img, (lv["w"], lv["h"]))
        cv2.putText(img, f"{self.name} {lv['name']}", (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, lv["q"]])
        return buf.tobytes() if ok else b""

    def actual_kbps(self):
        now = time.monotonic()
        self.bytes_window = [(t, b) for t, b in self.bytes_window if now - t < 2.0]
        return sum(b for _, b in self.bytes_window) * 8 / 2.0 / 1000

    def run(self):
        while True:
            with self.lock:
                links, level = self.links, self.level
            lv = self.ladder[level]
            if level == 0 or not links:
                time.sleep(0.05)
                continue
            t0 = time.monotonic()
            img = self._grab()
            if img is not None:
                jpg = self.encode(img, lv)
                self.seq += 1
                chunks = [jpg[i:i + P.MAX_CHUNK] for i in range(0, len(jpg), P.MAX_CHUNK)]
                for i, ch in enumerate(chunks):
                    payload = P.VFRAG.pack(i, len(chunks), level) + ch
                    for k, ln in enumerate(links):
                        self.send_fn(ln, self.stream, self.seq, payload, P.FLAG_DUP if k else 0)
                self.bytes_window.append((time.monotonic(), len(jpg) * len(links)))
            # wait for the next frame, but wake up at once if the target changes
            deadline = t0 + 1.0 / lv["fps"]
            while time.monotonic() < deadline:
                with self.lock:
                    if self.level != level or self.links != links:
                        break
                time.sleep(0.01)

    def start(self):
        threading.Thread(target=self.run, daemon=True).start()


class Reassembler:
    """Operator side: rebuild frames from fragments; keep only the newest."""

    def __init__(self):
        self.partial = {}      # (stream, seq) -> {idx: bytes}, count, level, t_first
        self.latest = {}       # stream -> (seq, jpg_bytes, level, t_send, t_done)

    def add(self, m):
        idx, count, level = P.VFRAG.unpack_from(m["payload"])
        key = (m["stream"], m["seq"])
        last = self.latest.get(m["stream"])
        if last and m["seq"] <= last[0]:
            return None                      # older than what we already showed
        p = self.partial.setdefault(key, {"frags": {}, "count": count, "level": level,
                                          "t0": time.time(), "t_send": m["t_send"]})
        p["frags"][idx] = m["payload"][P.VFRAG.size:]
        if len(p["frags"]) == count:
            jpg = b"".join(p["frags"][i] for i in range(count))
            del self.partial[key]
            self.latest[m["stream"]] = (m["seq"], jpg, level, p["t_send"], time.time())
            for k in [k for k in self.partial if k[0] == m["stream"] and k[1] < m["seq"]]:
                del self.partial[k]          # abandon older incomplete frames
            return m["stream"]
        return None
