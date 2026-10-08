"""Operator station (laptop).

  python3 operator_station.py                       # gamepad or keyboard driving
  python3 operator_station.py --record course.json  # also record commands
  python3 operator_station.py --replay course.json  # scripted run: replays a recording
  python3 operator_station.py --headless --replay course.json   # no window (tests)

Keys: arrows = drive · I inspect · H hide · P precision · N none (auto mode) · Esc quit
Shows: front + rear video, mode, per-link health, status banner.
"""
import argparse
import json
import socket
import threading
import time

import cv2
import numpy as np
import yaml

from common import protocol as P
from common.logger import Logger
from common.video import Reassembler

STATUS_COLOUR = {"OK": (60, 180, 75), "CAUTION": (255, 200, 0), "DEGRADED": (255, 130, 0),
                 "UNAVAILABLE": (220, 40, 40), "N/A": (120, 120, 120)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/run.yaml")
    ap.add_argument("--run", default=time.strftime("run_%H%M%S"))
    ap.add_argument("--record")
    ap.add_argument("--replay")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=0, help="stop after N s (0 = never)")
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    log = Logger(f"logs/{a.run}/operator.jsonl")
    hub = (cfg["hub"]["host"], cfg["hub"]["op_port"])
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))

    rs = Reassembler()
    shown = {"front": None, "rear": None}
    telem = [{}]
    ctl = {"thr": 0.0, "steer": 0.0, "mode": "", "quit": False}

    def rx():
        while True:
            data, _ = sock.recvfrom(65535)
            m = P.unpack(data)
            if not m:
                continue
            if m["stream"] in P.VIDEO:
                done = rs.add(m)
                if done:
                    seq, jpg, level, t_send, t_done = rs.latest[done]
                    name = P.VIDEO[done]
                    log.log("frame", cam=name, seq=seq, level=level,
                            latency_ms=round((t_done - t_send) * 1000, 1), bytes=len(jpg))
                    shown[name] = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            elif m["stream"] == P.TELEM:
                telem[0] = P.jload(m["payload"])
                log.log("telem", **telem[0])
    threading.Thread(target=rx, daemon=True).start()

    # ---- command source: replay file, or live input
    replay = json.load(open(a.replay)) if a.replay else None
    recorded = []

    def sender():
        seq, t_start = 0, time.time()
        period = 1 / cfg["timing"]["cmd_hz"]
        while not ctl["quit"]:
            seq += 1
            if replay is not None:
                k = min(int((time.time() - t_start) / period), len(replay) - 1)
                ctl.update(replay[k])
                if k == len(replay) - 1:
                    ctl["quit"] = True
            c = {"thr": round(ctl["thr"], 3), "steer": round(ctl["steer"], 3),
                 "mode": ctl["mode"], "t_op": time.time()}
            sock.sendto(P.pack(P.CMD, 0, seq, seq, time.time(), P.jdump(c)), hub)
            log.log("cmd_tx", seq=seq, **c)
            if a.record:
                recorded.append({k: c[k] for k in ("thr", "steer", "mode")})
            time.sleep(period)
    threading.Thread(target=sender, daemon=True).start()

    t_end = time.time() + a.duration if a.duration else None
    if a.headless:
        while not ctl["quit"] and (t_end is None or time.time() < t_end):
            time.sleep(0.1)
    else:
        run_window(ctl, shown, telem, replay is not None, t_end)
    ctl["quit"] = True
    if a.record:
        json.dump(recorded, open(a.record, "w"))
        print(f"recorded {len(recorded)} commands to {a.record}")


def run_window(ctl, shown, telem, replaying, t_end):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((1300, 600))
    pygame.display.set_caption("Operator station")
    font = pygame.font.SysFont("monospace", 18)
    big = pygame.font.SysFont("monospace", 30, bold=True)
    pad = pygame.joystick.Joystick(0) if pygame.joystick.get_count() else None
    if pad:
        pad.init()
    keymode = {pygame.K_i: "STOPPED", pygame.K_h: "HIDE", pygame.K_p: "PRECISION", pygame.K_n: ""}
    clock = pygame.time.Clock()
    while not ctl["quit"] and (t_end is None or time.time() < t_end):
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                ctl["quit"] = True
            if e.type == pygame.KEYDOWN and e.key in keymode:
                ctl["mode"] = keymode[e.key]
        if not replaying:
            if pad:      # left stick Y = throttle, right stick X = steering (check your pad)
                ctl["thr"], ctl["steer"] = -pad.get_axis(1), pad.get_axis(3)
            else:
                k = pygame.key.get_pressed()
                ctl["thr"] = 0.6 * (k[pygame.K_UP] - k[pygame.K_DOWN])
                ctl["steer"] = k[pygame.K_RIGHT] - k[pygame.K_LEFT]
        screen.fill((20, 20, 24))
        for i, cam in enumerate(("front", "rear")):
            img = shown[cam]
            if img is not None:
                img = cv2.cvtColor(cv2.resize(img, (640, 480)), cv2.COLOR_BGR2RGB)
                screen.blit(pygame.surfarray.make_surface(img.swapaxes(0, 1)), (10 + i * 650, 10))
            screen.blit(font.render(cam, True, (220, 220, 220)), (10 + i * 650, 495))
        t = telem[0]
        st = t.get("status", "N/A")
        pygame.draw.rect(screen, STATUS_COLOUR.get(st, (120, 120, 120)), (10, 525, 300, 60))
        screen.blit(big.render(st, True, (0, 0, 0)), (20, 538))
        info = f"mode {t.get('mode', '?')}  cap {t.get('cap', 1):.1f}  cmd via {t.get('cmd_links', [])}"
        screen.blit(font.render(info, True, (230, 230, 230)), (330, 525))
        x = 330
        for ln, v in t.get("links", {}).items():
            txt = f"{ln}: {'UP' if v['alive'] else 'DOWN'} {v['rtt']}ms loss {v['loss']} cap {v['cap']}k"
            screen.blit(font.render(txt, True, (200, 200, 200)), (x, 555))
            x += 470
        pygame.display.flip()
        clock.tick(30)
    pygame.quit()


if __name__ == "__main__":
    main()
