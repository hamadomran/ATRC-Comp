"""Link health, shared by baseline and proposed.

Per link it keeps:
  rtt_ms     median heartbeat round-trip over the last 1 s
  rtt_p90    90th percentile over the last 1 s
  rtt_min10  lowest answered round-trip over the last 10 s (queue-delay reference)
  loss       share of heartbeats (sent 0.4-1.4 s ago) never echoed
  miss_streak consecutive heartbeats unanswered after hb_timeout
  alive      False once 3 heartbeats in a row are missed
  cap_kbps   capacity estimate from hub delivery reports
  data_loss  loss of data packets on this link (hub-measured)
  wifi_signal / wifi_txrate / wifi_retries / lte_quality (radio stats)
"""
import re
import subprocess
import threading
import time
from collections import deque

import numpy as np


class LinkState:
    def __init__(self, name, nominal_kbps):
        self.name = name
        self.nominal = nominal_kbps
        self.sent = {}                      # hb seq -> send time (monotonic)
        self.echoes = deque()               # (t_recv, rtt_ms)
        self.rtt10 = deque()                # (t_recv, rtt_ms) answered heartbeats, last 10 s
        self.miss_streak = 0
        self.cap_kbps = nominal_kbps * 0.5  # (A) start at half nominal, probe up
        self.delivered_kbps = 0.0
        self.data_loss = 0.0
        self.wifi_signal = None
        self.wifi_txrate = None
        self.wifi_retries = None
        self.lte_quality = None
        self.last_report = 0.0


class Monitor:
    def __init__(self, links_cfg, hb_timeout_ms=300, clock=time.monotonic):
        self.clock = clock                  # the simulator passes its own clock
        self.s = {l["name"]: LinkState(l["name"], l["nominal_kbps"]) for l in links_cfg}
        self.timeout = hb_timeout_ms / 1000
        self.lock = threading.Lock()

    # ---- heartbeats -------------------------------------------------
    def hb_sent(self, link, seq):
        with self.lock:
            self.s[link].sent[seq] = self.clock()

    def hb_echo(self, link, seq):
        now = self.clock()
        with self.lock:
            st = self.s[link]
            t0 = st.sent.pop(seq, None)
            if t0 is not None:
                st.echoes.append((now, (now - t0) * 1000))
                st.rtt10.append((now, (now - t0) * 1000))
                while now - st.rtt10[0][0] > 10.0:
                    st.rtt10.popleft()
                st.miss_streak = 0

    def _expire(self, st, now):
        # heartbeats unanswered past the timeout count as misses
        for seq, t0 in list(st.sent.items()):
            if now - t0 > self.timeout:
                st.miss_streak += 1
                st.sent.pop(seq)
                st.echoes.append((now, None))       # record the miss
        while st.echoes and now - st.echoes[0][0] > 3.0:     # keep 3 s (30 heartbeats)
            st.echoes.popleft()

    # ---- hub delivery reports --------------------------------------
    def report(self, link, delivered_kbps, data_loss):
        with self.lock:
            st = self.s[link]
            st.delivered_kbps, st.data_loss = delivered_kbps, data_loss
            st.last_report = self.clock()
            # Capacity estimator (A), AIMD style with a queue check:
            #  * data lost while some got through -> over capacity:
            #    cut to what got through (at most halve per report)
            #  * delay well above the 10 s minimum while we use most of the
            #    estimate -> a queue is building: we are at capacity, hold below it
            #  * nothing got through -> an outage, not congestion: keep estimate
            #  * otherwise probe up: +5% per report when using most of it,
            #    else drift up slowly (0.25% of nominal per report)
            now = self.clock()
            recent = [r for (t, r) in st.echoes if r is not None and now - t <= 1.0]
            qdelay = (float(np.median(recent)) - min(r for _, r in st.rtt10)) if recent and st.rtt10 else 0.0
            busy = delivered_kbps >= 0.7 * st.cap_kbps
            if data_loss > 0.05 and delivered_kbps > 0:
                st.cap_kbps = max(50.0, 0.5 * st.cap_kbps, 0.85 * delivered_kbps)
            elif qdelay > 80 and delivered_kbps >= 0.5 * st.cap_kbps:
                st.cap_kbps = max(50.0, 0.9 * delivered_kbps)
            elif data_loss <= 0.05 and qdelay < 30:
                step = 0.05 * st.cap_kbps if busy else 0.0
                st.cap_kbps = min(st.nominal, st.cap_kbps + max(step, 0.0025 * st.nominal))

    def poll_wifi(self, link, iface):
        try:
            out = subprocess.run(["iw", "dev", iface, "station", "dump"],
                                 capture_output=True, text=True, timeout=1).stdout
        except Exception:
            return
        sig = re.search(r"signal:\s*(-?\d+)", out)
        rate = re.search(r"tx bitrate:\s*([\d.]+)", out)
        retr = re.search(r"tx retries:\s*(\d+)", out)
        with self.lock:
            st = self.s[link]
            st.wifi_signal = int(sig.group(1)) if sig else None
            st.wifi_txrate = float(rate.group(1)) if rate else None
            st.wifi_retries = int(retr.group(1)) if retr else None

    def poll_lte(self, link):
        # ModemManager modem; HiLink (router-style) dongles do not expose this.
        try:
            out = subprocess.run(["mmcli", "-m", "any", "--output-keyvalue"],
                                 capture_output=True, text=True, timeout=2).stdout
            m = re.search(r"modem.generic.signal-quality.value\s*:\s*(\d+)", out)
            with self.lock:
                self.s[link].lte_quality = int(m.group(1)) if m else None
        except Exception:
            pass

    def start_pollers(self, links_cfg):
        def loop():
            n = 0
            while True:
                for l in links_cfg:
                    if l["name"] == "wifi":
                        self.poll_wifi(l["name"], l["iface"])
                    elif l["name"] == "lte" and n % 5 == 0:
                        self.poll_lte(l["name"])
                n += 1
                time.sleep(0.2)
        threading.Thread(target=loop, daemon=True).start()

    # ---- snapshot used by the decision modules ----------------------
    def snapshot(self):
        now = self.clock()
        out = {}
        with self.lock:
            for name, st in self.s.items():
                self._expire(st, now)
                rtts = [r for (t, r) in st.echoes if r is not None and now - t <= 1.0]  # delay: last 1 s
                # loss over the last 3 s: misses / (answered + missed) heartbeats,
                # leaving out outages (3+ misses in a row): those are what
                # `alive` is for, so a link is not blamed for an outage it has
                # already recovered from (A)
                flags, run = [], 0
                for (_, r) in list(st.echoes) + [(0, 0.0)]:
                    if r is None:
                        run += 1
                        continue
                    flags += [True] * run if run < 3 else []
                    flags.append(False)
                    run = 0
                flags = flags[:-1] + ([True] * run if 0 < run < 3 else [])
                rt_loss = sum(flags) / len(flags) if flags else 1.0
                # a heartbeat must survive both directions, so convert the
                # round-trip loss to a one-way packet loss estimate, which is
                # what the per-traffic limits (e.g. video max_loss) refer to
                loss = 1.0 - (1.0 - rt_loss) ** 0.5
                alive = st.miss_streak < 3 and bool(rtts)
                report_fresh = now - st.last_report < 1.0
                out[name] = dict(
                    alive=alive,
                    rtt_ms=float(np.median(rtts)) if rtts else 9999.0,
                    rtt_p90=float(np.percentile(rtts, 90)) if rtts else 9999.0,
                    rtt_min10=float(min(r for _, r in st.rtt10)) if st.rtt10 else None,
                    loss=loss,
                    miss_streak=st.miss_streak,
                    cap_kbps=st.cap_kbps if (alive and report_fresh) else 0.0,
                    delivered_kbps=st.delivered_kbps,
                    data_loss=st.data_loss,
                    wifi_signal=st.wifi_signal, wifi_txrate=st.wifi_txrate,
                    wifi_retries=st.wifi_retries, lte_quality=st.lte_quality,
                    signal=st.wifi_signal if st.wifi_signal is not None else st.lte_quality,
                )
        return out
