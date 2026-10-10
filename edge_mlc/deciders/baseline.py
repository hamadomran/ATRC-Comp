"""Baseline decision module: ATSSS / SpeedFusion-style steering.

  * fixed link preference order (wifi, then lte)
  * fixed priority classes, as configured on real routers: commands > video > bulk
  * fixed per-traffic limits (commands, video) from config `limits`
  * a link is BAD for a class after `bad_after` violating checks in a row,
    GOOD again after `good_after` clean checks in a row (hysteresis)
  * commands always duplicated on the two best links (ATSSS redundant mode)
  * both cameras always sent; each camera adapts its own quality
    (step down on loss, step up after a quiet period), front has priority
  * bulk uploads sent as soon as they exist, FIFO, on the current video link,
    as a greedy TCP-like flow; no knowledge of manoeuvre or stops
  * no mode awareness, no prediction
"""

class Baseline:
    name = "baseline"
    reject_stale = False

    def __init__(self, cfg, dup_video=False):
        self.cfg = cfg
        self.dup_video = dup_video          # True = SpeedFusion "WAN smoothing": video always on 2 links
        self.b = cfg["baseline"]
        self.order = [l["name"] for l in cfg["links"]]
        self.lim = cfg["limits"]
        self.state = {(l, c): {"bad": False, "bad_n": 0, "good_n": 0}
                      for l in self.order for c in ("cmd", "video")}
        self.level = {"front": self.b["cam_max_level"], "rear": self.b["cam_max_level"]}
        self.last_loss_t = 0.0
        self.video_link = self.order[0]

    def _update_link_states(self, links):
        for (ln, c), st in self.state.items():
            L, lim = links[ln], self.lim[c]
            violating = (not L["alive"]) or L["rtt_ms"] > lim["max_rtt_ms"] or L["loss"] > lim["max_loss"]
            if violating:
                st["bad_n"] += 1
                st["good_n"] = 0
                if st["bad_n"] >= self.b["bad_after"]:
                    st["bad"] = True
            else:
                st["good_n"] += 1
                st["bad_n"] = 0
                if st["good_n"] >= self.b["good_after"]:
                    st["bad"] = False

    def _good(self, c):
        return [l for l in self.order if not self.state[(l, c)]["bad"]]

    def decide(self, s):
        now, links = s["now"], s["links"]
        reasons = []
        self._update_link_states(links)

        # commands: two best links in preference order
        good_cmd = self._good("cmd")
        cmd_links = (good_cmd + [l for l in self.order if l not in good_cmd])[:2]

        # video: first good link in preference order
        good_v = self._good("video")
        new_vlink = good_v[0] if good_v else self.video_link     # nothing good: stay put
        if new_vlink != self.video_link:
            reasons.append(f"video link {self.video_link}->{new_vlink} (threshold)")
            self.video_link = new_vlink

        # each camera adapts itself to data loss on its link (rear steps down first)
        loss = links[self.video_link]["data_loss"]
        if loss > self.b["cam_down_loss"]:
            cam = "rear" if self.level["rear"] >= self.level["front"] else "front"
            if self.level[cam] > self.b["cam_min_level"]:
                self.level[cam] -= 1
                reasons.append(f"{cam} quality down (loss {loss:.2f})")
            self.last_loss_t = now
        elif now - self.last_loss_t > self.b["cam_up_after_s"]:
            cam = "front" if self.level["front"] <= self.level["rear"] else "rear"
            if self.level[cam] < self.b["cam_max_level"]:
                self.level[cam] += 1
                reasons.append(f"{cam} quality up")
            self.last_loss_t = now

        vlinks = [self.video_link]
        if self.dup_video:
            other = [l for l in good_v + self.order if l != self.video_link]
            vlinks += other[:1]
        return {
            "mode": "N/A",
            "cmd_links": cmd_links,
            "telem_link": cmd_links[0],
            "video": {c: {"links": vlinks, "level": self.level[c]} for c in ("front", "rear")},
            # bulk uploads: greedy TCP-like flow on the video link, FIFO, as soon
            # as items exist (the car ignores this key)
            "bulk": ({"link": self.video_link, "mode": "greedy"}
                     if s.get("uploads") else None),
            "cap": 1.0, "status": "N/A", "hb_fast": False,
            "reasons": reasons,
        }
