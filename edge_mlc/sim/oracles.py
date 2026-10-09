"""Oracle deciders: upper bounds for handover, NOT deployable.

They run the real Proposed decider (mode detection, allocation, speed caps) and
then overrule the protected camera's link using GROUND TRUTH from the scenario,
which no real system has. They answer "how much could a perfect handover gain?",
so a result can be judged against what is possible in a scenario.

  OracleLinkChoice  picks the link that will stay within the camera's limits for
                    the next `look` seconds (perfect prediction, one link)
  OracleAllUp       sends the protected camera on every link that is truly up
                    (perfect redundancy, costs extra data)
Both also send commands on every truly-up link.
"""
import numpy as np

from deciders.proposed import Proposed
from sim import scenario as S


class _Oracle(Proposed):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.truth = None                       # set by the simulator (engine.Sim)

    def attach_truth(self, truth):
        self.truth = truth

    def _up(self, now):
        i = min(int(now / S.DT), len(self.truth["t"]) - 1)
        return [ln for ln in self.links if self.truth[ln]["up"][i]]

    def _ok(self, ln, t0, t1, req, kbps):
        """Truly within limits (and capacity) for the whole interval?"""
        L = self.truth[ln]
        i0 = int(t0 / S.DT)
        i1 = max(i0 + 1, int(min(t1, S.T_END) / S.DT))
        return bool(np.all(L["up"][i0:i1]) and np.all(L["loss"][i0:i1] <= req["max_loss"])
                    and np.all(2 * L["owd"][i0:i1] <= req["max_rtt_ms"])
                    and np.all(self.p["headroom"] * L["cap"][i0:i1] >= kbps))

    def pick(self, now, cur, req, kbps):
        raise NotImplementedError

    def decide(self, s):
        plan = super().decide(s)
        now = s["now"]
        needs = self.cfg["modes"].get(plan["mode"], {})
        for cam, n in needs.items():
            v = plan["video"][cam]
            if n["prio"] != 1 or not v["links"]:
                continue
            req = {"max_rtt_ms": n.get("max_rtt_ms", 600), "max_loss": n.get("max_loss", 0.2)}
            kbps = self.cfg["ladder"][v["level"]]["kbps"]
            v["links"] = self.pick(now, v["links"][0], req, kbps) or v["links"]
        up = self._up(now)
        if up:
            plan["cmd_links"] = up
        return plan


class OracleLinkChoice(_Oracle):
    name = "oracle_link"
    look = 0.5

    def pick(self, now, cur, req, kbps):
        good = [ln for ln in self.links if self._ok(ln, now, now + self.look, req, kbps)]
        if good:
            return [cur] if cur in good else [good[0]]
        return self._up(now)[:1]


class OracleAllUp(_Oracle):
    name = "oracle_all_up"

    def pick(self, now, cur, req, kbps):
        return self._up(now)
