"""A link that drops packets at random (stand-in for a real radio, Starlink or 4G link)."""
import numpy as np


class Link:
    def __init__(self, name, delivery=1.0, seed=0):
        self.name = name
        self.delivery = delivery          # share of packets that arrive (0 = link down)
        self.rng = np.random.default_rng(seed)
        self.sent = self.delivered = self.bytes_sent = 0

    def carry(self, packets):
        out = []
        for pk in packets:
            self.sent += 1
            self.bytes_sent += len(pk)
            if self.rng.random() < self.delivery:
                out.append(pk)
        self.delivered += len(out)
        return out
