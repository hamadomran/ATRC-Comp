"""JSON-lines logger: one event per line, every line timestamped."""
import json
import os
import threading
import time


class Logger:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "a", buffering=1)
        self.lock = threading.Lock()

    def log(self, event, **kw):
        kw["t"] = time.time()
        kw["ev"] = event
        with self.lock:
            self.f.write(json.dumps(kw, separators=(",", ":"), default=str) + "\n")
