"""Hub server: the far end of the multipath connection (like a SpeedFusion hub).

Car side (car_port):
  * remembers the car's address on EACH link (from the latest packet per link id;
    this also keeps the 4G carrier's NAT mapping open)
  * echoes heartbeats straight back on the same link
  * de-duplicates data (first copy wins) and forwards it to the operator
  * every 200 ms sends each link a REPORT: delivered kbps + data loss on that link
Operator side (op_port):
  * forwards operator commands to the car on the links the car asked for (CTRL)

Run:  python3 hub.py --config config/run.yaml [--bind 0.0.0.0]
"""
import argparse
import socket
import threading
import time
from collections import OrderedDict

import yaml

from common import protocol as P
from common.logger import Logger


class Dedup:
    def __init__(self, size=20000):
        self.seen, self.size = OrderedDict(), size

    def first(self, key):
        if key in self.seen:
            return False
        self.seen[key] = None
        if len(self.seen) > self.size:
            self.seen.popitem(last=False)
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/run.yaml")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--log", default="logs/hub.jsonl")
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    log = Logger(a.log)
    id2name = {l["id"]: l["name"] for l in cfg["links"]}

    car = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    car.bind((a.bind, cfg["hub"]["car_port"]))
    op = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    op.bind((a.bind, cfg["hub"]["op_port"]))

    car_addr = {}                 # link id -> (ip, port)
    op_addr = [None]
    cmd_links = [list(id2name)]   # link ids the car wants commands on
    stats = {lid: {"bytes": 0, "lseq_first": None, "lseq_last": None, "got": 0} for lid in id2name}
    lock = threading.Lock()
    dd = Dedup()
    lseq_out = {lid: 0 for lid in id2name}

    def send_car(lid, stream, seq, payload, flags=0):
        if lid in car_addr:
            lseq_out[lid] += 1
            car.sendto(P.pack(stream, lid, seq, lseq_out[lid], time.time(), payload, flags), car_addr[lid])

    def car_rx():
        while True:
            data, addr = car.recvfrom(65535)
            m = P.unpack(data)
            if not m or m["link"] not in id2name:
                continue
            lid = m["link"]
            with lock:
                car_addr[lid] = addr
                st = stats[lid]
                st["bytes"] += len(data)
                st["got"] += 1
                if st["lseq_first"] is None:
                    st["lseq_first"] = m["lseq"]
                st["lseq_last"] = max(st["lseq_last"] or 0, m["lseq"])
            s = m["stream"]
            if s == P.HB:
                car.sendto(P.pack(P.HB_ECHO, lid, m["seq"], 0, time.time(), b""), addr)
            elif s == P.CTRL:
                ids = [l["id"] for l in cfg["links"] if l["name"] in P.jload(m["payload"])["cmd_links"]]
                cmd_links[0] = ids or list(id2name)
            elif s in (P.FRONT, P.REAR, P.TELEM):
                key = (s, m["seq"], P.VFRAG.unpack_from(m["payload"])[0] if s in P.VIDEO else 0)
                if dd.first(key) and op_addr[0]:
                    op.sendto(data, op_addr[0])          # forward first copy unchanged

    def op_rx():
        while True:
            data, addr = op.recvfrom(65535)
            m = P.unpack(data)
            if not m:
                continue
            op_addr[0] = addr
            if m["stream"] == P.CMD:
                for lid in cmd_links[0]:
                    send_car(lid, P.CMD, m["seq"], m["payload"])
                log.log("cmd_fwd", seq=m["seq"], links=[id2name[l] for l in cmd_links[0]])

    def reporter():
        prev = {lid: (0, None) for lid in id2name}     # (bytes, lseq_last) at previous report
        while True:
            time.sleep(0.2)
            with lock:
                for lid, st in stats.items():
                    b0, l0 = prev[lid]
                    db = st["bytes"] - b0
                    got = st["got"]
                    expected = (st["lseq_last"] - l0) if (l0 is not None and st["lseq_last"]) else got
                    loss = max(0.0, 1 - got / expected) if expected and expected > 0 else 0.0
                    kbps = db * 8 / 0.2 / 1000
                    prev[lid] = (st["bytes"], st["lseq_last"])
                    st["got"] = 0
                    payload = P.jdump({"kbps": round(kbps, 1), "loss": round(loss, 3)})
                    send_car(lid, P.REPORT, 0, payload)
                    log.log("report", link=id2name[lid], kbps=kbps, loss=loss)

    for fn in (car_rx, op_rx, reporter):
        threading.Thread(target=fn, daemon=True).start()
    print(f"hub listening car:{cfg['hub']['car_port']} op:{cfg['hub']['op_port']}")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
