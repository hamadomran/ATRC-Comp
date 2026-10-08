"""Scheduled, repeatable link impairments with tc netem (run on the Pi as root).

  sudo python3 tools/impair.py --schedule tools/course_schedule.yaml --run r01
  sudo python3 tools/impair.py --clear

Impairs BOTH directions of an interface:
  * outgoing (car -> hub: video, heartbeats) with netem on the interface
  * incoming (hub -> car: commands, echoes) by redirecting ingress to an ifb device
Start it at the same moment as the operator replay, so the same impairment
lands at the same point on the course every run.
"""
import argparse
import subprocess
import time

import yaml

IFB = {"wlan0": "ifb0", "usb0": "ifb1"}   # one ifb per impaired interface


def sh(cmd, check=False):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and r.returncode:
        raise RuntimeError(f"{cmd}: {r.stderr}")
    return r


def setup_ifb(iface):
    ifb = IFB[iface]
    sh("modprobe ifb numifbs=2")
    sh(f"ip link set {ifb} up")
    sh(f"tc qdisc del dev {iface} ingress")
    sh(f"tc qdisc add dev {iface} handle ffff: ingress", check=True)
    sh(f"tc filter add dev {iface} parent ffff: protocol all u32 match u32 0 0 "
       f"action mirred egress redirect dev {ifb}", check=True)


def apply(iface, netem):
    for dev in (iface, IFB[iface]):
        sh(f"tc qdisc replace dev {dev} root netem {netem}", check=True)


def clear(iface):
    for dev in (iface, IFB[iface]):
        sh(f"tc qdisc del dev {dev} root")
    sh(f"tc qdisc del dev {iface} ingress")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--schedule")
    ap.add_argument("--clear", action="store_true")
    ap.add_argument("--run", default="manual")
    a = ap.parse_args()
    if a.clear:
        for i in IFB:
            clear(i)
        return
    sched = yaml.safe_load(open(a.schedule))
    ifaces = {e["iface"] for e in sched["events"]}
    for i in ifaces:
        setup_ifb(i)
    log = open(f"logs/{a.run}/impair.jsonl", "a")
    t0 = time.time()
    try:
        for e in sorted(sched["events"], key=lambda e: e["at"]):
            time.sleep(max(0, t0 + e["at"] - time.time()))
            apply(e["iface"], e["netem"])
            log.write(f'{{"t":{time.time()},"iface":"{e["iface"]}","netem":"{e["netem"]}"}}\n')
            print(f"{e['at']:6.1f}s {e['iface']}: {e['netem']}")
        time.sleep(sched.get("tail_s", 5))
    finally:
        for i in ifaces:
            clear(i)


if __name__ == "__main__":
    main()
