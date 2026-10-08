"""One UDP socket per physical link, pinned to its interface.

SO_BINDTODEVICE forces packets out of that interface (needs root),
so the same destination (the hub) is reached over Wi-Fi OR 4G on demand.
"""
import random
import socket
import threading
import time

from . import protocol as P

SO_BINDTODEVICE = 25


class Link:
    def __init__(self, name, link_id, iface, hub_addr, bind_device=True, local_ip=""):
        self.name, self.id, self.iface = name, link_id, iface
        self.hub = hub_addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        if bind_device and iface:
            self.sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, iface.encode() + b"\0")
        self.sock.bind((local_ip, 0))
        self.lseq = 0
        self.bytes_sent = 0
        self.sim_loss = 0.0          # --sim only: drop this share of sent packets
        self.lock = threading.Lock()

    def send(self, stream, seq, payload, flags=0):
        with self.lock:
            self.lseq += 1
            pkt = P.pack(stream, self.id, seq, self.lseq, time.time(), payload, flags)
            self.bytes_sent += len(pkt)
        if self.sim_loss and random.random() < self.sim_loss:
            return False
        try:
            self.sock.sendto(pkt, self.hub)
            return True
        except OSError:          # interface down / no route: treated as loss
            return False

    def start_rx(self, callback):
        def loop():
            while True:
                try:
                    data, _ = self.sock.recvfrom(65535)
                except OSError:
                    time.sleep(0.05)
                    continue
                m = P.unpack(data)
                if m:
                    m["rx_link"] = self.name
                    callback(m)
        threading.Thread(target=loop, daemon=True).start()
