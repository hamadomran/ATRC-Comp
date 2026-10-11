"""Two-stream multi-link video codec: sender side and receiver side.

Every captured frame is encoded twice:

  base  a small JPEG (default 320x240). It goes whole on EVERY link, so any
        one link that delivers it gives a usable picture, upscaled to full
        size by interpolation.
  full  the full-resolution frame cut into 16-pixel horizontal stripes, each
        stripe its own JPEG (split into halves or quarters if it would not
        fit one packet), packed whole into fixed-size packets. Every
        packet that arrives therefore sharpens its stripes on its own. The
        packets are the data symbols of a systematic Reed-Solomon code
        (zfec); parity packets are added, so once enough packets arrive from
        any links, the missing ones are rebuilt and the whole frame is sharp.

Links: with one link everything goes on it. With more, link 1 carries the
base and the parity packets, and the other links carry the base and the data
packets. The receiver shows, per frame:

  >= k packets of the full stream    -> every stripe (rebuilt by FEC if needed)
  fewer                              -> upscaled base, with every stripe that
                                        did arrive pasted on top
  nothing                            -> the previous picture is held

Stripes of one width share one JPEG header (same size and quality), so it is not sent:
both ends rebuild it from the settings. Packets are plain bytes (10-byte
header + payload), ready for UDP.
"""
import math
import struct

import cv2
import numpy as np
import zfec

PACKET = 1100                      # payload bytes per packet
HDR = struct.Struct("!IBBHH")      # frame, kind, index, k (data packets), m (all packets)
REC = struct.Struct("!BBH")        # record inside a data packet: stripe, piece (split level << 4 | index), length
MAX_SPLIT = 2                      # a stripe too big for one packet is split in halves, at most into quarters
KIND_BASE, KIND_FULL = 0, 1


def _jpeg(img, q):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(q)])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()


def stripe_header(width, height, q):
    """The JPEG header every stripe of this size and quality starts with."""
    j = _jpeg(np.zeros((height, width, 3), np.uint8), q)
    sos = j.find(b"\xff\xda")
    return j[:sos + 2 + int.from_bytes(j[sos + 2:sos + 4], "big")]


class Sender:
    """Turns frames into per-link packet lists."""

    def __init__(self, n_links, full_size=(640, 480), base_size=(320, 240),
                 full_q=60, base_q=50, stripe_h=16, parity=0.5):
        self.n_links = n_links
        self.full_size, self.base_size = full_size, base_size
        self.full_q, self.base_q = full_q, base_q
        self.stripe_h = stripe_h
        self.n_stripes = full_size[1] // stripe_h
        self.parity = parity if n_links > 1 else 0.0
        self.headers = [stripe_header(full_size[0] >> lv, stripe_h, full_q) for lv in range(MAX_SPLIT + 1)]

    def _pieces(self, tile, level=0, j=0):
        """Stripe JPEG bodies (shared header removed), split into left/right
        halves until each piece fits one packet. -> [(piece id, body)]"""
        body = _jpeg(tile, self.full_q)[len(self.headers[level]):]
        if len(body) <= PACKET - REC.size or level == MAX_SPLIT:
            return [((level << 4) | j, body[:PACKET - REC.size])]
        h = tile.shape[1] // 2
        return (self._pieces(tile[:, :h], level + 1, 2 * j) +
                self._pieces(tile[:, h:], level + 1, 2 * j + 1))

    def layout(self, k, m):
        """Packet indices per link."""
        if self.n_links == 1:
            return [list(range(m))]
        out = [list(range(k, m))] + [[] for _ in range(self.n_links - 1)]
        for i in range(k):
            out[1 + i % (self.n_links - 1)].append(i)
        return out

    def encode(self, frame_no, img):
        """-> list (one per link) of packet lists for this frame."""
        full = cv2.resize(img, self.full_size, interpolation=cv2.INTER_AREA)
        # pack whole stripes into packets (a stripe never spans two packets)
        packets, cur = [], b""
        for i in range(self.n_stripes):
            for piece, body in self._pieces(full[i * self.stripe_h:(i + 1) * self.stripe_h]):
                rec = REC.pack(i, piece, len(body)) + body
                if len(cur) + len(rec) > PACKET:
                    packets.append(cur)
                    cur = b""
                cur += rec
        packets.append(cur)
        k = len(packets)
        m = k + (math.ceil(self.parity * k) if self.parity else 0)
        blocks = [p + bytes(PACKET - len(p)) for p in packets]
        if m > k:
            blocks = zfec.Encoder(k, m).encode(blocks)

        base = _jpeg(cv2.resize(img, self.base_size, interpolation=cv2.INTER_AREA), self.base_q)
        nb = max(1, math.ceil(len(base) / PACKET))
        base_pk = [HDR.pack(frame_no, KIND_BASE, i, nb, nb) + base[i * PACKET:(i + 1) * PACKET] for i in range(nb)]
        return [base_pk + [HDR.pack(frame_no, KIND_FULL, i, k, m) + blocks[i] for i in idx]
                for idx in self.layout(k, m)]


class Receiver:
    """Rebuilds the best picture it can from whatever packets arrived."""

    def __init__(self, full_size=(640, 480), full_q=60, stripe_h=16):
        self.full_size, self.stripe_h = full_size, stripe_h
        self.n_stripes = full_size[1] // stripe_h
        self.headers = [stripe_header(full_size[0] >> lv, stripe_h, full_q) for lv in range(MAX_SPLIT + 1)]
        self.last = np.zeros((full_size[1], full_size[0], 3), np.uint8)

    def decode(self, packets):
        """packets: all packets received for ONE frame, any order, any link.
        -> (image, info dict)"""
        base_parts, full, k, m, nb = {}, {}, None, None, 0
        for pk in packets:
            _, kind, idx, a, b = HDR.unpack_from(pk)
            if kind == KIND_BASE:
                base_parts[idx], nb = pk[HDR.size:], a
            else:
                full[idx], k, m = pk[HDR.size:], a, b
        base = b"".join(base_parts[i] for i in range(nb)) if base_parts and len(base_parts) == nb else None

        data = {i: full[i] for i in full if i < (k or 0)}
        rebuilt = False
        if k and len(full) >= k and len(data) < k:
            nums = sorted(full)[:k]
            data = dict(enumerate(zfec.Decoder(k, m).decode([full[j] for j in nums], nums)))
            rebuilt = True
        pieces = []
        for blk in data.values():
            off = 0
            while off + REC.size <= len(blk):
                si, piece, n = REC.unpack_from(blk, off)
                if n == 0:
                    break                                       # padding
                pieces.append((si, piece, self.headers[piece >> 4] + blk[off + REC.size:off + REC.size + n]))
                off += REC.size + n
        stripes = {si for si, _, _ in pieces}

        if base is not None:
            small = cv2.imdecode(np.frombuffer(base, np.uint8), cv2.IMREAD_COLOR)
            img = cv2.resize(small, self.full_size, interpolation=cv2.INTER_CUBIC)   # interpolation
        elif stripes:
            img = self.last.copy()
        else:
            return self.last.copy(), {"base": False, "stripes": 0, "rebuilt": False, "complete": False, "held": True}
        W = self.full_size[0]
        for si, piece, j in pieces:
            tile = cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR)
            if tile is not None:
                w = W >> (piece >> 4)
                x = (piece & 0xF) * w
                img[si * self.stripe_h:(si + 1) * self.stripe_h, x:x + w] = tile[:, :w]
        self.last = img
        return img, {"base": base is not None, "stripes": len(stripes), "rebuilt": rebuilt,
                     "complete": bool(k) and len(data) == k, "held": False}
