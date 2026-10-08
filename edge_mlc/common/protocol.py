"""Wire format shared by car (Pi), hub and operator.

Every UDP packet = 22-byte header + payload.
  magic  2s  b'EM'
  ver    B   1
  stream B   which data (see constants)
  link   B   link id the packet was SENT on (1=wifi, 2=lte)
  flags  B   bit0 = duplicate copy
  seq    I   per-stream sequence number (frame number for video)
  lseq   I   per-link sequence number (hub uses gaps to measure loss)
  t_send d   sender wall-clock time (time.time()); clocks synced by chrony
"""
import json
import struct

HDR = struct.Struct("!2sBBBBIId")
MAGIC, VER = b"EM", 1

CMD, FRONT, REAR, TELEM, HB, HB_ECHO, REPORT, CTRL, HELLO = range(1, 10)
VIDEO = {FRONT: "front", REAR: "rear"}
STREAM_ID = {"front": FRONT, "rear": REAR}
FLAG_DUP = 1

VFRAG = struct.Struct("!HHB")   # frag_idx, frag_count, level
MAX_CHUNK = 1100                 # keeps packets under typical MTU


def pack(stream, link, seq, lseq, t_send, payload, flags=0):
    return HDR.pack(MAGIC, VER, stream, link, flags, seq & 0xFFFFFFFF,
                    lseq & 0xFFFFFFFF, t_send) + payload


def unpack(data):
    if len(data) < HDR.size:
        return None
    magic, ver, stream, link, flags, seq, lseq, t = HDR.unpack_from(data)
    if magic != MAGIC or ver != VER:
        return None
    return dict(stream=stream, link=link, flags=flags, seq=seq, lseq=lseq,
                t_send=t, payload=data[HDR.size:])


def jdump(obj):
    return json.dumps(obj, separators=(",", ":")).encode()


def jload(b):
    return json.loads(b.decode())
