#!/usr/bin/env python3
"""Raw CAN listener with error frames turned on.

There is no candump on this machine, and `ip -s link` was not telling the truth
about what the controller was hearing - its rx error counter read the same 136
at every bitrate, including ones that cannot both be right. This asks the socket
directly and shows error frames as well as data frames, so "nothing on the wire"
and "something on the wire I cannot decode" look different.

    canwatch.py <iface> [seconds]
"""
import socket
import struct
import sys
import time

CAN_RAW = 1
CAN_RAW_ERR_FILTER = 2
CAN_ERR_FLAG = 0x20000000
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_MASK = 0x1FFFFFFF

ERR_BITS = [
    (0x00000001, "TX timeout"),
    (0x00000002, "lost arbitration"),
    (0x00000004, "controller"),
    (0x00000008, "protocol"),
    (0x00000010, "transceiver"),
    (0x00000020, "no ACK"),
    (0x00000040, "bus off"),
    (0x00000080, "bus error"),
    (0x00000100, "restarted"),
]

iface = sys.argv[1] if len(sys.argv) > 1 else "can0"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack("=I", 0x1FFFFFFF))
s.bind((iface,))
s.settimeout(0.5)

data_n, err_n = 0, 0
ids = {}
errs = {}
t0 = time.time()
while time.time() - t0 < secs:
    try:
        frame = s.recv(16)
    except socket.timeout:
        continue
    can_id, dlc = struct.unpack("=IB3x", frame[:8])
    payload = frame[8:8 + dlc]
    if can_id & CAN_ERR_FLAG:
        err_n += 1
        for bit, name in ERR_BITS:
            if can_id & bit:
                errs[name] = errs.get(name, 0) + 1
        continue
    data_n += 1
    key = can_id & (0x1FFFFFFF if can_id & CAN_EFF_FLAG else 0x7FF)
    e = ids.setdefault(key, {"n": 0, "last": b"", "eff": bool(can_id & CAN_EFF_FLAG)})
    e["n"] += 1
    e["last"] = payload

print(f"{iface}: {data_n} data frames, {err_n} error frames in {secs:.0f}s")
if errs:
    print("  errors: " + ", ".join(f"{k} x{v}" for k, v in sorted(errs.items())))
for k in sorted(ids):
    e = ids[k]
    print(f"  {'x' if e['eff'] else ' '}{k:03X}  n={e['n']:<6d} {e['last'].hex(' ')}")
if not data_n and not err_n:
    print("  nothing at all - no traffic and no bus errors")
