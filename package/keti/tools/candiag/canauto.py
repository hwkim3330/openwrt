#!/usr/bin/env python3
"""Wait for the PCAN adapter, bring both channels up, and report what arrives.

Written so the hardware can be handled freely while this runs: unplug the
adapter, swap CAN_H and CAN_L, plug it back in - this notices the interfaces
reappearing, configures them, and says which channel is healthy and which is
erroring. When frames arrive it prints them and identifies the ids.

Run under sudo. Logs one line every few seconds so the state is always visible.
"""
import os
import re
import socket
import struct
import subprocess
import sys
import time

CAN_RAW, CAN_RAW_ERR_FILTER = 1, 2
CAN_ERR_FLAG, CAN_EFF_FLAG = 0x20000000, 0x80000000
BITRATE = 500000
IFACES = ("can0", "can1")

V1_SYSTEM, V1_MOTION, V1_CMD = 0x151, 0x131, 0x130
V2_MOTION, V2_SYSTEM, V2_RC, V2_CMD = 0x221, 0x211, 0x241, 0x111


def sh(*args):
    return subprocess.run(args, capture_output=True, text=True).stdout


def exists(i):
    return os.path.exists(f"/sys/class/net/{i}")


def state(i):
    out = sh("ip", "-d", "link", "show", i)
    m = re.search(r"can state (\S+)", out)
    st = m.group(1) if m else "?"
    m = re.search(r"berr-counter tx (\d+) rx (\d+)", out)
    return st, (f"tx{m.group(1)}/rx{m.group(2)}" if m else "")


def bring_up(i):
    sh("ip", "link", "set", i, "down")
    subprocess.run(["ip", "link", "set", i, "up", "type", "can",
                    "bitrate", str(BITRATE), "restart-ms", "100"],
                   capture_output=True, text=True)


def v1_checksum(can_id, data):
    total = (can_id & 0xFF) + ((can_id >> 8) & 0xFF) + len(data)
    for b in data[:len(data) - 1]:
        total += b
    return total & 0xFF


def be16s(b, off):
    v = (b[off] << 8) | b[off + 1]
    return v - 0x10000 if v & 0x8000 else v


def main():
    print("waiting for the PCAN adapter ...", flush=True)
    up = {}
    socks = {}
    seen = {}
    last = 0.0
    t0 = time.time()
    announced = False

    while time.time() - t0 < 3600:
        # (re)configure whatever is present
        for i in IFACES:
            if exists(i) and i not in up:
                bring_up(i)
                try:
                    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
                    s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                                 struct.pack("=I", 0x1FFFFFFF))
                    s.bind((i,))
                    s.settimeout(0.05)
                    socks[i] = s
                    up[i] = True
                    seen.setdefault(i, {})
                    print(f"  {i} attached and configured at {BITRATE}", flush=True)
                except OSError as e:
                    print(f"  {i}: {e}", flush=True)
            if i in up and not exists(i):
                print(f"  {i} went away", flush=True)
                up.pop(i, None)
                s = socks.pop(i, None)
                if s:
                    s.close()

        for i, s in list(socks.items()):
            for _ in range(200):
                try:
                    raw = s.recv(16)
                except (socket.timeout, BlockingIOError):
                    break
                except OSError:
                    socks.pop(i, None)
                    up.pop(i, None)
                    break
                can_id, dlc = struct.unpack("=IB3x", raw[:8])
                if can_id & CAN_ERR_FLAG:
                    continue
                key = can_id & (0x1FFFFFFF if can_id & CAN_EFF_FLAG else 0x7FF)
                d = seen[i].setdefault(key, {"n": 0, "last": b""})
                d["n"] += 1
                d["last"] = raw[8:8 + dlc]
                if not announced:
                    announced = True
                    print(f"\n  *** {i} IS ALIVE - frames arriving ***\n", flush=True)

        now = time.time()
        if now - last >= 3:
            last = now
            bits = []
            for i in IFACES:
                if not exists(i):
                    bits.append(f"{i}: absent")
                    continue
                st, err = state(i)
                n = sum(v["n"] for v in seen.get(i, {}).values())
                bits.append(f"{i}: {st} {err} frames={n}")
            print("  " + " | ".join(bits), flush=True)
            for i in IFACES:
                for key in sorted(seen.get(i, {})):
                    e = seen[i][key]
                    tag = ""
                    if key == V1_SYSTEM:
                        tag = "  <- v1 system state (generation marker)"
                    elif key in (V2_MOTION, V2_RC):
                        tag = "  <- v2 marker"
                    elif key == V1_CMD:
                        tag = "  <- v1 motion command"
                    elif key == V2_CMD:
                        tag = "  <- v2 motion command"
                    elif key == V1_MOTION:
                        tag = "  <- v1 motion state / v2 brake command"
                    ck = ""
                    if len(e["last"]) == 8:
                        ck = " ck:ok" if e["last"][7] == v1_checksum(key, e["last"]) \
                             else " ck:no"
                    print(f"      {i} {key:03X} n={e['n']:<6d} "
                          f"{e['last'].hex(' ')}{ck}{tag}", flush=True)
        time.sleep(0.05)


if __name__ == "__main__":
    sys.exit(main())
