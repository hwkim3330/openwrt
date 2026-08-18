#!/usr/bin/env python3
"""Everything the bus is doing, live, on one screen.

Top block is the electrical state - it tells you whether signal is arriving at
all, which is a different question from whether it can be decoded:

    frames rising                 decoded, working
    err-pass/warn rising          edges arrive, cannot be decoded
    everything flat at zero       no edges - nothing driving the line, or
                                  CAN_H/CAN_L inverted, which makes every
                                  dominant bit read recessive

Below that, every id seen, with the AgileX meaning where there is one, and the
motion frame decoded into m/s and rad/s so it can be checked against what the
vehicle is actually doing.

    canshow.py [iface] [bitrate]      (default can0 500000)
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

V1_SYSTEM, V1_MOTION, V1_CMD, V1_LIGHT = 0x151, 0x131, 0x130, 0x141
V2_MOTION, V2_SYSTEM, V2_RC, V2_CMD = 0x221, 0x211, 0x241, 0x111
LABEL = {
    0x111: "v2 motion command",
    0x121: "v2 light command",
    0x130: "v1 motion command",
    0x131: "v1 motion state / v2 brake cmd",
    0x140: "v1 light command",
    0x141: "v1 light state",
    0x151: "v1 SYSTEM STATE  -> generation v1",
    0x211: "v2 system state",
    0x221: "v2 MOTION STATE  -> generation v2",
    0x241: "v2 RC state      -> generation v2",
    0x251: "v2 actuator 1 hs", 0x252: "v2 actuator 2 hs",
    0x253: "v2 actuator 3 hs", 0x254: "v2 actuator 4 hs",
    0x261: "v2 actuator 1 ls", 0x262: "v2 actuator 2 ls",
    0x263: "v2 actuator 3 ls", 0x264: "v2 actuator 4 ls",
    0x291: "v2 motion mode state",
    0x311: "v2 odometry",
    0x361: "v2 BMS",
}

iface = sys.argv[1] if len(sys.argv) > 1 else "can0"
bitrate = sys.argv[2] if len(sys.argv) > 2 else "500000"


def sh(*a):
    return subprocess.run(a, capture_output=True, text=True).stdout


def be16s(b, off):
    if len(b) < off + 2:
        return 0
    v = (b[off] << 8) | b[off + 1]
    return v - 0x10000 if v & 0x8000 else v


def v1_ck(cid, d):
    t = (cid & 0xFF) + ((cid >> 8) & 0xFF) + len(d)
    for x in d[:len(d) - 1]:
        t += x
    return t & 0xFF


def electrical():
    out = sh("ip", "-s", "-d", "link", "show", iface)
    m = re.search(r"can state (\S+)", out)
    state = m.group(1) if m else "?"
    m = re.search(r"berr-counter tx (\d+) rx (\d+)", out)
    tx, rx = (m.group(1), m.group(2)) if m else ("?", "?")
    m = re.search(r"re-started bus-errors arbit-lost error-warn error-pass bus-off\s*\n\s*"
                  r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", out)
    cum = m.groups() if m else ("?",) * 6
    return state, tx, rx, cum


# bring the link up fresh
sh("ip", "link", "set", iface, "down")
subprocess.run(["ip", "link", "set", iface, "up", "type", "can",
                "bitrate", bitrate, "restart-ms", "100"],
               capture_output=True, text=True)

s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack("=I", 0x1FFFFFFF))
s.bind((iface,))
s.settimeout(0.02)

seen = {}
total = errfr = 0
t0 = time.time()
last_draw = 0.0
rate_mark, rate_n, rate_hz = t0, 0, 0.0

print(f"listening on {iface} at {bitrate} - ctrl-c to stop\n")
try:
    while True:
        for _ in range(500):
            try:
                raw = s.recv(16)
            except (socket.timeout, BlockingIOError):
                break
            except OSError:
                break
            if len(raw) < 8:
                continue
            cid, dlc = struct.unpack("=IB3x", raw[:8])
            if cid & CAN_ERR_FLAG:
                errfr += 1
                continue
            key = cid & (0x1FFFFFFF if cid & CAN_EFF_FLAG else 0x7FF)
            e = seen.setdefault(key, {"n": 0, "last": b"", "t": time.time()})
            e["n"] += 1
            e["last"] = raw[8:8 + dlc]
            e["t"] = time.time()
            total += 1
            rate_n += 1

        now = time.time()
        if now - rate_mark >= 1.0:
            rate_hz = rate_n / (now - rate_mark)
            rate_mark, rate_n = now, 0

        if now - last_draw >= 0.5:
            last_draw = now
            state, tx, rx, cum = electrical()
            os.system("clear")
            print(f"  {iface} @ {bitrate}      t={now - t0:6.0f}s")
            print(f"  state {state:<14s} berr tx {tx} rx {rx}")
            print(f"  cumulative: restarts {cum[0]}  bus-err {cum[1]}  "
                  f"arb-lost {cum[2]}  warn {cum[3]}  passive {cum[4]}  "
                  f"bus-off {cum[5]}")
            print(f"  frames {total}   {rate_hz:6.1f}/s   error frames {errfr}")
            if not total:
                print("\n  nothing decoded yet.")
                print("  errors flat at zero as well -> no edges on the wire:")
                print("    the pair is not connected, or CAN_H/CAN_L are inverted")
                print("  errors rising -> edges arrive but cannot be decoded:")
                print("    wrong bitrate, or no termination")
            else:
                print(f"\n  {'id':>5} {'n':>7} {'age':>6}  data                     meaning")
                for k in sorted(seen):
                    e = seen[k]
                    age = now - e["t"]
                    ck = ""
                    if len(e["last"]) == 8:
                        ck = " ck" if e["last"][7] == v1_ck(k, e["last"]) else "   "
                    print(f"  {k:5X} {e['n']:7d} {age:5.1f}s  "
                          f"{e['last'].hex(' '):<24s}{ck} {LABEL.get(k, '')}")
                for mid in (V2_MOTION, V1_MOTION):
                    if mid in seen:
                        d = seen[mid]["last"]
                        print(f"\n  motion {mid:03X}:  linear {be16s(d,0)/1000:+7.3f} m/s"
                              f"   angular {be16s(d,2)/1000:+7.3f} rad/s"
                              f"   lateral {be16s(d,4)/1000:+7.3f} m/s")
                if V2_SYSTEM in seen:
                    d = seen[V2_SYSTEM]["last"]
                    if len(d) >= 6:
                        print(f"  system 211:  vehicle {d[0]}  mode {d[1]}  "
                              f"battery {((d[2]<<8)|d[3])/10:.1f} V  "
                              f"error 0x{((d[4]<<8)|d[5]):04X}")
                if V1_SYSTEM in seen:
                    d = seen[V1_SYSTEM]["last"]
                    if len(d) >= 6:
                        print(f"  system 151:  vehicle {d[0]}  mode {d[1]}  "
                              f"battery {((d[2]<<8)|d[3])/10:.1f} V  "
                              f"error 0x{((d[4]<<8)|d[5]):04X}")
                gen = ("v2" if (V2_MOTION in seen or V2_RC in seen)
                       else "v1" if V1_SYSTEM in seen else "not yet decided")
                print(f"\n  protocol generation: {gen}")
        time.sleep(0.02)
except KeyboardInterrupt:
    print("\nstopped")
