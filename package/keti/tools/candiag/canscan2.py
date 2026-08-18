#!/usr/bin/env python3
"""Try every plausible way of listening, not just every bitrate.

What we know: with the vehicle off the channel is clean, and with it on the
receive error counter runs to error-passive within 200 ms at every classic
bitrate tried. So the vehicle is transmitting and none of those settings can
decode it. A wrong bitrate is only one reason for that; the other is that the
frames are not classic CAN at all.

So this sweeps the combinations rather than one axis:

  - classic CAN at eight bitrates
  - CAN FD at the usual arbitration/data pairs (a classic controller reports a
    form error on an FD frame, which looks exactly like a wrong bitrate)
  - listen-only, which does not ACK and does not send error flags, so a
    receiver that is being knocked into error-passive by its own error flags
    still gets to see the traffic

Reports what each combination saw. Stops on the first one that decodes frames.
Run under sudo.
"""
import os
import re
import socket
import struct
import subprocess
import sys
import time

CAN_RAW, CAN_RAW_ERR_FILTER, CAN_RAW_FD_FRAMES = 1, 2, 5
CAN_ERR_FLAG, CAN_EFF_FLAG = 0x20000000, 0x80000000
DWELL = 3.0

CLASSIC = [500000, 250000, 1000000, 125000, 800000, 100000, 50000, 20000]
FD = [(500000, 2000000), (500000, 5000000), (1000000, 4000000),
      (250000, 1000000), (500000, 1000000)]

V1_SYSTEM, V1_MOTION, V1_CMD = 0x151, 0x131, 0x130
V2_MOTION, V2_SYSTEM, V2_RC, V2_CMD = 0x221, 0x211, 0x241, 0x111
LABEL = {V1_SYSTEM: "v1 system state (generation marker)",
         V1_MOTION: "v1 motion state / v2 brake command",
         V1_CMD: "v1 motion command",
         V2_MOTION: "v2 motion state (generation marker)",
         V2_SYSTEM: "v2 system state",
         V2_RC: "v2 RC state (generation marker)",
         V2_CMD: "v2 motion command"}


def sh(*a):
    return subprocess.run(a, capture_output=True, text=True).stdout


def berr(i):
    m = re.search(r"berr-counter tx (\d+) rx (\d+)",
                  sh("ip", "-d", "link", "show", i))
    return (int(m.group(1)), int(m.group(2))) if m else (0, 0)


def state(i):
    m = re.search(r"can state (\S+)", sh("ip", "-d", "link", "show", i))
    return m.group(1) if m else "?"


def v1_checksum(cid, d):
    t = (cid & 0xFF) + ((cid >> 8) & 0xFF) + len(d)
    for b in d[:len(d) - 1]:
        t += b
    return t & 0xFF


def configure(iface, args):
    sh("ip", "link", "set", iface, "down")
    r = subprocess.run(["ip", "link", "set", iface, "up", "type", "can"] + args,
                       capture_output=True, text=True)
    return r.returncode == 0, r.stderr.strip()


def listen(iface, secs, fd_mode):
    try:
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
        s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                     struct.pack("=I", 0x1FFFFFFF))
        if fd_mode:
            try:
                s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                             struct.pack("=i", 1))
            except OSError:
                pass
        s.bind((iface,))
        s.settimeout(0.2)
    except OSError as e:
        return {}, 0, str(e)
    frames, errs = {}, 0
    t0 = time.time()
    while time.time() - t0 < secs:
        try:
            raw = s.recv(72)
        except socket.timeout:
            continue
        except OSError:
            break
        if len(raw) < 8:
            continue
        cid, dlc = struct.unpack("=IB3x", raw[:8])
        if cid & CAN_ERR_FLAG:
            errs += 1
            continue
        key = cid & (0x1FFFFFFF if cid & CAN_EFF_FLAG else 0x7FF)
        e = frames.setdefault(key, {"n": 0, "last": b""})
        e["n"] += 1
        e["last"] = raw[8:8 + min(dlc, 64)]
    s.close()
    return frames, errs, ""


def report(iface, how, frames):
    total = sum(v["n"] for v in frames.values())
    print(f"\n  *** DECODED on {iface} with {how}: {total} frames ***")
    for k in sorted(frames):
        e = frames[k]
        ck = ""
        if len(e["last"]) == 8:
            ck = "  ck:ok" if e["last"][7] == v1_checksum(k, e["last"]) else "  ck:no"
        lab = "  <- " + LABEL[k] if k in LABEL else ""
        print(f"    {k:03X}  n={e['n']:<6d} {e['last'].hex(' ')}{ck}{lab}")


def main():
    ifaces = [i for i in ("can0", "can1") if os.path.exists(f"/sys/class/net/{i}")]
    if not ifaces:
        print("no can interfaces")
        return 1

    combos = []
    for br in CLASSIC:
        combos.append((f"classic {br}", ["bitrate", str(br), "restart-ms", "100"], False))
        combos.append((f"classic {br} listen-only",
                       ["bitrate", str(br), "listen-only", "on", "restart-ms", "100"], False))
    for br, dbr in FD:
        combos.append((f"fd {br}/{dbr}",
                       ["bitrate", str(br), "dbitrate", str(dbr), "fd", "on",
                        "restart-ms", "100"], True))
        combos.append((f"fd {br}/{dbr} listen-only",
                       ["bitrate", str(br), "dbitrate", str(dbr), "fd", "on",
                        "listen-only", "on", "restart-ms", "100"], True))

    print(f"{len(combos)} combinations x {len(ifaces)} channels, "
          f"{DWELL:.0f}s each\n", flush=True)
    for rnd in range(1, 50):
        for how, args, fd_mode in combos:
            for iface in ifaces:
                ok, err = configure(iface, args)
                if not ok:
                    print(f"  {iface:5s} {how:28s} unsupported ({err})", flush=True)
                    continue
                frames, errs, sockerr = listen(iface, DWELL, fd_mode)
                n = sum(v["n"] for v in frames.values())
                tx, rx = berr(iface)
                st = state(iface)
                verdict = ("FRAMES" if n else
                           "edges, undecodable" if (errs or rx or tx) else
                           "silent")
                print(f"  r{rnd} {iface:5s} {how:28s} frames={n:<5d} "
                      f"errfr={errs:<4d} berr=tx{tx}/rx{rx:<4d} {st:<14s} {verdict}",
                      flush=True)
                if n:
                    report(iface, how, frames)
                    return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
