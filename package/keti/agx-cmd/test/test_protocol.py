#!/usr/bin/env python3
"""Which generation of the AgileX protocol agx-cmd puts on the bus.

The Scout Mini Omni can be either v1 or v2 - the vendor's own SDK detects at
runtime rather than assuming - and the two are not dialects of each other. The
same eight bytes mean different things:

  v2  0x111  linear mm/s as big-endian int16, no checksum
  v1  0x130  linear as a signed percentage of the vehicle's 3.0 m/s maximum,
             a rolling frame counter, and a checksum the vehicle enforces

So sending the wrong one is not a command that fails. Byte 2 of a v1 frame is
`linear_percentage`; in a v2 frame the same offset is the low half of the linear
velocity. A v2 frame for 0.25 m/s is 00 FA ... - which a v1 vehicle reads as 0%
linear and 250% angular, clamped to its maximum yaw rate. That is why this file
exists, and why the daemon refuses to command at all until something has told it
which generation it is talking to.

agx-cmd only transmits, so it cannot detect anything itself without putting a
frame on the bus first - exactly what must not happen. can-bridge is already
listening and already writes what it heard, so the generation is read from there.

Each case starts its own daemon, because the generation is settled once and then
left alone: a value that could change mid-drive would change what every
subsequent command byte means.
"""
import binascii
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

BIN = os.environ.get("AGX_CMD_BIN", "./agx-cmd")
TELE_PORT = 27612
INJECT_PORT = 27711

# What the daemon is configured to call full stick, and what the vehicle's own
# maximum is. They are different numbers on purpose: -L is walking pace, and the
# v1 percentage is a fraction of the vehicle's maximum, not of walking pace.
MAX_LIN, RATE = 0.5, 20
VEHICLE_MAX_LIN = 3.0		# AGX1_MINI_MAX_LINEAR in agilex.h

fails = []


def check(name, got, want):
    ok_ = got == want
    print(f"  {'PASS' if ok_ else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok_:
        fails.append(name)


def ok(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        fails.append(name)


def tele(seq, fwd=0.0, armed=True):
    """One TELE frame, matching teleop.c's emitter."""
    p = bytearray(32)
    p[0:4] = b"TELE"
    p[4] = 1
    p[5] = 1 if armed else 0
    struct.pack_into("<I", p, 8, seq)
    struct.pack_into("<Q", p, 12, int(time.time() * 1000) & 0xFFFFFFFFFFFFFFFF)
    struct.pack_into("<h", p, 22, int(round(fwd * 10000)))
    return bytes(p)


def v1_checksum(can_id, data):
    """The vendor's CalcCanFrameChecksumV1, independently written here."""
    s = (can_id & 0xFF) + ((can_id >> 8) & 0xFF) + 8
    for b in data[:7]:
        s += b
    return s & 0xFF


def run(label, bridge_body, protocol=None, seconds=1.6, stick=1.0):
    """Run the daemon once with a given bridge status, return its inject frames."""
    bridge = tempfile.mkstemp(suffix=".json")[1]
    status = tempfile.mkstemp(suffix=".json")[1]
    if bridge_body is None:
        os.remove(bridge)		# a path that does not exist
    else:
        open(bridge, "w").write(bridge_body)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", INJECT_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    argv = [BIN, "-f", "-l", str(TELE_PORT), "-i", f"127.0.0.1:{INJECT_PORT}",
            "-L", str(MAX_LIN), "-H", str(RATE), "-t", "300",
            "-S", status, "--bridge-status", bridge]
    if protocol:
        argv += ["--protocol", protocol]
    proc = subprocess.Popen(argv, stderr=subprocess.PIPE, text=True)
    time.sleep(0.4)

    frames, seq, t0 = [], 0, time.time()
    while time.time() - t0 < seconds:
        seq += 1
        tx.sendto(tele(seq, fwd=stick), ("127.0.0.1", TELE_PORT))
        time.sleep(0.02)
        while True:
            try:
                d = rx.recv(2048)
            except BlockingIOError:
                break
            (can_id,) = struct.unpack_from("<I", d, 8)
            frames.append((can_id, d[16:24]))

    proc.terminate()
    proc.wait(timeout=3)
    log = proc.stderr.read()
    rx.close()
    tx.close()
    st = json.load(open(status)) if os.path.getsize(status) else {}
    for f in (bridge, status):
        if os.path.exists(f):
            os.remove(f)

    print(f"\n--- {label} ---")
    print(f"  {len(frames)} frames, status protocol={st.get('protocol')} "
          f"skips={st.get('protocol_skips')} applied={st.get('applied')}")
    if frames:
        cid, dat = frames[-1]
        print(f"  last: id={cid:03X} data={binascii.hexlify(dat).decode()}")
    return frames, st, log


def main():
    # ---- 1. nothing has said which generation ----
    #
    # The operator is armed and asking for full stick. Withholding is the whole
    # point: the alternative is a one-in-two chance of a command that means
    # something else entirely.
    f, st, log = run("no bridge status", None)
    ok("withholds every command", len(f) == 0, f"{len(f)} frames")
    ok("counts what it withheld", st.get("protocol_skips", 0) > 0,
       f"skips={st.get('protocol_skips')}")
    check("reports the generation as unknown", st.get("protocol"), "unknown")
    ok("still reads teleop while withholding", st.get("applied", 0) > 0,
       f"applied={st.get('applied')}")
    ok("says why, once", log.count("not commanding") == 1,
       f"{log.count('not commanding')} lines")

    # ---- 2. the bus said both, which the vendor calls UNKNOWN ----
    f, _, _ = run("bridge says unknown", '{\n\t"agilex_protocol": "unknown"\n}\n')
    ok("withholds on a conflicted bus", len(f) == 0, f"{len(f)} frames")

    # A truncated or unrelated file must not be read as a generation either.
    f, _, _ = run("bridge status is garbage", "not json at all\n")
    ok("withholds on an unparseable status", len(f) == 0, f"{len(f)} frames")

    # ---- 3. v1 ----
    f, st, _ = run("bridge says v1", '{\n\t"agilex_protocol": "v1"\n}\n')
    ok("commands once the generation is known", len(f) > 3, f"{len(f)} frames")
    check("uses the v1 command id", sorted({c for c, _ in f}), [0x130])
    check("settles on v1", st.get("protocol"), "v1")
    dat = f[-1][1]
    check("control mode is CAN", dat[0], 1)
    check("clears no errors", dat[1], 0)
    # Full stick is -L, and the percentage is of the VEHICLE's maximum. Those
    # being different numbers is the mistake this pins: dividing by -L instead
    # would send 100% - full speed on the first command.
    want_pct = round(MAX_LIN / VEHICLE_MAX_LIN * 100)
    check("linear is a percentage of the vehicle maximum",
          struct.unpack("b", dat[2:3])[0], want_pct)
    ok("which is not full scale", want_pct < 50, f"{want_pct}%")
    counts = [d[6] for _, d in f]
    ok("the frame counter rolls forward",
       all((counts[i + 1] - counts[i]) % 256 == 1 for i in range(len(counts) - 1)),
       str(counts[:8]))
    ok("every frame carries a valid checksum",
       all(d[7] == v1_checksum(c, d) for c, d in f))

    # ---- 4. v2 ----
    f, st, _ = run("bridge says v2", '{\n\t"agilex_protocol": "v2"\n}\n')
    check("uses the v2 command id", sorted({c for c, _ in f}), [0x111])
    check("settles on v2", st.get("protocol"), "v2")
    lin_mm = struct.unpack_from(">h", f[-1][1], 0)[0]
    check("linear is mm/s, not a percentage", lin_mm, round(MAX_LIN * 1000))
    ok("and byte 7 is not a checksum", f[-1][1][7] == 0,
       f"{f[-1][1][7]:02X}")

    # ---- 5. an explicit generation needs no detector ----
    #
    # For a bench with no can-bridge running, and for the case where the answer
    # is already known and the operator does not want to depend on a second
    # daemon's file to move.
    f, st, _ = run("--protocol v1 overrides, with no bridge status at all",
                   None, protocol="v1")
    ok("commands without any detection", len(f) > 3, f"{len(f)} frames")
    check("and it is v1", sorted({c for c, _ in f}), [0x130])

    # ---- 6. the resolved generation does not drift ----
    #
    # A bridge file that changes generation mid-run must not change what the
    # command bytes mean. Written v2 first, then v1 while running.
    bridge = tempfile.mkstemp(suffix=".json")[1]
    open(bridge, "w").write('{\n\t"agilex_protocol": "v2"\n}\n')
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", INJECT_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    proc = subprocess.Popen(
        [BIN, "-f", "-l", str(TELE_PORT), "-i", f"127.0.0.1:{INJECT_PORT}",
         "-L", str(MAX_LIN), "-H", str(RATE), "-t", "300",
         "--bridge-status", bridge], stderr=subprocess.DEVNULL, text=True)
    time.sleep(0.4)
    ids, seq, t0, flipped = set(), 0, time.time(), False
    while time.time() - t0 < 1.6:
        seq += 1
        tx.sendto(tele(seq, fwd=1.0), ("127.0.0.1", TELE_PORT))
        time.sleep(0.02)
        if not flipped and time.time() - t0 > 0.6:
            flipped = True
            open(bridge, "w").write('{\n\t"agilex_protocol": "v1"\n}\n')
        while True:
            try:
                d = rx.recv(2048)
            except BlockingIOError:
                break
            ids.add(struct.unpack_from("<I", d, 8)[0])
    proc.terminate()
    proc.wait(timeout=3)
    rx.close()
    tx.close()
    os.remove(bridge)
    print(f"\n--- bridge changes its mind mid-run ---")
    print(f"  ids seen: {[f'{i:03X}' for i in sorted(ids)]}")
    check("the generation is settled once and kept", sorted(ids), [0x111])

    print()
    if fails:
        print(f"FAILED ({len(fails)}): " + ", ".join(fails))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
