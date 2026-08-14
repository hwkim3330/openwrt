#!/usr/bin/env python3
"""Safety properties of agx-cmd, which is the only code here that can move a
vehicle.

The daemon is run for real, fed TELE frames over a socket, and the inject
datagrams it produces are decoded back into physical units. Every check below is
a way this could hurt someone or something:

  - emitting while disarmed
  - full stick meaning full speed rather than the configured limit
  - a step input arriving at the wheels as a step
  - losing the operator and leaving the last command standing
  - strafing the wrong way
  - exiting with a non-zero command as the last thing on the bus
"""
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

BIN = os.environ.get("AGX_CMD_BIN", "./agx-cmd")
TELE_PORT = 27602
INJECT_PORT = 27701
STATUS = tempfile.mkstemp(suffix=".json")[1]

MAX_LIN, MAX_LAT, MAX_ANG, ACCEL, RATE = 1.0, 1.0, 2.0, 2.0, 50

fails = []


def check(name, got, want, tol=None):
    ok = abs(got - want) <= tol if tol is not None else got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


def ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        fails.append(name)


def tele(seq, armed, strafe=0.0, fwd=0.0, yaw=0.0):
    """One TELE frame, matching teleop.c's emitter."""
    p = bytearray(32)
    p[0:4] = b"TELE"
    p[4] = 1
    p[5] = 1 if armed else 0
    struct.pack_into("<I", p, 8, seq)
    struct.pack_into("<Q", p, 12, int(time.time() * 1000) & 0xFFFFFFFFFFFFFFFF)
    for i, v in enumerate((strafe, fwd, yaw, 0.0)):
        struct.pack_into("<h", p, 20 + i * 2, int(round(v * 10000)))
    return bytes(p)


def decode_inject(d):
    """A BCAN inject datagram -> (can_id, linear, angular, lateral) in SI."""
    assert len(d) >= 24, len(d)
    (magic,) = struct.unpack_from("<I", d, 0)
    assert magic == 0x4E414342, hex(magic)
    assert d[4] == 1
    cnt = d[5]
    assert cnt == 1, cnt
    (can_id,) = struct.unpack_from("<I", d, 8)
    dlc = d[12]
    assert dlc == 8, dlc
    body = d[16:24]
    lin, ang, lat, steer = struct.unpack(">4h", body)
    return can_id, lin / 1000.0, ang / 1000.0, lat / 1000.0, steer


def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", INJECT_PORT))
    rx.settimeout(0.6)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # --protocol v2 is not a default being papered over: decode_inject() below
    # reads mm/s big-endian, which is the v2 command, and every physical check
    # in this file is expressed in those units. Without it the daemon would
    # correctly refuse to command anything at all, because nothing here is
    # publishing what generation the bus is - that refusal is its own test, in
    # test_protocol.py.
    proc = subprocess.Popen(
        [BIN, "-f", "-l", str(TELE_PORT), "-i", f"127.0.0.1:{INJECT_PORT}",
         "-L", str(MAX_LIN), "-X", str(MAX_LAT), "-A", str(MAX_ANG),
         "-a", str(ACCEL), "-H", str(RATE), "-t", "300", "-S", STATUS,
         "--protocol", "v2"],
        stderr=subprocess.PIPE, text=True)
    time.sleep(0.7)
    if proc.poll() is not None:
        print("  FAIL  agx-cmd exited immediately:", proc.stderr.read())
        return 1

    seq = [0]

    def send(**kw):
        seq[0] += 1
        tx.sendto(tele(seq[0], **kw), ("127.0.0.1", TELE_PORT))

    def drain(n=1, timeout=0.6):
        out = []
        rx.settimeout(timeout)
        for _ in range(n):
            try:
                out.append(decode_inject(rx.recvfrom(2048)[0]))
            except socket.timeout:
                break
        return out

    def drain_all(timeout=0.5):
        """Every queued frame, newest last.

        drain(n) returns the OLDEST n, which for a held stick is the start of
        the ramp rather than where it settled - a test that read those and
        expected the limit was measuring its own impatience."""
        out = []
        rx.settimeout(timeout)
        while True:
            try:
                out.append(decode_inject(rx.recvfrom(2048)[0]))
            except socket.timeout:
                return out

    def flush():
        rx.settimeout(0.01)
        try:
            while True:
                rx.recvfrom(2048)
        except (socket.timeout, BlockingIOError):
            pass

    try:
        # ---- 1. silence while disarmed ----
        print("\n--- disarmed ---")
        for _ in range(10):
            send(armed=False, fwd=1.0)
            time.sleep(0.02)
        got = drain(2, timeout=0.4)
        ok("nothing emitted while disarmed", len(got) == 0)

        # ---- 2. armed, and the id and units are right ----
        print("\n--- armed ---")
        flush()
        # Hold the stick so the ramp has time to reach the limit.
        deadline = time.time() + 1.6
        while time.time() < deadline:
            send(armed=True, fwd=1.0)
            time.sleep(0.02)
        got = drain_all()
        ok("frames are emitted when armed", len(got) >= 3)
        if got:
            # The peak, not the first or the last. The first is the start of the
            # ramp and the last is the deadman's stop, because draining outlasts
            # the 300 ms timeout - so neither end says whether the limit was
            # reached while the stick was actually held.
            can_id, lin, ang, lat, steer = max(got, key=lambda f: abs(f[1]))
            check("can id is the motion command", can_id, 0x111)
            check("full stick reaches the configured limit", lin, MAX_LIN,
                  tol=0.05)
            check("no yaw was asked for", ang, 0.0, tol=0.001)
            check("no strafe was asked for", lat, 0.0, tol=0.001)
            check("steering unused", steer, 0)
            ok("limit is not exceeded", lin <= MAX_LIN + 1e-6)

        # ---- 3. the ramp ----
        # Back to neutral, then a step to full: the first frames must be
        # partial, not full. accel 2.0 at 50 Hz is 0.04 per frame, so full
        # speed cannot be reached inside two frames.
        print("\n--- slew limiting ---")
        deadline = time.time() + 0.8
        while time.time() < deadline:
            send(armed=True, fwd=0.0)
            time.sleep(0.02)
        flush()
        t0 = time.time()
        while time.time() < t0 + 0.12:
            send(armed=True, fwd=1.0)
            time.sleep(0.02)
        got = drain(3, timeout=0.5)
        ok("a step input does not arrive as a step",
           bool(got) and got[0][1] < MAX_LIN * 0.6)
        if len(got) >= 2:
            ok("and it is climbing", got[1][1] > got[0][1])

        # ---- 4. the deadman ----
        print("\n--- deadman ---")
        deadline = time.time() + 1.6
        while time.time() < deadline:
            send(armed=True, fwd=1.0)
            time.sleep(0.02)
        flush()
        # Stop sending. Commands must decay to zero and then stop entirely.
        time.sleep(1.2)
        got = drain(40, timeout=0.4)
        ok("deadman produced frames on the way down", len(got) > 0)
        if got:
            ok("the last command before going quiet is a stop",
               abs(got[-1][1]) < 1e-6 and abs(got[-1][2]) < 1e-6
               and abs(got[-1][3]) < 1e-6)
        flush()
        time.sleep(0.8)
        ok("and then it is silent", len(drain(1, timeout=0.4)) == 0)

        # ---- 5. the strafe sign ----
        # a0 is +right; AgileX takes +lateral as left, so a right stick has to
        # produce a negative lateral. This is the one convention in the path
        # that has not been checked against a vehicle, so the test at least
        # pins what the code intends.
        print("\n--- strafe sign ---")
        deadline = time.time() + 1.6
        while time.time() < deadline:
            send(armed=True, strafe=1.0)
            time.sleep(0.02)
        got = drain_all()
        if got:
            peak = max(got, key=lambda f: abs(f[3]))
            ok("stick right gives negative lateral (left-positive)",
               peak[3] < -0.5)
            check("strafe reaches its own limit", abs(peak[3]), MAX_LAT,
                  tol=0.05)
            ok("lateral limit is not exceeded", abs(peak[3]) <= MAX_LAT + 1e-6)

        # ---- 6. replay ----
        print("\n--- replay ---")
        import json
        before = json.load(open(STATUS))["rejected_seq"]
        tx.sendto(tele(1, True, fwd=1.0), ("127.0.0.1", TELE_PORT))
        time.sleep(0.3)
        after = json.load(open(STATUS))["rejected_seq"]
        ok("an old sequence number is rejected", after > before)

        # ---- 7. the last thing on the bus is a stop ----
        print("\n--- shutdown ---")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            send(armed=True, fwd=1.0)
            time.sleep(0.02)
        flush()
        proc.terminate()
        proc.wait(timeout=4)
        got = drain(6, timeout=0.6)
        ok("exit emits a command", len(got) > 0)
        if got:
            ok("and it is zero",
               all(abs(v) < 1e-6 for v in got[-1][1:4]))

    finally:
        if proc.poll() is None:
            proc.kill()
        try:
            os.remove(STATUS)
        except OSError:
            pass

    print()
    if fails:
        print(f"FAILED ({len(fails)}): " + ", ".join(fails))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
