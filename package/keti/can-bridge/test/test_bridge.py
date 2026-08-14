#!/usr/bin/env python3
"""Verify can-bridge against real SocketCAN interfaces.

Needs two virtual CAN interfaces, which cost nothing to create:

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0

The checks that matter:
  - frames on the bus reach the UDP peer, with ids and payloads intact
  - batching works (many frames in one datagram)
  - injection is REFUSED by default, and counted as rejected
  - injection works once --allow-inject is given
  - tracked ids are decoded into the status file
"""
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

BIN = os.environ.get("CAN_BRIDGE_BIN", "./can-bridge")
IFACE = os.environ.get("CAN_IFACE", "vcan0")
UDP_PORT, INJECT_PORT = 26700, 26701

MAGIC = b"BCAN"
HDR, REC = 8, 16
CAN_FRAME = "<IB3x8s"          # matches struct can_frame

fails = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


def can_socket(iface):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


def send_frame(s, can_id, data):
    s.send(struct.pack(CAN_FRAME, can_id, len(data), data.ljust(8, b"\0")))


def decode(pkt):
    assert pkt[:4] == MAGIC, pkt[:4]
    ver, cnt = pkt[4], pkt[5]
    out = []
    for i in range(cnt):
        rec = pkt[HDR + i * REC:HDR + (i + 1) * REC]
        cid, dlc = struct.unpack_from("<IB", rec, 0)
        out.append((cid, rec[8:8 + dlc]))
    return ver, out


def encode(frames):
    pkt = MAGIC + bytes([1, len(frames), 0, 0])
    for cid, data in frames:
        pkt += struct.pack("<IB3x8s", cid, len(data), data.ljust(8, b"\0"))
    return pkt


def start(allow_inject, status, agilex=False):
    args = [BIN, "-f", "-i", IFACE,
            "-r", f"127.0.0.1:{UDP_PORT}",
            "-l", str(INJECT_PORT),
            "-t", "251,252",
            "-S", status, "-I", "50"]
    if allow_inject:
        args.append("--allow-inject")
    if agilex:
        args.append("--agilex")
    p = subprocess.Popen(args, stderr=subprocess.PIPE, text=True)
    time.sleep(0.6)
    if p.poll() is not None:
        print("  FAIL  bridge exited immediately:", p.stderr.read())
        sys.exit(1)
    return p


def run(allow_inject):
    label = "allow-inject" if allow_inject else "read-only (default)"
    print(f"\n--- {label} ---")
    status = tempfile.mkstemp(suffix=".json")[1]

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", UDP_PORT))
    rx.settimeout(3)

    proc = start(allow_inject, status)
    bus = can_socket(IFACE)
    listener = can_socket(IFACE)
    listener.settimeout(2)

    try:
        # ---- CAN -> UDP ----
        send_frame(bus, 0x251, bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]))
        try:
            pkt, _ = rx.recvfrom(65535)
            ver, frames = decode(pkt)
            check("wire version", ver, 1)
            check("frame reached UDP", frames[0], (0x251, bytes(range(1, 9))))
        except socket.timeout:
            check("frame reached UDP", "timeout", "a datagram")

        # ---- batching: several frames should share one datagram ----
        for i in range(8):
            send_frame(bus, 0x252, bytes([i] * 4))
        time.sleep(0.3)
        total = 0
        rx.settimeout(0.5)
        try:
            while True:
                pkt, _ = rx.recvfrom(65535)
                _, frames = decode(pkt)
                total += len(frames)
        except socket.timeout:
            pass
        check("all 8 frames forwarded", total, 8)
        rx.settimeout(3)

        # ---- UDP -> CAN ----
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # drain anything already pending on the bus
        listener.settimeout(0.2)
        try:
            while True:
                listener.recv(16)
        except socket.timeout:
            pass
        listener.settimeout(1.5)

        tx.sendto(encode([(0x123, b"\xde\xad\xbe\xef")]),
                  ("127.0.0.1", INJECT_PORT))
        time.sleep(0.4)

        got_injected = None
        try:
            while True:
                raw = listener.recv(16)
                cid, dlc, data = struct.unpack(CAN_FRAME, raw)
                if cid == 0x123:
                    got_injected = data[:dlc]
                    break
        except socket.timeout:
            pass

        if allow_inject:
            check("injected frame on the bus", got_injected, b"\xde\xad\xbe\xef")
        else:
            check("injection refused", got_injected, None)

        proc.terminate()
        proc.wait(timeout=3)

        st = json.load(open(status))
        check("status interface", st["interface"], IFACE)
        check("status inject_allowed", st["inject_allowed"], allow_inject)
        check("tracked 251 decoded", st["frames"].get("251", {}).get("data"),
              "0102030405060708")
        check("tracked 252 count", st["frames"].get("252", {}).get("count"), 8)
        if not allow_inject:
            check("rejected counted", st["rejected"], 1)
            check("nothing injected", st["injected"], 0)
        else:
            check("injected counted", st["injected"], 1)
        check("rx counted", st["rx"] >= 9, True)
        # None of the frames above is a generation marker, so the bridge must
        # say it does not know rather than pick one. A default of "v2" here
        # would be the same mistake as assuming the vehicle matches agilex.c.
        check("protocol not guessed", st["agilex_protocol"], "unknown")
    finally:
        if proc.poll() is None:
            proc.kill()
        bus.close()
        listener.close()
        rx.close()
        if os.path.exists(status):
            os.remove(status)


print(f"can-bridge verification on {IFACE}")
try:
    can_socket(IFACE).close()
except OSError as e:
    print(f"cannot open {IFACE}: {e}")
    print("create it with: sudo modprobe vcan && "
          f"sudo ip link add dev {IFACE} type vcan && sudo ip link set up {IFACE}")
    sys.exit(2)

def run_protocol_detection():
    """The generation markers, exactly as ugv_sdk defines them.

    src/utilities/protocol_detector.cpp treats 0x151 as unique to v1 and
    0x221/0x241 as unique to v2, and calls it UNKNOWN when both appear. This
    matters more than it looks: agilex.c implements v2, and the vendor's own
    Scout Mini Omni demo detects at runtime instead of assuming, so a v1
    vehicle is a real possibility rather than a hypothetical one.
    """
    print("\n  protocol generation detection")
    for desc, ids, want in (
        ("0x151 alone is v1", [0x151], "v1"),
        ("0x221 alone is v2", [0x221], "v2"),
        ("0x241 alone is v2", [0x241], "v2"),
        ("both markers is unknown", [0x151, 0x221], "unknown"),
        ("an unrelated id decides nothing", [0x300], "unknown"),
    ):
        status = tempfile.mkstemp(suffix=".json")[1]
        bus = can_socket(IFACE)
        proc = start(False, status)
        try:
            for cid in ids:
                send_frame(bus, cid, bytes(8))
                time.sleep(0.25)
            time.sleep(0.3)
            proc.terminate()
            proc.wait(timeout=3)
            check(desc, json.load(open(status))["agilex_protocol"], want)
        finally:
            if proc.poll() is None:
                proc.kill()
            bus.close()
            if os.path.exists(status):
                os.remove(status)


def v1_checksum(can_id, data):
    """The vendor's CalcCanFrameChecksumV1, written out again here rather than
    imported, so a mistake in agilex.c cannot agree with itself."""
    total = (can_id & 0xFF) + ((can_id >> 8) & 0xFF) + 8
    for b in data[:7]:
        total += b
    return total & 0xFF


def run_generation_decoding():
    """Whether the decoder that runs is the one the bus asked for.

    Detection alone is not enough. The two generations reuse identifiers for
    unrelated things - 0x131 is a brake command in v2 and the motion state in
    v1 - so the daemon has to hold off decoding until it knows, and then use the
    matching decoder. Getting this wrong does not look like an error: the v2
    decoder reads a v1 motion state as a brake command and publishes velocities
    that were never sent.
    """
    print("\n  decoding follows the detected generation")

    # --- v1: 0x151 settles it, then a checksummed 0x131 must decode ---
    status = tempfile.mkstemp(suffix=".json")[1]
    bus = can_socket(IFACE)
    proc = start(False, status, agilex=True)
    try:
        sysframe = bytearray(8)
        sysframe[1] = 0x01			# control mode
        sysframe[2], sysframe[3] = 0x00, 0xF6	# 246 -> 24.6 V
        sysframe[7] = v1_checksum(0x151, sysframe)
        send_frame(bus, 0x151, bytes(sysframe))
        time.sleep(0.3)

        mot = bytearray(8)
        mot[0], mot[1] = 0x03, 0xE8		# +1000 -> 1.000 m/s
        mot[2], mot[3] = 0xFF, 0x9C		#  -100 -> -0.100 rad/s
        mot[4], mot[5] = 0x00, 0xC8		#  +200 -> 0.200 m/s sideways
        mot[7] = v1_checksum(0x131, mot)
        send_frame(bus, 0x131, bytes(mot))
        time.sleep(0.4)
        proc.terminate()
        proc.wait(timeout=3)
        st = json.load(open(status))
        a = st.get("agilex", {})
        check("v1 detected from the bus", st["agilex_protocol"], "v1")
        check("v1 battery decoded", a.get("battery_v"), 24.6)
        check("v1 linear decoded", a.get("linear_mps"), 1.0)
        check("v1 lateral decoded", a.get("lateral_mps"), 0.2)
        # 0x131 under the v2 decoder is a brake command with no velocities, so
        # a linear_mps at all is the proof that the v1 decoder ran.
        check("v1 sideways motion identifies the mecanum base",
              a.get("variant", "").startswith("omni"), True)
    finally:
        if proc.poll() is None:
            proc.kill()
        bus.close()
        if os.path.exists(status):
            os.remove(status)

    # --- nothing is decoded before the generation is known ---
    #
    # 0x131 arrives on both generations, so it is exactly the frame that must
    # not be decoded on a guess. It should be counted as undecided instead.
    status = tempfile.mkstemp(suffix=".json")[1]
    bus = can_socket(IFACE)
    proc = start(False, status, agilex=True)
    try:
        for _ in range(4):
            send_frame(bus, 0x131, bytes(8))
            time.sleep(0.15)
        time.sleep(0.4)
        proc.terminate()
        proc.wait(timeout=3)
        st = json.load(open(status))
        check("an ambiguous id decides nothing", st["agilex_protocol"], "unknown")
        check("and is counted as undecided rather than decoded",
              st.get("agilex_undecided", 0) > 0, True)
        check("nothing was decoded", st.get("agilex", {}).get("decoded"), 0)
    finally:
        if proc.poll() is None:
            proc.kill()
        bus.close()
        if os.path.exists(status):
            os.remove(status)


run(False)
run(True)
run_protocol_detection()
run_generation_decoding()

print()
if fails:
    print(f"FAILED ({len(fails)}): {', '.join(fails)}")
    sys.exit(1)
print("all checks passed")
