#!/usr/bin/env python3
"""Drive an AgileX chassis over its RS232 port.

    agxserial.py watch                       read only, decode, print
    agxserial.py enable                      put the chassis in serial mode
    agxserial.py standby                     put it back
    agxserial.py drive --mm-s 0 --mrad-s 0   hold a velocity at 20 Hz
    agxserial.py clear                       clear latched errors

This exists because CAN went silent on this vehicle and stayed silent through every
wiring configuration, while the serial port was framing perfectly. Serial is a
documented control path, not a workaround: the protocol is in the SCOUT 2.0 manual,
section 3.4.

    5A A5 | len | type | id | data | frame id | checksum

    len       counts from itself to the frame id - so 6 data bytes gives 0x0A
    type      0x55 control, 0xAA feedback
    frame id  rolls 0-255, one per frame sent
    checksum  sum of every byte before it, & 0xFF

That last line was worked out from a capture before the manual was found - 6637 of
6637 frames agreed - and the manual then said the same thing. Worth knowing that the
near-miss rules scored 30 and 37 out of 6637: a wrong checksum rule still hits
sometimes, so "it matches a few" means nothing.

**On safety.** The chassis takes a velocity and holds it, and stops on its own only
when 500 ms pass with no command. So this sends continuously while driving and sends
a zero on the way out, and `drive` will not start unless the chassis is already in
serial mode - `enable` is deliberately a separate step you have to have taken.

**On this being the SCOUT 2.0 manual.** The vehicle here is a SCOUT MINI OMNI, which
has a lateral axis the 2.0 does not, and the 2.0 control frame marks bytes 4 and 5
"reserved". They are plausibly lateral speed on an OMNI. That is a guess and it is
not made here: those bytes are sent as zero.
"""
import argparse
import struct
import sys
import time

SYNC = b"\x5a\xa5"
CONTROL, FEEDBACK = 0x55, 0xAA


def frame(cmd_type, cmd_id, data, fid):
    """One message. `len` counts the length byte through the frame id."""
    body = bytes([1 + 1 + 1 + len(data) + 1, cmd_type, cmd_id]) + bytes(data) + bytes([fid & 0xFF])
    out = SYNC + body
    return out + bytes([sum(out) & 0xFF])


def decode(f):
    """Feedback frames, from the manual's tables plus what was observed."""
    if len(f) < 8 or f[3] != FEEDBACK:
        return None
    d = f[5:-2]
    q = f[4]
    s16 = lambda o: struct.unpack(">h", d[o:o + 2])[0]
    u16 = lambda o: struct.unpack(">H", d[o:o + 2])[0]
    if q == 0x01 and len(d) >= 6:
        # byte[0] vehicle state, byte[1] mode, byte[2:4] battery in 0.1 V
        return {"id": "system", "state": d[0], "mode": d[1],
                "volts": round(u16(2) / 10.0, 1)}
    if q == 0x02 and len(d) >= 6:
        return {"id": "motion", "mm_s": s16(0), "mrad_s": s16(2),
                "lateral": s16(4)}
    if q in (0x03, 0x04, 0x05, 0x06) and len(d) >= 6:
        return {"id": f"actuator{q - 2}", "rpm": s16(0), "temp_c": d[4]}
    return {"id": f"0x{q:02x}", "raw": d.hex(" ")}


class Link:
    def __init__(self, dev, baud=115200):
        import serial
        self.s = serial.Serial(dev, baud, timeout=0.05)
        self.fid = 0
        self.buf = bytearray()

    def send(self, cmd_id, data):
        self.fid = (self.fid + 1) & 0xFF
        f = frame(CONTROL, cmd_id, data, self.fid)
        self.s.write(f)
        return f

    def read(self):
        """Whatever complete frames have arrived."""
        try:
            self.buf += self.s.read(512)
        except Exception:
            return []
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 4:
                break
            total = self.buf[2] + 3
            if len(self.buf) < total:
                break
            out.append(bytes(self.buf[:total]))
            del self.buf[:total]
        return out

    def state(self, timeout=2.0):
        """The next system-status frame, so a mode change can be confirmed."""
        end = time.time() + timeout
        while time.time() < end:
            for f in self.read():
                d = decode(f)
                if d and d["id"] == "system":
                    return d
            time.sleep(0.02)
        return None


MODE_NAMES = {0x00: "standby", 0x01: "CAN", 0x02: "serial", 0x03: "remote"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("watch", "enable", "standby", "drive", "clear"))
    ap.add_argument("--dev", default="/dev/ttyUSB0")
    ap.add_argument("--mm-s", type=int, default=0, help="forward, mm/s")
    ap.add_argument("--mrad-s", type=int, default=0, help="yaw, 0.001 rad/s")
    ap.add_argument("--seconds", type=float, default=3.0)
    # The mode gate turned out to be unverifiable on this vehicle: byte[1] of the
    # system frame never moved even while the remote was plainly driving, so it is
    # not the control mode here whatever the SCOUT 2.0 table says. That makes
    # "refuse unless mode is serial" a gate on a reading that means nothing, and the
    # only way left to learn whether the chassis accepts commands is to send one.
    ap.add_argument("--force", action="store_true",
                    help="drive without confirming serial mode - the vehicle may move")
    a = ap.parse_args()

    link = Link(a.dev)

    if a.cmd == "watch":
        end = time.time() + a.seconds
        seen = {}
        while time.time() < end:
            for f in link.read():
                d = decode(f)
                if d:
                    seen[d["id"]] = d
            time.sleep(0.05)
        for k, v in sorted(seen.items()):
            if k == "system":
                v = dict(v, mode_name=MODE_NAMES.get(v["mode"], "?"))
            print(f"  {k:10s} {v}")
        return 0

    before = link.state()
    print(f"  before: {before}  mode={MODE_NAMES.get((before or {}).get('mode'), '?')}")

    if a.cmd in ("enable", "standby"):
        want = 0x02 if a.cmd == "enable" else 0x00
        # Sent more than once: this is a single datagram on a line with no
        # acknowledgement, and one lost byte would leave the mode unchanged with
        # nothing to say so.
        for _ in range(5):
            link.send(0x02, [want])
            time.sleep(0.05)
        after = link.state()
        print(f"  after:  {after}  mode={MODE_NAMES.get((after or {}).get('mode'), '?')}")
        ok = after and after["mode"] == want
        print(f"  {'모드 변경됨' if ok else '모드 안 바뀜 - 섀시가 명령을 받지 않았거나 거부'}")
        return 0 if ok else 1

    if a.cmd == "clear":
        for _ in range(3):
            link.send(0x03, [0x00])
            time.sleep(0.05)
        print("  error clear sent")
        return 0

    if a.cmd == "drive":
        # Refuses unless the chassis is already in serial mode. Enabling implicitly
        # would mean one command both arming and moving, which is the shape of
        # interface that gets a vehicle away from someone.
        if not a.force and (not before or before["mode"] != 0x02):
            print("  섀시가 시리얼 모드가 아닙니다. `enable` 후 재시도하거나 --force.")
            return 1
        data = list(struct.pack(">hh", a.mm_s, a.mrad_s)) + [0, 0]
        print(f"  {a.seconds}s 동안 {a.mm_s} mm/s, {a.mrad_s} mrad/s 유지 (20 Hz)")
        end = time.time() + a.seconds
        last = 0.0
        try:
            while time.time() < end:
                link.send(0x01, data)
                time.sleep(0.02)
                if time.time() - last > 0.5:
                    last = time.time()
                    for f in link.read():
                        d = decode(f)
                        if d and d["id"] == "motion":
                            print(f"    실측 {d['mm_s']:+5d} mm/s  {d['mrad_s']:+5d} mrad/s")
        finally:
            # Zero on the way out rather than relying on the 500 ms timeout: the
            # timeout is the backstop, not the plan.
            for _ in range(5):
                link.send(0x01, [0, 0, 0, 0, 0, 0])
                time.sleep(0.02)
            print("  정지 명령 전송")
        return 0


if __name__ == "__main__":
    sys.exit(main())
