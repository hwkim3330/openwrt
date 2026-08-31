#!/usr/bin/env python3
"""Ask a CANable 2.0 whether its CAN controller is alive - with nothing else on the bus.

    selftest.py [--dev /dev/ttyACM0] [--bitrate S6]

CAN needs a second node to acknowledge a frame, so the obvious self-test - send
something and see if it goes - cannot be run by one adapter. That was the conclusion
here for a while, and it was wrong. The adapter will tell you, indirectly, through
its error register.

The firmware keeps a latched bitmask (normaldotcom/canable2-fw, inc/error.h) and
returns it for the non-standard `E` command:

    0  ERR_PERIPHINIT          FDCAN would not initialise
    1  ERR_USBTX_BUSY
    2  ERR_CAN_TXFAIL          HAL refused a message the firmware offered it
    3  ERR_CANRXFIFO_OVERFLOW
    4  ERR_FULLBUF_CANTX       the firmware's own 64-frame queue overflowed
    5  ERR_FULLBUF_USBRX
    6  ERR_FULLBUF_USBTX

Bit 4 is the one that decides it, and the reasoning is worth writing down because
the bit sounds like a complaint rather than a result.

`can_process()` only hands a frame to the peripheral while
`HAL_FDCAN_GetTxFifoFreeLevel() > 0`, and it advances its own tail either way. So:

  - If FDCAN never started, the hardware FIFO reads as empty forever, every frame is
    offered, every offer fails, and you get ERR_CAN_TXFAIL with the software queue
    draining normally. Bit 4 stays clear.
  - If FDCAN started and transmissions complete, the FIFO drains and the software
    queue never builds up. Bit 4 stays clear.
  - If FDCAN started, accepted frames, and is stuck retransmitting because nothing
    on the bus acknowledges them, the hardware FIFO stays full, the software queue
    backs up behind it, and bit 4 sets.

So **bit 4 setting is the pass condition** for a lone adapter. It means the
peripheral is initialised, took the frames, and is retrying them - everything up to
the transceiver is working. Measured on this adapter: 0x04 before, 0x14 after 200
frames at 1 kHz.

What this does not cover is the transceiver's differential output and the wiring
past it. Nothing short of a second node or a meter covers that.

**The same run is a vehicle test.** Connect the adapter to a chassis that is powered
and transmitting, and its CAN controller will acknowledge our frames, so the FIFO
drains and bit 4 stays clear. Bit 4 setting with the vehicle attached says the
vehicle is not acknowledging - which separates "our side is dead" from "the vehicle
is not on the bus" without needing to receive a single frame from it. That is the
question this bench has been stuck on.

Two cautions. The register is latched and never cleared, so a bit that is already
set when the run starts says only "at some point since power-on"; unplug and replug
the adapter for a clean baseline. And this drives the TX path hard on purpose - it
is safe on an empty bus, and on a live bus it puts a few hundred frames of ID 0x123
onto it, so do not run it against a vehicle that is free to move.
"""
import argparse
import sys
import time

ERR_NAMES = [
    "PERIPHINIT", "USBTX_BUSY", "CAN_TXFAIL", "CANRXFIFO_OVERFLOW",
    "FULLBUF_CANTX", "FULLBUF_USBRX", "FULLBUF_USBTX",
]
BIT_FULLBUF_CANTX = 4
BIT_PERIPHINIT = 0
TEST_ID = "123"


def bits(v):
    return [ERR_NAMES[i] for i in range(len(ERR_NAMES)) if v >> i & 1]


class Adapter:
    def __init__(self, dev, baud=115200):
        import serial
        self.s = serial.Serial(dev, baud, timeout=0.8)
        time.sleep(0.3)

    def cmd(self, c, wait=0.35):
        self.s.reset_input_buffer()
        self.s.write(c)
        time.sleep(wait)
        return self.s.read(128)

    def version(self):
        r = self.cmd(b"V\r")
        return r.decode(errors="replace").strip() or None

    def errors(self):
        """The latched error register, or None if the adapter did not answer.

        Only `V` and `E` produce a reply on this firmware - `O`, `C`, `S6` and a
        frame write all return nothing at all, which is not a fault and is why an
        earlier version of this file concluded the adapter was mute.
        """
        r = self.cmd(b"E\r")
        try:
            return int(r.decode().split(":")[1].strip(), 16)
        except Exception:
            return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttyACM0")
    ap.add_argument("--bitrate", default="S6", help="S6 = 500k, matches the SCOUT")
    ap.add_argument("--frames", type=int, default=200)
    a = ap.parse_args()

    try:
        d = Adapter(a.dev)
    except Exception as e:
        print(f"  {a.dev} 열기 실패: {type(e).__name__}: {e}")
        return 2

    v = d.version()
    print(f"  어댑터 : {v or '응답 없음'}")
    if not v:
        print("  V 에 응답이 없습니다. slcand 가 포트를 잡고 있는지 확인하세요.")
        return 2

    before = d.errors()
    if before is None:
        print("  E 에 응답이 없습니다 - 이 펌웨어가 아닐 수 있습니다.")
        return 2
    print(f"  시작   : 0x{before:02X}  {bits(before) or ['없음']}")
    if before >> BIT_FULLBUF_CANTX & 1:
        print("  주의   : FULLBUF_CANTX 가 이미 서 있습니다(래치). 뽑았다 꽂고 다시 하세요.")

    # Auto-retransmission on is what makes the FIFO stay full when nothing answers.
    # With A0 the controller gives up after one attempt and the queue drains, and the
    # test says nothing.
    d.cmd(b"C\r")
    d.cmd(a.bitrate.encode() + b"\r")
    d.cmd(b"A1\r")
    d.cmd(b"M0\r")
    d.cmd(b"O\r")

    print(f"  {a.frames} 프레임 송신 (ID 0x{TEST_ID}, 1 ms 간격)")
    for _ in range(a.frames):
        d.s.write(f"t{TEST_ID}8DEADBEEF00112233\r".encode())
        time.sleep(0.001)
    time.sleep(1.0)

    after = d.errors()
    d.cmd(b"C\r")
    print(f"  끝     : 0x{after:02X}  {bits(after) or ['없음']}")

    if after is None:
        print("  판정   : 알 수 없음 - 송신 후 E 에 응답이 없습니다")
        return 2
    if after >> BIT_PERIPHINIT & 1:
        print("  판정   : 불량 - FDCAN 초기화 실패(PERIPHINIT)")
        return 1
    if after >> BIT_FULLBUF_CANTX & 1:
        print("  판정   : 어댑터 정상 - 페리페럴이 프레임을 받아 재전송 중이고,")
        print("           아무도 ACK 하지 않아 큐가 찼습니다. 상대 노드가 없을 때의")
        print("           정상 모습입니다. (차량에 물린 상태였다면: 차량이 ACK 안 함)")
        return 0
    print("  판정   : 큐가 빠졌습니다 - 송신이 완료되고 있다는 뜻이므로")
    print("           누군가 ACK 하고 있습니다. 차량에 물려 있다면 버스가 살아 있는 것이고,")
    print("           아무것도 안 물려 있다면 예상 밖이니 배선을 의심하세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
