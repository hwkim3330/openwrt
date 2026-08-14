# CAN on the A3004NS-M

## Short answer

The SoC has no CAN controller. A USB-CAN adapter works, the driver is in
OpenWrt (`kmod-can-usb-gs`, the `gs_usb` / candleLight family that AgileX's own
documentation uses), and `can-bridge` puts the bus on the network. The catch is
not software: **the router has one USB port**, and on this build the camera and
its microphone are already on it.

## What is and is not possible

| approach | verdict |
|---|---|
| MT7621 native CAN | **no such peripheral.** The `kmod-can-c-can`, `-flexcan` platform drivers have nothing to bind to |
| USB-CAN on `gs_usb` | **works** — `kmod-can-usb-gs` is in the tree and builds for mipsel_24kc |
| USB-CAN on `slcan` | works too (`kmod-can-slcan`), needs `slcand`, which is not packaged |
| MCP2515 over SPI | electrically possible; `spi0` already carries the NOR flash and there is no header, so it means soldering to a second chip-select. Not recommended |
| bit-banging CAN on GPIO | no. CAN arbitration is bit-timed in hardware |

## The USB port problem

One USB 3.0 port, and a Logitech StreamCam presents as *two* devices on it (UVC
video plus USB audio). Adding CAN needs a hub.

Use a **USB 3.0** hub, not 2.0: the camera negotiated SuperSpeed (measured at
5000 Mbit/s on the bench), and 1080p60 MJPEG is around 83 Mbit/s of bulk
traffic. A CAN dongle is Full Speed and rides the USB 2.0 lanes of the same hub
without competing for it. A 2.0-only hub would drag the camera down to High
Speed and share that bandwidth.

## Wiring, for an AgileX SCOUT MINI Omni

Only two of the four pins are the bus. From the vehicle's own manual, with the
key oriented as the manual shows: pin 1 = +23–29 V, pin 2 = GND, pin 3 = CAN_H,
pin 4 = CAN_L, and on AgileX's cable **yellow = CAN_H, blue = CAN_L**, red/black
= power. Follow the colours, not the pin numbers, when making a cable — the
numbering mirrors when you look at the male shell from the solder side.

The 24 V is *power for accessories*. It is not the control interface. Drive
commands are CAN frames at 500 kbps, CAN 2.0B.

```
SCOUT MINI 4-pin
  red    +24V ──────► accessory power (lidar, etc.)
  black  GND  ──────►
  yellow CAN_H ──┬── USB-CAN ── USB ── router (or Jetson)
  blue   CAN_L ──┘
```

### Termination is the thing that will bite you

CAN wants exactly 120 Ω at each of the two ends of the bus, and nothing in
between. If the vehicle terminates one end and a Jetson's adapter terminates the
other, adding the router as a third node means:

- **do not** enable termination on the router's adapter (most have a jumper or
  solder bridge — check it), and
- keep the stub to the router short.

Getting this wrong produces intermittent, load-dependent errors that look like
software flakiness. `ip -details -statistics link show can0` reports bus errors;
`can-up` prints that automatically.

## Bring it up

```sh
opkg install can-bridge          # pulls kmod-can, kmod-can-raw, kmod-can-usb-gs
can-up can0 500000               # or let the init script do it
```

`can-up` prints what to check when the interface does not appear — enumeration,
driver binding, and the single-USB-port caveat.

```sh
uci set can-bridge.bus.enabled='1'
uci set can-bridge.bus.discover='1'      # find out what the vehicle sends
uci commit can-bridge
/etc/init.d/can-bridge start
logread -e can-bridge
```

`discover` exists because `can-utils` is **not** in the OpenWrt feeds, so there
is no `candump` on the device. It logs each distinct id the first time it is
seen. Once you know the ids, put them in `option track` and they appear decoded
in the dashboard's CAN panel and in `/var/run/can-bridge.json`.

## Decoding an AgileX vehicle

`option agilex '1'` turns raw hex into named values. The ids, field layouts and
scale factors come from AgileX's own SDK - `src/protocol_v2/agilex_protocol_v2.h`
for the layouts and `agilex_msg_parser_v2.c` for the scaling, in
[ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) - not from a forum post.
That distinction matters: the notes circulating for this vehicle say `0x251` is
motion feedback, and it is not. Motion state is **`0x221`**; `0x251` is the first
actuator's high-speed state. Both carry plausible small numbers, so the mix-up
survives a glance at a dashboard.

These are the **protocol v2** ids. v1 uses a different set; see below for how the
daemon decides which it is looking at.

| id | meaning |
|---|---|
| `0x211` | system state: vehicle state, control mode, battery (0.1 V), error bitmap |
| `0x221` | motion state: linear, angular, lateral, steering (mm/s, mrad/s) |
| `0x241` | RC state |
| `0x251`–`0x258` | actuator high-speed: rpm, current (0.1 A), pulse count |
| `0x261`–`0x268` | actuator low-speed: driver volts, driver temp, motor temp, driver state |
| `0x291` | motion mode state |
| `0x311` | odometry: left and right wheel, 32-bit signed |
| `0x361` | BMS: SoC, SoH, volts, amps, temperature (0.1 units) |
| `0x111`/`0x121`/`0x131`/`0x141` | commands. Recognised for naming; never emitted |

**The payload is big-endian.** The SDK's `struct16_t` is `{high_byte, low_byte}`,
so every 16-bit field is most-significant byte first. Everything else in this tree
is little-endian, which makes this the obvious place to get it wrong, and a
wrong-endian read of 1.0 m/s is −6.144 m/s rather than something obviously absurd.

Decoding is **off by default**. On a bus that is not an AgileX vehicle these ids
mean something else, and named fields would be confident nonsense.

### Skid or Omni

The daemon does not need telling. Both share the same feedback frames -
`MotionStateFrame` always carries a lateral field - and differ only in that the
Omni can be *commanded* sideways. So a non-zero lateral velocity, or a motion-mode
frame at all, means mecanum wheels, and the status file reports what it observed:

```json
"agilex": { "variant": "omni (mecanum: lateral motion observed)", ... }
```

Before enough frames it says `unknown (no motion state yet)`, which is honest
rather than a guess.

The vehicle on this bench is the **Omni** — mecanum wheels, so it translates
sideways and diagonally. That is why the teleop path carries three axes rather
than two: see `TELEOP.md`. The inference stays in anyway, because it is the check
that the bus agrees with that assumption.

### Which generation - and why it is not a detail

The Scout Mini Omni ships in **two protocol generations**, and `ugv_sdk` itself
decides at runtime (`src/utilities/protocol_detector.cpp`) rather than assuming.
They are not dialects of one protocol:

| | v2 | v1 |
|---|---|---|
| motion command | `0x111` | `0x130` |
| units | mm/s, mrad/s as big-endian int16 | signed **percentage** of the vehicle's maximum |
| integrity | none | checksum byte the vehicle enforces |
| sequencing | none | rolling `count` byte |
| resolution | 1 mm/s | 1% of 3.0 m/s = **30 mm/s** |
| motion state | `0x221` | `0x131` |
| system state | `0x211` | `0x151` |

Note that `0x131` is a *brake command* in v2 and the *motion state* in v1. That
overlap is why there are two decoders (`agx_decode` and `agx1_decode`) and why
`can-bridge` decodes nothing until it knows which one applies: on a v1 bus the v2
decoder reads the motion state as a brake command and reports velocities nobody
sent, which looks like data rather than like an error. The frames that go by
before the answer arrives are counted as `agilex_undecided`; both discriminators
are periodic at 50 Hz, so the wait is tens of milliseconds.

Sending the wrong generation is worse than sending nothing. A v2 command for a
0.25 m/s crawl is `00 FA 00 00 …`; a v1 vehicle reads byte 2 as `linear_percentage`
= 0 and byte 3 as `angular_percentage` = 250, clamps it, and yaws at its maximum
rate. So `agx-cmd` defaults to `option protocol 'auto'`, reads the answer out of
`can-bridge`'s status file, and **withholds every command until it has one** -
counted as `protocol_skips`, and logged once at startup so an armed stick that
moves nothing has a visible reason.

The v1 percentages are fractions of the vehicle's own maxima, not of `max_linear`.
These are the exact divisors `ProtocolV1Parser<ScoutMiniLimits>` uses, and they
are the MINI's, not the plain SCOUT's (which is 1.5 m/s, 0.5235 rad/s, no lateral):

```
AGX1_MINI_MAX_LINEAR   3.0    m/s
AGX1_MINI_MAX_ANGULAR  2.5235 rad/s
AGX1_MINI_MAX_LATERAL  2.0    m/s
```

To go slower, clamp the intent before encoding - which is what `max_linear` does.
Dividing by `max_linear` instead would turn walking pace into 100%.

One consequence worth knowing before the first drive: on v1 the command
resolution is 30 mm/s, so walking-pace commands are small integers (0.5 m/s is
17%) and a fine correction below 30 mm/s rounds to zero. v2 has 1 mm/s.

**Which one this vehicle is has not been observed yet.** Both paths are
implemented and tested, so it costs nothing to find out - see the bring-up order
below.

### Still read your own ids

`option discover '1'` first, regardless. Firmware versions differ, and if
`undecoded` climbs while `decoded` stays flat, this is not a protocol-v2 vehicle
and the table above does not apply. The raw bytes are always reported alongside
the decoded fields, so nothing is lost either way.

## Injection is off by default, on purpose

`can-bridge` forwards bus → network freely. Network → bus is refused unless
`option allow_inject '1'` is set, and the refusals are counted in the status
file so you can see it happening.

This is not paranoia about the code, it is about where the code sits. A drive
command that crosses a WiFi link and a router's scheduler has:

- **no bound on latency.** The lidar path measured under a millisecond on a
  desktop, but WiFi retries are unbounded and airtime is shared with the camera
  stream on the same radio.
- **no failsafe.** CAN drive commands are level-triggered: the last velocity
  keeps being acted on. If the link stalls mid-command the vehicle keeps going.
  A real control loop sends a heartbeat and stops on timeout; that logic belongs
  on the machine holding the loop, not behind a bridge.

So the sane split is the one AgileX documents:

```
Jetson/PC ── USB-CAN ── CAN_H/L ── vehicle        control loop, direct, no hops
router    ── USB-CAN ── same bus (unterminated)   telemetry, read-only
```

The router reads the bus and shows it to a tablet. If you do want the router to
command something, use `--allow-inject` on a bench, with the wheels off the
ground, and add your own heartbeat.

## What has been verified

`can-bridge` is tested against real SocketCAN interfaces (`vcan`) in
`../can-bridge/test/test_bridge.py`:

- frames reach the UDP peer with ids and payloads intact
- multiple frames batch into one datagram
- injection is refused by default and counted as `rejected`
- injection works once `--allow-inject` is given
- tracked ids decode into the status file with counts and ages

The protocol v1 encoder and decoder are tested in
`../can-bridge/test/test_agilex.c` against values computed by hand from
`agilex_msg_parser_v1.c` - the checksum formula, percentage saturation at ±100,
a zero maximum commanding zero rather than dividing, and a corrupt frame being
refused but counted.

The generation choice is tested end to end in `../agx-cmd/test/test_protocol.py`,
which runs the daemon and reads the frames it emits:

- with nothing known, every command is withheld and counted, while teleop input
  is still being read
- a conflicted bus (`"unknown"`) and an unparseable status file both withhold too
- v1 emits `0x130` with a percentage of 3.0 m/s, a rolling counter and a valid
  checksum on every frame
- v2 emits `0x111` with mm/s and no checksum byte
- `--protocol v1` commands with no detector at all, for a bench
- a status file that changes generation mid-run does not change what is sent

On emulated mipsel (`emu/run-emu.py`, vcan on the guest's own kernel) the whole
v1 command chain runs end to end - TELE into `agx-cmd`, its inject datagram into
`can-bridge`, and the frame back off `vcan0` where a second instance reads it:

```
0x130 on the bus: 01 00 11 00 00 00 2C 77   (45 frames)
                  ^  ^  ^              ^  ^
                  |  |  17% linear     |  checksum, computed on soft-float
                  |  clear no errors   rolling count
                  CAN control mode
```

17%, not 100%: full stick is `--max-linear 0.5` and v1 divides by the vehicle's
3.0 m/s. The same run confirms `0x241` alone reads as v2, `0x151` alone as v1,
both together going back to `unknown`, and `agx-cmd` picking the generation out
of `can-bridge`'s status file rather than being told.

Not verified, because it needs the hardware: a real USB-CAN adapter enumerating
on the router, `peak_usb` binding on mipsel, **which generation this vehicle
speaks**, and the sign of the lateral axis.

### The first minute on the real bus

In this order, because each step makes the next one safe:

1. `can-bridge --interface can0 --discover` with no `--allow-inject`. Read-only;
   it cannot move anything.
2. Look at `agilex_protocol` in `/var/run/can-bridge.json`. `v1` or `v2` answers
   the open question. `unknown` with `rx` climbing means neither discriminator is
   arriving - check the ids in the log before going further.
3. Check `decoded` climbing and `undecoded` flat. If `agilex_undecided` keeps
   rising, step 2 never settled.
4. Only then, wheels off the ground, `--allow-inject` plus `agx-cmd` enabled, and
   confirm the lateral sign - it is the one thing no document settles.
