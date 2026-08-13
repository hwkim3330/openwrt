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

Not verified, because it needs the hardware: a real USB-CAN adapter enumerating
on the router, `gs_usb` binding on mipsel, and anything about the vehicle.
