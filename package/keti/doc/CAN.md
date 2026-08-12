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

## Wiring, for an AgileX SCOUT MINI

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

Do not copy ids from a forum post. They differ between AgileX firmware versions,
and a wrong id silently decodes the wrong bytes. Read them off your own vehicle.

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
