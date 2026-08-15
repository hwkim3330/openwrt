# Bench runbook — the CAN session

One page, in order, with the decision at each step. `CAN.md` is the same ground
with the reasoning attached; this is the version to work from with the vehicle in
front of you.

The previous edition of this page was the first-flash runbook. That session
happened on 2026-08-13 and its results are in `VERIFY.md`; the procedure itself
lives in `BRINGUP.md`, which is where to look if a board ever needs flashing from
scratch again.

Budget about an hour. Everything that does not need the vehicle is done: both
protocol generations are implemented and tested, the command path refuses to
guess between them, and the whole v1 chain has run on emulated mipsel. This
session answers one question the bench cannot — **which generation this vehicle
speaks** — and then, only if that goes well, one more: **which way is positive
lateral**.

## Have ready

- the router, and `openwrt-ramips-mt7621-iptime_a3004ns-m-squashfs-sysupgrade.bin`
- the PEAK PCAN-USB adapter and the vehicle's 4-pin CAN connector
- the SCOUT MINI Omni, **on a stand with the wheels clear of the ground**
- its remote, powered on, so there is a way to stop it that does not involve
  this software

Do not flash the vehicle. `SCOUT-FIRMWARE.md` covers what was found: there is no
SCOUT MINI firmware image, and the two methods that circulate for getting one
were each checked and neither works.

## 0 · Flash the router first — 10 min

The running firmware predates all of this. Without it the router has no v1
support, `agx-cmd` is on the old port, and the ring default is wrong.

```sh
package/keti/tools/flash-router            # checks the image, then explains
```

`-n` is required on mt7621: the board.d catch-all is compat version 1.1 and the
image defaults to 1.0, so sysupgrade refuses to keep settings. The tool says so
rather than leaving it to be discovered.

## 1 · Bring the bus up, read-only — 5 min

```sh
uci set can-bridge.bus.enabled='1'
uci set can-bridge.bus.interface='can0'
uci set can-bridge.bus.discover='1'
uci set can-bridge.bus.agilex='1'
uci commit can-bridge && /etc/init.d/can-bridge restart
logread -e can-bridge
```

`allow_inject` stays at 0. Nothing in this step can move anything: the daemon has
no write path until that flag is set, and refusals are counted.

The adapter needs `kmod-can-usb-peak` — a PCAN-USB Pro FD is not `gs_usb`. If
`can0` does not appear, that is the first thing to check.

## 2 · The one question that matters — 2 min

```sh
sed -n 's/.*"agilex_protocol": "\([a-z0-9]*\)".*/\1/p' /var/run/can-bridge.json
```

- **`v1`** → percentages of 3.0 m/s, checksummed, rolling counter. `agx-cmd`
  picks this up on its own.
- **`v2`** → mm/s, no checksum. Likewise.
- **`unknown` with `rx` climbing** → neither discriminator is arriving. Look at
  the ids in the log before going further; the table in `CAN.md` is v2's, and if
  nothing matches either set this is not an AgileX vehicle at the ids we expect.
- **`unknown` and `rx` at 0** → the bus is not wired or not terminated. See the
  termination note in `CAN.md`; it is the thing most likely to bite.

Write the answer down in `CAN.md` where it says the generation has not been
observed yet. That sentence is the last thing in this tree that is guessing.

## 3 · Does the decoder agree with the bus — 3 min

```sh
grep -E '"decoded"|"undecoded"|agilex_undecided' /var/run/can-bridge.json
```

`decoded` climbing and `undecoded` flat means the generation is right.
`undecoded` climbing means it is not, whatever step 2 said. `agilex_undecided`
should stop rising the moment step 2 answered.

Battery volts and a plausible motion state in the same file are the confirmation
that the fields are being read at the right offsets and the right endianness.

## 4 · Wheels clear, then command — 20 min

Only now, and only with the wheels off the ground.

```sh
uci set can-bridge.bus.allow_inject='1'
uci commit can-bridge && /etc/init.d/can-bridge restart

uci set agx-cmd.cmd.enabled='1'
uci commit agx-cmd && /etc/init.d/agx-cmd restart
logread -e agx-cmd            # says which generation it settled on
```

Two switches, both off by default, neither implying the other. If `agx-cmd`
reports `protocol_skips` rising and sends nothing, that is the safeguard working:
step 2 has not answered.

Test in this order, because each makes the next safe:

1. **Deadman.** Arm, command forward, then put the tablet down or kill the app.
   The wheels must stop. `deadman_stops` in `/var/run/agx-cmd.json` increments.
2. **Forward.** A small forward command should turn the wheels forward. If it
   goes backwards, the linear sign is wrong and nothing else should be tried.
3. **Lateral.** Command strafe right. The one convention no document settles.
   If the vehicle would go left, set `option lateral_invert '1'` rather than
   editing `agilex.c` — that flag exists for exactly this.

Commands go out at 50 Hz, which is what `ugv_sdk` states the vehicle wants. On v1
the resolution is 30 mm/s, so slow crawls come out as small integers; that is the
protocol, not a bug.

## What to leave alone

- The vehicle's firmware.
- `max_linear` and friends. They are walking pace on purpose. Raise them
  deliberately, after the three tests above, and not while the vehicle is on the
  floor.

## If something is wrong

```sh
uci set can-bridge.bus.allow_inject='0'
uci set agx-cmd.cmd.enabled='0'
uci commit && /etc/init.d/can-bridge restart; /etc/init.d/agx-cmd stop
```

Either switch alone is enough to make the router harmless again.

## After it works

`navigate` already emits TELE to `agx-cmd`, so a destination on the tablet map
becomes motion the moment this session succeeds. Do that on a stand first too:
`navigate --dry-run` plans and reports without commanding.
