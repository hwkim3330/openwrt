# Bench runbook

One page, in order, with the decision at each step. `BRINGUP.md` is the same
ground with the reasoning attached; this is the version to work from with the
board in front of you.

Budget about an hour. Everything that did not need the board is already done and
verified, so this session is only about four things: does it flash, does DBDC
come up, does the lidar keep up, and what does the EEPROM say.

## Have ready

- the A3004NS-M, and its stock firmware image (to go back)
- an ethernet cable from the PC to a **LAN** port
- `openwrt-ramips-mt7621-iptime_a3004ns-m-initramfs-kernel.bin`
- `openwrt-ramips-mt7621-iptime_a3004ns-m-squashfs-sysupgrade.bin`
- the StreamCam with a USB-C-male → USB-A-male cable
- the OS-64 and its power
- optional, and only if a USB 3.0 hub is on hand: the USB-CAN adapter

A 3.3 V USB-UART on J4 at 57600 is worth wiring **before** flashing rather than
after. If the board comes up headless you will want to know why, and the pinout
is `[3V3] (TXD) (RXD) (GND)` — leave 3V3 unconnected.

## 1 · Flash the initramfs — 10 min

Stock web UI → firmware upgrade → the **initramfs** image. This runs from RAM;
a power cycle returns you to stock, which is why it goes first.

```sh
ping 192.168.1.1          # from the PC, LAN port
ssh root@192.168.1.1
```

- **it boots** → step 2.
- **it refuses the file** → the u-boot image-name check rejected it.
  `UIMAGE_NAME := a3004nm` came from the original submission and was never
  verified here. Power cycle back to stock and say so; it is a one-line fix once
  the real string is known, and the serial console prints it.
- **nothing on the network** → serial console. Do not reflash blind.

## 2 · The one question that matters — 2 min

```sh
ls /sys/class/ieee80211/
```

**Two phys is the whole point of this exercise.** The port was closed in 2023
because only one came up.

- **phy0 and phy1** → the fix works. This is the result that unblocks the
  upstream PRs.
- **phy0 only** → the override did not take. `doc/DBDC.md` has the checks. Try
  `echo 1 > /sys/kernel/debug/ieee80211/phy0/mt76/dbdc`; if a second phy appears
  that way, the hardware is fine and the DT path is what is wrong, which is a
  much smaller problem.

Either way, before touching anything else:

```sh
first-boot-report > /tmp/report.txt
```

That collects the three things only this board can answer — DBDC state, the
EEPROM's band and chainmask bytes, and every MAC — and it takes a second. Copy
it off with `scp`, and compare the MACs against the sticker on the case while
the board is in your hands, because that is the part nobody can do later.

## 3 · Commit to flash — 5 min

Only once step 2 looks right:

```sh
scp openwrt-...-squashfs-sysupgrade.bin root@192.168.1.1:/tmp/
ssh root@192.168.1.1 'sysupgrade -n /tmp/openwrt-...-squashfs-sysupgrade.bin'
```

Then set a WiFi password — the firmware ships with the radios enabled and
unconfigured deliberately:

```sh
uci set wireless.default_radio1.ssid='A3004-SENSOR'
uci set wireless.default_radio1.encryption='psk2'
uci set wireless.default_radio1.key='<password>'
uci commit wireless && wifi reload
iwinfo
```

## 4 · Camera — 5 min

```sh
sensor-probe            # read the VERDICT line
/etc/init.d/ustreamer start
```

The StreamCam's formats were measured on a PC already: MJPEG to 1920×1080, and
1280×720@60 is the shipped default at a measured 39.6 Mbit/s. What is **not**
known is what MT7621 does while moving it, so watch `top` — `ustreamer` should
barely register, because it copies rather than encodes. If it is eating a core,
something is transcoding and the format negotiation went wrong.

Then `http://192.168.1.1:8080/stream` in a browser.

## 5 · Lidar — 15 min

Gigabit LAN port. `sensor-probe` prints link speeds; 100 Mbit will not work and
the sensor will not negotiate it anyway.

```sh
cat /tmp/dhcp.leases                 # get the sensor's MAC
uci set dhcp.ouster.mac='<mac>'      # the host entry already exists
uci commit dhcp && /etc/init.d/dnsmasq restart
```

Point it at the router, from any machine on the LAN:

```sh
curl -X POST http://192.168.1.50/api/v1/sensor/config \
  -H 'Content-Type: application/json' \
  -d '{"udp_dest": "192.168.1.1", "udp_port_lidar": 7502}'
```

```sh
uci set ouster-edge.lidar.enabled='1'
uci set ouster-edge.lidar.sensor_ip='192.168.1.50'
uci commit ouster-edge && /etc/init.d/ouster-edge start
logread -e ouster-edge               # expect the detected profile and packet size
```

**The number to watch is `missed_columns`, and it must stay at zero:**

```sh
sensor-lan-tune
while :; do
  sed -n 's/.*"missed_columns": \([0-9]*\).*/\1/p' /var/run/ouster-edge.json
  sleep 2
done
```

- **stays 0** → the router keeps up. Note the CPU load; that is the number the
  bandwidth budget in `ARCHITECTURE.md` predicted and nobody has checked.
- **climbing** → almost certainly fragment reassembly. `sensor-lan-tune` tries a
  9000-byte MTU; whether MT7621 accepts one is itself unverified. If it does not,
  drop the sensor to 512×10 and see whether that holds — halving the data rate is
  a legitimate answer.

## 6 · Dashboard and tablet — 5 min

```
http://192.168.1.1/sensors/
```

Then join the 5 GHz AP from the tablet and open the same URL, or use the app and
enter `192.168.1.1`. Both were verified against the real daemons already, so if
either misbehaves here it is the router that is different, not them.

The **feed** pill should read `push`. If it says `polling`, port 7603 is not
reachable.

## Leave for another day

CAN and RC both need a USB hub, because the camera owns the only USB port. They
are also the two things that can wait: `can-bridge` and `rc-ibus` are verified
against `vcan` and a pty respectively, so what is untested there is only whether
the adapters enumerate. See `CAN.md` and `RC-AND-WIFI.md`.

Teleop should not be pointed at anything that moves until the deadman has been
tested with the wheels off the ground. See `TELEOP.md`.

## If you have to go back

```sh
sysupgrade /tmp/<stock-image>
```

That is the whole safety net, and it is why step 1 uses the initramfs image: a
power cycle undoes it.

## After it works

`doc/UPSTREAM.md` has the checklist for opening the two pull requests. Nothing
there should be opened before step 2 has a good answer — submitting an unflashed
device port is exactly how the first attempt got stuck.
