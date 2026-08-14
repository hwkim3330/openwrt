# What is inside the SCOUT's controller

Static analysis only. Nothing here was flashed.

## Read this part first: the archive does not contain firmware for this vehicle

The vehicle is a **SCOUT MINI Omni**. The archive's twenty `.bin` files are for
**SCOUT**.

`github.com/agilexrobotics/agilex_firmware` is 404; the surviving copy is
[`westonrobot/agilex_firmware`](https://github.com/westonrobot/agilex_firmware).
Its top-level README advertises three products:

> + HUNTER
> + SCOUT MINI
> + SCOUT 1.0
> + SCOUT 2.0

What is actually in the tree is not that:

| path | contents |
|---|---|
| `firmware_bin_files/scout/` | `README.md` + `bin_files/` with **20 images** |
| `firmware_bin_files/hunter/` | `README.md` only, **no images** |
| anything matching `mini` | **nothing**, in any path, on any branch |

`git ls-tree -r --all` returns zero paths matching `mini`. The SCOUT MINI is
named in the README and then never appears again.

So the question is not "which version is safe to flash" but "these are not for
this machine". Flashing a SCOUT image onto a SCOUT MINI Omni means a different
chassis, different motors and different kinematics — the Omni has lateral motion,
which the vehicle these images were built for does not have at all.

**Nothing in this archive should be written to the vehicle.**

### Where the SCOUT MINI firmware is not

Checked by `git ls-remote`, so these are existence facts rather than guesses:

| | |
|---|---|
| `agilexrobotics/agilex_firmware` | gone (the 404 that started this) |
| `agilexrobotics/scout_mini` | does not exist |
| `agilexrobotics/SCOUT_MINI` | does not exist |
| `agilexrobotics/ugv_firmware` | does not exist |
| `westonrobot/scout_mini_firmware` | does not exist |
| `westonrobot/agilex_firmware` | **exists** — the SCOUT archive analysed here |
| `agilexrobotics/scout_ros2` | exists, but a ROS 2 driver, not firmware |

Searching more widely than guessed names finds more copies of the same thing and
no new products. `agilexrobotics` is a user account rather than an organisation,
94 repositories, none of them firmware. `westonrobot` has 90, of which two carry
binaries:

| | |
|---|---|
| `westonrobot/agilex_firmware` | the 20 SCOUT images analysed here |
| `westonrobot/firmware_upgrade` | one more SCOUT image, `scout-v1.4-12-0-g070a72.bin` |
| `liweikeai2002/agilex_firmware` | an independent copy, not a fork, pushed 2019 — same 20 SCOUT images, no MINI |
| `agilexrobotics/scout_mini_omni_ros` | this exact vehicle, but a ROS package: no firmware in it |

Three independent copies of the archive, and every one of them is SCOUT only.

**There is no public SCOUT MINI firmware.** Getting one means asking AgileX or
Weston Robot for it, with the vehicle's serial number and the version it is
currently running — and that version has to be read off the vehicle over the
CP210x port either way, which is also the vendor's own advice about which
firmware to use. So the serial port comes first whatever the answer turns out to
be.

## The compatibility notes, which are about SCOUT

Worth recording accurately, because these are the notes that prompted the
caution in the first place, and they describe a different product. From
`firmware_bin_files/scout/README.md`, translated:

- The only differences between batches are the **remote control**, the **motor
  reduction gearing**, **light control** and the **cooling system**; the core is
  unchanged.
- Adapt by reference to **the version currently in use**. The things to weigh are
  remote compatibility and the gear ratio difference.
- Three remotes have shipped across the iterations: **DJI-RM, DJI-DT7,
  FS-i6s**.
- Early versions used a **1:32** reduction; later motors changed to **1:30**.
- Early versions **cannot** do light control — the hardware does not support it.
  Later ones can.
- The cooling system needs hardware support; check whether the version
  previously in use supported it.

That last piece of advice is the useful one and it removes the need for most of
what follows: the vendor's own instruction is to read the version the vehicle is
running and stay on that lineage.

## The chip

From the vector tables and the peripheral addresses these images reference. This
part is sound and is about the SCOUT controller.

Each `.bin` starts with its version string and CRLF, then the image:

```
0000  76 31 2e 34 2d 30 2d 67 37 36 30 33 38 62 65 20   v1.4-0-g76038be
0010  0d 0a 68 1f 00 20 a9 36 02 08                     ..h.. .6..
          └─ vector table starts at 0x12
```

| | |
|---|---|
| Core | Cortex-M, initial SP in SRAM at `0x2000xxxx` |
| Family | **STM32F4** (or F2) — GPIOA/B/C at `0x40020000/0400/0800` on AHB1, RCC at `0x40023800`, DMA1/2 at `0x40026000/6400`, ADC1 at `0x40012000`. An F1 would put GPIO at `0x40010800` and RCC at `0x40021000` |
| Application base | linked at `0x08020000`, so there is a **bootloader in the first 128 KiB** and the utility replaces only the application |
| CAN | **both bxCAN1 (`0x40006400`) and bxCAN2 (`0x40006800`)** — two buses, which fits an external user bus plus an internal motor bus |
| Serial | USART1, USART2, USART3 |
| Timers | TIM1, TIM2 |

The same part across every version:

| file | reset vector | vector entries |
|---|---|---|
| v1.2-8-g5b23807 | `0x080236A9` | 97 |
| v1.3-0-g431e8f0 | `0x080236A9` | 97 |
| 1.3.2-0-ge201c35 | `0x080236A9` | 97 |
| v1.3.3-1-g3a7ed85 | `0x080236A9` | 97 |
| v1.4-0-g76038be | `0x080236A9` | 97 |

Identical entry point and interrupt count throughout. The differing stack
pointer is RAM usage, not a different chip.

The archive also ships a **CP210x** driver bundle, so the vehicle's serial port
goes through a Silicon Labs USB-UART bridge. That is the port to read a version
from.

## Which protocol generation — settled, and it must be detected

`can-bridge/src/agilex.c` implements protocol **v2**. Whether that matches the
vehicle is the one question that has to be answered before anything is
commanded, and the vendor answers it twice over.

**The Scout Mini Omni can be either generation.** `ugv_sdk`'s own
`sample/scout_demo/scout_mini_omni_demo.cpp` constructs
`ScoutMiniOmniRobot(ProtocolVersion::AGX_V1)` or `AGX_V2` according to a runtime
`ProtocolDetector`. The vendor does not assume, for this exact model, so neither
should we.

**The discriminators are exact.** From `src/utilities/protocol_detector.cpp`:

| heard on the bus | verdict |
|---|---|
| `0x151` — state feedback | **v1** ("unique to V1 protocol") |
| `0x221` or `0x241` — motion state, rc state | **v2** ("unique to V2 protocol") |
| both | **UNKNOWN** — the detector refuses to choose |
| neither, within the timeout | **UNKNOWN** |

Detection is entirely passive: the detector installs a receive callback and puts
nothing on the bus.

`can-bridge` now implements exactly this, always, not only under `--discover`.
It logs one line when the generation is first settled, warns rather than informs
when the answer is v1, logs the conflicting case separately, and publishes
`"agilex_protocol": "v1" | "v2" | "unknown"` in its status JSON. `unknown` covers
both "nothing heard yet" and the conflict, and a consumer should treat either as
do-not-command. `test/test_bridge.py` covers all five cases, including that a bus
carrying only `0x251`/`0x252` still reports `unknown` — the bridge must not
default to the generation its decoder happens to implement.

An earlier version of this document said "ids around `0x211`, `0x221`, `0x251…` →
v2". That was imprecise: `0x211` and `0x251` are not discriminators, and the
positive marker for v1 is `0x151`, which was not mentioned at all.

## Two methods that did not work, so they are not repeated

**Searching the images for CAN ids as byte patterns.** Each 11-bit id was tried
raw, as `<< 21` (bxCAN's `CAN_TIxR` field position) and as `<< 5`. The result
looked decisive and was noise: the strongest "v1" evidence was `0x200 << 21`,
which is `0x40000000` — the peripheral base region, present in any STM32 image
many times over. Do not score a firmware this way.

**Searching for the gear ratio as a constant.** The ratio is not there as
`30`/`32` in any width, nor as the floats 32.0, 30.0, 1/32, 1/30, or those times
60. A stronger test also failed: enumerate every 4-byte aligned word, keep those
present in some images and absent from others, and keep the ones whose presence
splits the set contiguously in version order. **1792 words** pass that filter, and
the top candidates are `0x48xx` half-words — Thumb-2 `ldr rN, [pc, #imm]` — so
they are code motion between recompiles, not data. Twenty separate builds differ
everywhere; a partition carries no information.

Recovering the ratio would need actual disassembly of the velocity path. It is
not worth it, because the vendor's advice makes it unnecessary.

## What to do

In order, and none of it involves writing to the vehicle:

1. **Get a USB-CAN adapter.** Nothing below is possible without one, and it is
   the cheapest unblocking purchase in the project.
2. **Listen.** `can-bridge --interface can0 --discover` with `allow_inject` off,
   which is the default. Read `agilex_protocol` from the status file. If it says
   `v1`, `agilex.c` and `agx_encode_motion()` do not apply to this vehicle and
   writing them onto the bus would be commanding it in a language it does not
   speak.
3. **Read the version** over the CP210x serial port. That, plus the SCOUT README's
   "adapt by reference to the version currently in use", is the whole of the
   firmware question.
4. **Do not flash anything from this archive.** It has no SCOUT MINI images. If
   the vehicle ever does need firmware, it has to come from a source that names
   the SCOUT MINI.
