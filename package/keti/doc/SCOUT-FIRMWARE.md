# What is inside the SCOUT's controller

Static analysis only. Nothing here was flashed, and nothing should be until the
generation question below is settled.

## Where the firmware is

The address the older SCOUT MINI documents point at,
`github.com/agilexrobotics/agilex_firmware`, is **404**. A copy survives at
[`westonrobot/agilex_firmware`](https://github.com/westonrobot/agilex_firmware)
(38 MB, last pushed 2020-09-02) with `firmware_bin_files/scout/bin_files/`
holding twenty `.bin` files across v1.2, v1.3, 1.3.2, 1.3.3 and v1.4, plus a
Windows Qt flashing tool (`FirmwareUpgradeV1.47`) and a CP210x driver bundle.

That last detail is useful on its own: the vehicle's serial port goes through a
**Silicon Labs CP210x** USB-UART bridge, which is what to expect when reading
version information over the internal DB9.

## The container

Each `.bin` starts with its own version string and CRLF, then the firmware:

```
0000  76 31 2e 34 2d 30 2d 67 37 36 30 33 38 62 65 20   v1.4-0-g76038be
0010  0d 0a 68 1f 00 20 a9 36 02 08                     ..h.. .6..
          └─ vector table starts at 0x12
```

## The chip

From the vector table and the peripheral addresses the code references:

| | |
|---|---|
| Core | Cortex-M, initial SP in SRAM at `0x2000xxxx` |
| Family | **STM32F4** (or F2) — GPIOA/B/C at `0x40020000/0400/0800` on AHB1, RCC at `0x40023800`, DMA1/2 at `0x40026000/6400`, ADC1 at `0x40012000`. An F1 would put GPIO at `0x40010800` and RCC at `0x40021000` |
| Application base | linked to run at `0x08020000`, so there is a **bootloader in the first 128 KiB** and the utility replaces only the application |
| CAN | **both bxCAN1 (`0x40006400`) and bxCAN2 (`0x40006800`)** — two buses, which fits an external user bus plus an internal motor bus |
| Serial | USART1, USART2 and USART3 all referenced |
| Timers | TIM1 and TIM2 |

## The same chip across every version

| file | reset vector | vector entries | CAN1 refs | CAN2 refs |
|---|---|---|---|---|
| v1.2-8-g5b23807 | `0x080236A9` | 97 | 5 | 11 |
| v1.3-0-g431e8f0 | `0x080236A9` | 97 | 5 | 10 |
| 1.3.2-0-ge201c35 | `0x080236A9` | 97 | 3 | 4 |
| v1.3.3-1-g3a7ed85 | `0x080236A9` | 97 | 3 | 4 |
| v1.4-0-g76038be | `0x080236A9` | 97 | 3 | 4 |

Identical entry point and identical interrupt count in all of them. The stack
pointer differs between builds, which is RAM usage rather than a different part.

**So the compatibility warnings are not about hardware.** The differences the
documentation describes — DJI versus FS-i6S remote, 1:32 versus 1:30 gear ratio —
are configuration compiled into the same binary for the same MCU. Two things
follow:

- Flashing the wrong version is unlikely to brick the controller: same chip, same
  bootloader, same layout.
- It will change behaviour. A gear ratio compiled in wrong means every commanded
  velocity is scaled wrong, and a vehicle that moves at the wrong speed is worse
  than one that does not move.

## Two code lineages

`v1.2`/`v1.3` reference CAN2 ten or eleven times; `1.3.2` onward reference it
four. The constant `32` appears sixteen or seventeen times in the first group and
nine in the second. Something about the CAN handling was reworked between them,
and that is the most likely place the generation split lives.

The gear ratio itself was **not** identified. `30` does not appear as a 32-bit
integer in any of the twenty files and `32` appears too often to attribute - it is
a normal buffer size. Whatever encodes the ratio is not a plain literal.

## What to do before flashing anything

The protocol generation matters more than the firmware version, because
`can-bridge/src/agilex.c` implements **protocol v2**. If this vehicle speaks v1,
both the decoder and `agx_encode_motion()` are talking to the wrong protocol.

That is answerable without writing to the vehicle at all:

```sh
can-bridge --interface can0 --discover      # read-only; logs every distinct id
```

- ids around `0x211`, `0x221`, `0x251…` → protocol v2, and the existing code fits
- a different id range → protocol v1, and the decoder and encoder need writing
  for that generation

`--allow-inject` is off by default, so nothing reaches the bus. Do this first,
read the version over the CP210x serial port second, and only then consider
whether any firmware needs replacing at all.
