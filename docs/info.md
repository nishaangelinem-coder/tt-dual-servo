<!---

This file is used to generate project datasheet. 
-->

## How it works

This project is a dual-channel RC servo command controller. It generates two standard servo pulse trains (default 20 ms frame, 1000–2000 µs pulse) with 1 µs resolution, and is configured by a host microcontroller over SPI. It produces logic-level command signals only: servo power and the servo's internal position loop are external.

**Timebase.** A programmable prescaler divides the project clock (10 MHz nominal) to a 1 MHz tick. A 15-bit counter counts ticks up to the programmed frame period. Both servo outputs rise together at the start of each frame and fall when the counter reaches that channel's active pulse width.

**Atomic updates.** The host writes requested positions to shadow registers (`POS0`, `POS1`). Nothing changes at the outputs until the host sets `COMMIT`. At the next frame boundary both shadow values are clamped to their channel limits and copied to the active registers in the same clock cycle, so the two channels always change together and never mid-pulse.

**Limits.** Each channel has its own `MIN` and `MAX` pulse width. A request outside the limits is clamped and a warning bit is latched. Limits, frame period, prescaler and watchdog timeout can only be written while the controller is disarmed.

**Enable sequence.** After reset the outputs are low. To arm, the `HW_EN` pin must be high and the host must write `0x5A` then `0xA5` to the `KEY` register. Pulses start at the next frame boundary. Any other write to `KEY` while armed disarms the controller.

**Watchdog.** While armed, the host must write `0x5C` to `WDT_KICK` at least once every `WDT_TIMEOUT` frames (default 5 frames, about 100 ms). If it does not, pulses stop and the `WDT` fault is latched.

**Faults.** Watchdog expiry, loss of `HW_EN` while armed, an invalid configuration at arm time, or a wrong key sequence put the controller in the FAULT state. By default the pulse in progress is completed and later pulses are suppressed; setting `STOP_MODE` cuts the outputs immediately instead. The host reads the cause from the `FAULT` register, clears it with `FAULT_CLR`, and must arm again.

### SPI protocol

SPI mode 0, MSB first, SCK at most clk/8. The first byte is `{W, ADDR[6:0]}` (W = 1 for write). Data bytes follow, with the address auto-incrementing after each byte. 16-bit values are little-endian.

### Register map

| Addr | Name | Access | Reset | Description |
|------|------|--------|-------|-------------|
| 0x00 | ID | RO | 0x5C | Device ID |
| 0x01 | VERSION | RO | 0x01 | Design revision |
| 0x02 | CTRL | RW | 0x00 | bit0 COMMIT, bit1 CH0_EN, bit2 CH1_EN, bit3 FAULT_CLR, bit4 STOP_MODE |
| 0x03 | KEY | WO | – | Arm: write 0x5A then 0xA5. Any write while armed disarms |
| 0x04 | STATUS | RO | – | bits1:0 STATE, bit2 UPDATE_PENDING, bit3 HW_EN, bit4 CH0_ACTIVE, bit5 CH1_ACTIVE, bit7 ANY_FAULT |
| 0x05 | FAULT | RO | 0x00 | bit0 WDT, bit1 HW_EN_LOST, bit2 CFG_INVALID, bit3 KEY_ERR, bit4 CLAMP0, bit5 CLAMP1, bit6 LOCK_ERR |
| 0x06 | PRESCALE | RW (locked when armed) | 9 | Clock cycles per 1 µs tick, minus 1 |
| 0x07 | WDT_TIMEOUT | RW (locked when armed) | 5 | Watchdog timeout in frames, 0 disables |
| 0x08 | WDT_KICK | WO | – | Write 0x5C to reload the watchdog |
| 0x0A–0x0B | PERIOD | RW (locked when armed) | 20000 | Frame period in µs |
| 0x0C–0x0D | MIN0 | RW (locked when armed) | 1000 | Channel 0 lower limit in µs |
| 0x0E–0x0F | MAX0 | RW (locked when armed) | 2000 | Channel 0 upper limit in µs |
| 0x10–0x11 | MIN1 | RW (locked when armed) | 1000 | Channel 1 lower limit in µs |
| 0x12–0x13 | MAX1 | RW (locked when armed) | 2000 | Channel 1 upper limit in µs |
| 0x14–0x15 | POS0 | RW | 1500 | Channel 0 requested pulse width in µs |
| 0x16–0x17 | POS1 | RW | 1500 | Channel 1 requested pulse width in µs |
| 0x18–0x19 | ACT0 | RO | 1500 | Channel 0 active pulse width after clamping |
| 0x1A–0x1B | ACT1 | RO | 1500 | Channel 1 active pulse width after clamping |
| 0x1C | FRAME_CNT | RO | 0 | Free-running frame counter |

STATE values: 0 = DISABLED, 1 = ARM_PENDING, 2 = RUN, 3 = FAULT.

## How to test

1. Set the project clock to 10 MHz and apply reset. Both servo outputs stay low.
2. Read register `0x00` over SPI and check that it returns `0x5C`.
3. Drive `HW_EN` (`ui[0]`) high.
4. Write `0x06` to `CTRL` (`0x02`) to enable both channels.
5. Write `0x5A` and then `0xA5` to `KEY` (`0x03`). `ARMED` (`uo[5]`) goes high at the next frame boundary and both outputs show a 1500 µs pulse every 20 ms.
6. From this point, write `0x5C` to `WDT_KICK` (`0x08`) at least every 100 ms.
7. Write new 16-bit values to `POS0` and `POS1`, then write `0x07` to `CTRL` (COMMIT with both channels still enabled). Both pulse widths change together at the next frame boundary.
8. Request a value outside the limits (for example 2500 µs). The output is clamped to `MAX`, `LIMIT_HIT` (`uo[7]`) goes high, and the clamp bit is set in `FAULT`.
9. Stop kicking the watchdog. After 5 frames the pulses stop and `FAULT` (`uo[3]`) and `WDT_EXPIRED` (`uo[4]`) go high.
10. Write `0x08` to `CTRL` to clear the fault, then repeat the arm sequence.

Pulse widths and frame timing can be checked with an oscilloscope or logic analyser on `SERVO0`, `SERVO1` and `FRAME_SYNC` (`uo[2]`).

## External hardware

- A host microcontroller with an SPI master (the RP2040 on the Tiny Tapeout demo board can be used).
- One or two standard RC servos with their own 5–6 V supply. Connect only the servo signal wire to `SERVO0` / `SERVO1` and share ground with the demo board. Do not power servos from the chip's I/O supply.
- A level shifter or buffer if the servo requires a 5 V signal level.
- Optional: oscilloscope or logic analyser for timing checks.
