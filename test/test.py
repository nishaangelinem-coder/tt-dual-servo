# SPDX-License-Identifier: Apache-2.0
#
# cocotb tests for the Deterministic Dual-Channel RC Servo Controller.
#
# The host side is a bit-banged SPI master (mode 0) on uio[3:0].
# Pulse widths are measured from edge timestamps on uo_out.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, Timer, ValueChange, with_timeout
from cocotb.utils import get_sim_time

CLK_NS = 100  # 10 MHz project clock
SETTLE_NS = 20  # wait after an output edge before sampling other outputs
TOL_NS = 10  # timing tolerance, well below one clock period

# uo_out bit positions
SERVO0, SERVO1, FRAME_SYNC, FAULT, WDT_EXPIRED, ARMED, UPDATE_PENDING, LIMIT_HIT = range(8)

# Register addresses
ID, VERSION, CTRL, KEY, STATUS, FAULT_REG, PRESCALE, WDT_TIMEOUT, WDT_KICK = range(9)
PERIOD, MIN0, MAX0, MIN1, MAX1 = 0x0A, 0x0C, 0x0E, 0x10, 0x12
POS0, POS1, ACT0, ACT1, FRAME_CNT = 0x14, 0x16, 0x18, 0x1A, 0x1C

# CTRL bits
COMMIT, CH0_EN, CH1_EN, FAULT_CLR, STOP_MODE = 0x01, 0x02, 0x04, 0x08, 0x10
BOTH_EN = CH0_EN | CH1_EN

# FAULT register bits
F_WDT, F_HW_EN, F_CFG, F_KEY, F_CLAMP0, F_CLAMP1, F_LOCK = (1 << i for i in range(7))

# STATUS.STATE values
ST_DISABLED, ST_ARM_PENDING, ST_RUN, ST_FAULT = range(4)

# Fast timing used by most tests: 1 clock per tick, 2500-tick frame.
FAST_PERIOD = 2500
FAST_TICK_NS = CLK_NS


class Host:
    """Bit-banged SPI master plus a few convenience helpers."""

    HALF = 5  # SCK half period in clk cycles (SCK = clk/10)

    def __init__(self, dut):
        self.dut = dut
        self.cs_n, self.mosi, self.sck = 1, 0, 0
        self._drive()

    def _drive(self):
        self.dut.uio_in.value = self.cs_n | (self.mosi << 1) | (self.sck << 3)

    async def clks(self, n):
        for _ in range(n):
            await FallingEdge(self.dut.clk)

    async def xfer(self, tx):
        """Send the bytes in tx, return the bytes clocked back on MISO."""
        rx = []
        self.cs_n = 0
        self._drive()
        await self.clks(self.HALF)
        for byte in tx:
            value = 0
            for i in range(7, -1, -1):
                self.mosi = (byte >> i) & 1
                self._drive()
                await self.clks(self.HALF)
                self.sck = 1
                self._drive()
                await self.clks(self.HALF)
                value = (value << 1) | ((int(self.dut.uio_out.value) >> 2) & 1)
                self.sck = 0
                self._drive()
            rx.append(value)
        await self.clks(self.HALF)
        self.cs_n = 1
        self._drive()
        await self.clks(self.HALF)
        return rx

    async def write(self, addr, *data):
        await self.xfer([0x80 | addr, *data])

    async def write16(self, addr, value):
        await self.write(addr, value & 0xFF, value >> 8)

    async def read(self, addr, n=1):
        rx = await self.xfer([addr] + [0] * n)
        return rx[1:]

    async def read8(self, addr):
        return (await self.read(addr))[0]

    async def read16(self, addr):
        lo, hi = await self.read(addr, 2)
        return lo | (hi << 8)

    async def state(self):
        return (await self.read8(STATUS)) & 0x3

    async def arm(self):
        await self.write(KEY, 0x5A)
        await self.write(KEY, 0xA5)

    async def kick(self):
        await self.write(WDT_KICK, 0x5C)


def now():
    """Simulation time in whole nanoseconds."""
    return round(get_sim_time("ns"))


def out_bit(dut, bit):
    return (int(dut.uo_out.value) >> bit) & 1


async def wait_level(dut, bit, level, timeout_ns):
    """Wait until uo_out[bit] == level. Returns the time of the change in ns.

    Simulators may report the bits of uo_out changing one at a time, and a
    gate-level netlist adds small delays, so after the change is seen this
    waits SETTLE_NS for the other bits to settle (and rejects glitches)."""
    deadline = now() + timeout_ns
    while True:
        if out_bit(dut, bit) == level:
            t = now()
            await Timer(SETTLE_NS, unit="ns")
            if out_bit(dut, bit) == level:
                return t
        remaining = deadline - now()
        assert remaining > 0, f"timeout waiting for uo_out[{bit}] == {level}"
        await with_timeout(ValueChange(dut.uo_out), remaining, "ns")


def same(measured_ns, expected_ns):
    """Compare two times, allowing for gate delays in gate-level simulation."""
    return abs(measured_ns - expected_ns) <= TOL_NS


async def measure_pulse(dut, bit, timeout_ns):
    """Wait for the next complete high pulse on uo_out[bit].
    Returns (rise time, width), both in ns."""
    await wait_level(dut, bit, 0, timeout_ns)
    rise = await wait_level(dut, bit, 1, timeout_ns)
    fall = await wait_level(dut, bit, 0, timeout_ns)
    return rise, fall - rise


async def start(dut, hw_en=1):
    """Start the clock, apply reset, return a Host."""
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    host = Host(dut)
    dut.rst_n.value = 0
    await host.clks(10)
    dut.rst_n.value = 1
    await host.clks(5)
    dut.ui_in.value = hw_en
    await host.clks(5)
    return host


async def fast_config(host, wdt=0):
    """1 clock per tick, 2500-tick frame, chosen watchdog timeout."""
    await host.write(PRESCALE, 0)
    await host.write16(PERIOD, FAST_PERIOD)
    await host.write(WDT_TIMEOUT, wdt)


FRAME_NS = FAST_PERIOD * FAST_TICK_NS


@cocotb.test()
async def test_reset_and_registers(dut):
    """Outputs idle after reset, ID and default register values read back."""
    host = await start(dut)

    assert out_bit(dut, SERVO0) == 0 and out_bit(dut, SERVO1) == 0
    assert out_bit(dut, ARMED) == 0 and out_bit(dut, FAULT) == 0
    assert int(dut.uio_oe.value) == 0b0000_0100

    assert await host.read8(ID) == 0x5C
    assert await host.read8(VERSION) == 0x01
    assert await host.read8(PRESCALE) == 9
    assert await host.read8(WDT_TIMEOUT) == 5

    # Burst read with address auto-increment: PERIOD, MIN0, MAX0, MIN1, MAX1
    raw = await host.read(PERIOD, 10)
    words = [raw[i] | (raw[i + 1] << 8) for i in range(0, 10, 2)]
    assert words == [20000, 1000, 2000, 1000, 2000]

    assert await host.read16(POS0) == 1500
    assert await host.read16(POS1) == 1500
    assert await host.read16(ACT0) == 1500
    assert await host.read16(ACT1) == 1500
    assert await host.state() == ST_DISABLED

    # Write and read back a 16-bit register
    await host.write16(MAX1, 2345)
    assert await host.read16(MAX1) == 2345

    # No pulses while disarmed, even with channels enabled
    await host.write(CTRL, BOTH_EN)
    await host.clks(1000)
    assert out_bit(dut, SERVO0) == 0 and out_bit(dut, SERVO1) == 0


@cocotb.test()
async def test_default_timing(dut):
    """With reset defaults: 1500 us pulses every 20 ms on both channels."""
    host = await start(dut)
    await host.write(CTRL, BOTH_EN)
    await host.arm()
    assert await host.state() in (ST_ARM_PENDING, ST_RUN)

    timeout = 25_000_000  # 25 ms
    rise0 = await wait_level(dut, SERVO0, 1, timeout)
    assert out_bit(dut, SERVO1) == 1, "both channels must rise together"
    assert out_bit(dut, ARMED) == 1
    fall0 = await wait_level(dut, SERVO0, 0, timeout)
    assert out_bit(dut, SERVO1) == 0, "equal widths must fall together"
    assert same(fall0 - rise0, 1_500_000), f"pulse width {fall0 - rise0} ns"

    await host.kick()
    rise1 = await wait_level(dut, SERVO0, 1, timeout)
    assert same(rise1 - rise0, 20_000_000), f"frame period {rise1 - rise0} ns"


@cocotb.test()
async def test_atomic_update(dut):
    """Position writes only take effect together, at a frame boundary, after COMMIT."""
    host = await start(dut)
    await fast_config(host)
    await host.write(CTRL, BOTH_EN)
    await host.arm()

    _, w0 = await measure_pulse(dut, SERVO0, 3 * FRAME_NS)
    assert same(w0, 1500 * FAST_TICK_NS)

    # Shadow writes without COMMIT must not change the outputs
    await host.write16(POS0, 1200)
    await host.write16(POS1, 1800)
    await wait_level(dut, SERVO1, 0, 3 * FRAME_NS)
    r0 = await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
    assert out_bit(dut, SERVO1) == 1
    f0 = await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    assert out_bit(dut, SERVO1) == 0, "equal widths must fall together"
    assert same(f0 - r0, 1500 * FAST_TICK_NS)
    assert await host.read16(ACT0) == 1500 and await host.read16(ACT1) == 1500

    # COMMIT: pending flag until the boundary, then both change in the same frame
    await host.write(CTRL, BOTH_EN | COMMIT)
    assert out_bit(dut, UPDATE_PENDING) == 1
    r0 = await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
    assert out_bit(dut, SERVO1) == 1
    assert out_bit(dut, UPDATE_PENDING) == 0
    f0 = await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    f1 = await wait_level(dut, SERVO1, 0, 3 * FRAME_NS)
    assert same(f0 - r0, 1200 * FAST_TICK_NS), f"ch0 width {f0 - r0}"
    assert same(f1 - r0, 1800 * FAST_TICK_NS), f"ch1 width {f1 - r0}"
    assert await host.read16(ACT0) == 1200 and await host.read16(ACT1) == 1800
    assert out_bit(dut, LIMIT_HIT) == 0

    # Frame period check with the reconfigured timebase
    r1 = await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
    assert same(r1 - r0, FRAME_NS)

    # Disabling one channel takes effect at a frame boundary
    await host.write(CTRL, CH0_EN)
    await wait_level(dut, SERVO1, 0, 3 * FRAME_NS)
    await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
    await host.clks(5)
    assert out_bit(dut, SERVO1) == 0


@cocotb.test()
async def test_limits_and_lock(dut):
    """Out-of-range requests are clamped; configuration is locked while armed."""
    host = await start(dut)
    await fast_config(host)
    await host.write16(MIN1, 1100)
    await host.write16(MAX0, 1900)
    await host.write(CTRL, BOTH_EN)
    await host.arm()
    await wait_level(dut, ARMED, 1, 3 * FRAME_NS)

    await host.write16(POS0, 2400)  # above MAX0
    await host.write16(POS1, 300)   # below MIN1
    await host.write(CTRL, BOTH_EN | COMMIT)
    await wait_level(dut, UPDATE_PENDING, 0, 3 * FRAME_NS)

    r0 = await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
    f1 = await wait_level(dut, SERVO1, 0, 3 * FRAME_NS)
    f0 = await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    assert same(f0 - r0, 1900 * FAST_TICK_NS)
    assert same(f1 - r0, 1100 * FAST_TICK_NS)
    assert out_bit(dut, LIMIT_HIT) == 1
    assert await host.read16(ACT0) == 1900 and await host.read16(ACT1) == 1100
    assert await host.read8(FAULT_REG) == F_CLAMP0 | F_CLAMP1
    assert await host.state() == ST_RUN, "clamping is a warning, not a fault"
    assert out_bit(dut, FAULT) == 0

    # An in-range commit clears LIMIT_HIT (the sticky warning bits remain)
    await host.write16(POS0, 1500)
    await host.write16(POS1, 1500)
    await host.write(CTRL, BOTH_EN | COMMIT)
    await wait_level(dut, UPDATE_PENDING, 0, 3 * FRAME_NS)
    assert out_bit(dut, LIMIT_HIT) == 0

    # Locked registers reject writes while armed
    await host.write16(MAX0, 2500)
    await host.write(PRESCALE, 3)
    assert await host.read16(MAX0) == 1900
    assert await host.read8(PRESCALE) == 0
    assert (await host.read8(FAULT_REG)) & F_LOCK
    assert await host.state() == ST_RUN

    # Any KEY write while armed disarms cleanly
    await host.write(KEY, 0x00)
    assert await host.state() == ST_DISABLED
    await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    await host.clks(2 * FAST_PERIOD)
    assert out_bit(dut, SERVO0) == 0 and out_bit(dut, SERVO1) == 0

    # Unlocked again once disarmed
    await host.write16(MAX0, 2500)
    assert await host.read16(MAX0) == 2500


@cocotb.test()
async def test_watchdog(dut):
    """Pulses continue while the host kicks, stop when it does not."""
    host = await start(dut)
    await fast_config(host, wdt=3)
    await host.write(CTRL, BOTH_EN)
    await host.arm()
    await wait_level(dut, ARMED, 1, 3 * FRAME_NS)

    # Kick once per frame for 8 frames: must stay armed
    for _ in range(8):
        await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
        await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
        await host.kick()
        assert out_bit(dut, ARMED) == 1 and out_bit(dut, FAULT) == 0

    # A wrong kick value does not reload the watchdog; stop kicking
    t0 = now()
    await host.write(WDT_KICK, 0x00)
    t_fault = await wait_level(dut, WDT_EXPIRED, 1, 5 * FRAME_NS)
    assert t_fault - t0 <= 3 * FRAME_NS + FRAME_NS // 2
    assert out_bit(dut, FAULT) == 1 and out_bit(dut, ARMED) == 0
    assert await host.state() == ST_FAULT
    assert (await host.read8(FAULT_REG)) & F_WDT

    # No pulses after the expiry
    await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    await host.clks(3 * FAST_PERIOD)
    assert out_bit(dut, SERVO0) == 0 and out_bit(dut, SERVO1) == 0

    # KEY is ignored in the FAULT state
    await host.arm()
    assert await host.state() == ST_FAULT

    # Clear and re-arm
    await host.write(CTRL, BOTH_EN | FAULT_CLR)
    assert await host.state() == ST_DISABLED
    assert await host.read8(FAULT_REG) == 0
    assert out_bit(dut, FAULT) == 0 and out_bit(dut, WDT_EXPIRED) == 0
    await host.arm()
    _, w = await measure_pulse(dut, SERVO0, 3 * FRAME_NS)
    assert same(w, 1500 * FAST_TICK_NS)


@cocotb.test()
async def test_arming_rules(dut):
    """Arming needs HW_EN, a valid configuration and the correct key sequence."""
    host = await start(dut, hw_en=0)
    await fast_config(host)
    await host.write(CTRL, BOTH_EN)

    async def expect_fault(bit):
        assert await host.state() == ST_FAULT
        assert await host.read8(FAULT_REG) == bit
        assert out_bit(dut, FAULT) == 1 and out_bit(dut, ARMED) == 0
        await host.write(CTRL, BOTH_EN | FAULT_CLR)
        assert await host.state() == ST_DISABLED

    # HW_EN low
    await host.arm()
    await expect_fault(F_HW_EN)
    dut.ui_in.value = 1
    await host.clks(5)

    # Second key byte without the first
    await host.write(KEY, 0xA5)
    await expect_fault(F_KEY)

    # Wrong byte after a correct first byte
    await host.write(KEY, 0x5A)
    await host.write(KEY, 0x5A + 1)
    await expect_fault(F_KEY)

    # MIN above MAX
    await host.write16(MIN0, 2100)
    await host.arm()
    await expect_fault(F_CFG)
    await host.write16(MIN0, 1000)

    # MAX not below the frame period
    await host.write16(MAX1, FAST_PERIOD)
    await host.arm()
    await expect_fault(F_CFG)
    await host.write16(MAX1, 2000)

    # Still no pulses after all the failed attempts
    await host.clks(2 * FAST_PERIOD)
    assert out_bit(dut, SERVO0) == 0 and out_bit(dut, SERVO1) == 0

    # Correct sequence
    await host.arm()
    _, w = await measure_pulse(dut, SERVO0, 3 * FRAME_NS)
    assert same(w, 1500 * FAST_TICK_NS)
    assert await host.state() == ST_RUN


@cocotb.test()
async def test_interlock(dut):
    """Losing HW_EN while armed stops the pulses, per STOP_MODE."""
    host = await start(dut)
    await fast_config(host)
    await host.write(CTRL, BOTH_EN)
    await host.arm()

    # STOP_MODE = 0: the pulse in progress completes, later ones are suppressed
    await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    rise = await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
    await host.clks(300)
    dut.ui_in.value = 0
    fall = await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    assert same(fall - rise, 1500 * FAST_TICK_NS), "pulse must not be truncated"
    assert out_bit(dut, FAULT) == 1 and out_bit(dut, ARMED) == 0
    await host.clks(3 * FAST_PERIOD)
    assert out_bit(dut, SERVO0) == 0 and out_bit(dut, SERVO1) == 0
    assert await host.read8(FAULT_REG) == F_HW_EN

    # STOP_MODE = 1: outputs are cut within a few clock cycles
    dut.ui_in.value = 1
    await host.write(CTRL, BOTH_EN | STOP_MODE | FAULT_CLR)
    await host.arm()
    await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    rise = await wait_level(dut, SERVO0, 1, 3 * FRAME_NS)
    await host.clks(300)
    t_drop = now()
    dut.ui_in.value = 0
    fall = await wait_level(dut, SERVO0, 0, 3 * FRAME_NS)
    assert fall - t_drop <= 6 * CLK_NS + TOL_NS, f"cut took {fall - t_drop} ns"
    assert out_bit(dut, SERVO1) == 0
    assert await host.state() == ST_FAULT
