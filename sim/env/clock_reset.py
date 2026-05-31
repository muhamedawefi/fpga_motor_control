import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


async def setup_clock(dut, period_ns=10):
    """
    Start the clock and wait 1 cycle so everything aligns.
    """
    cocotb.start_soon(Clock(dut.clk, period_ns, units="ns").start())

    # allow first delta cycle + clock stabilization
    await RisingEdge(dut.clk)


async def reset_dut(dut, cycles=5, settle_cycles=50):
    """
    Proper reset sequence for control-system simulation.

    cycles        -> how long reset is asserted
    settle_cycles -> stabilization time AFTER reset
    """

    # -------------------------
    # ASSERT RESET
    # -------------------------
    dut.reset.value = 1

    for _ in range(cycles):
        await RisingEdge(dut.clk)

    # -------------------------
    # DEASSERT RESET
    # -------------------------
    dut.reset.value = 0

    # IMPORTANT: wait for internal RTL to fully exit reset
    for _ in range(settle_cycles):
        await RisingEdge(dut.clk)


async def soft_start(dut, cycles=50):
    """
    Optional helper: keeps plant frozen after reset
    to avoid first-cycle PI explosion.
    """

    dut.speed_fb.value = 0
    dut.sim_current_ma.value = 0

    for _ in range(cycles):
        await RisingEdge(dut.clk)
