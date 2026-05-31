import cocotb
from cocotb.triggers import RisingEdge, ClockCycles
from cocotb_tb.env.clock_reset import setup_clock, reset_dut
from cocotb_tb.drivers.uart_driver import UARTDriver
from backends.cocotb_backend import CocotbBackend
from backends.sim_runner import Simulation

def init_inputs(dut):
    dut.enc_a.value          = 0
    dut.enc_b.value          = 0
    dut.uart_rx.value        = 1   # ← idle high, kills the 0x27 garbage
    dut.uart_addr.value      = 0
    dut.uart_data_in.value   = 0
    dut.uart_wr.value        = 0
    dut.uart_rd.value        = 0
    dut.spi_miso.value       = 0
    dut.spi_cs.value         = 1
    dut.spi_clk.value        = 0
    dut.speed_fb.value       = 0
    dut.sim_current_ma.value = 0

@cocotb.test(timeout_time=100, timeout_unit='ms')
async def run_app_bridge(dut):
    init_inputs(dut)
    await setup_clock(dut)
    await ClockCycles(dut.clk, 1)
    await reset_dut(dut)
    await ClockCycles(dut.clk, 10)

    backend = CocotbBackend(dut)
    sim     = Simulation(dut, backend)

    uart = UARTDriver(dut)
    await uart.write(0x00, 3200)        # 25.0 rad/s setpoint
    await ClockCycles(dut.clk, 50000)  # let PI react before loop

    backend.start_stream()

    for _ in range(1_000_000):
        await sim.step()

    assert abs(sim.speed)   > 1,   f"Plant never moved: {sim.speed:.2f}"
    assert abs(sim.speed)   < 300, f"Speed diverged: {sim.speed:.2f}"
    assert abs(sim.current) < 50,  f"Current diverged: {sim.current:.3f}"

    cocotb.log.info(
        f"✓ Bridge OK | speed={sim.speed:.2f} rad/s | current={sim.current:.3f} A"
    )
