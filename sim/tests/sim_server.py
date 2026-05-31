import cocotb
from cocotb.triggers import ClockCycles
from cocotb_tb.env.clock_reset import setup_clock, reset_dut
from cocotb_tb.drivers.uart_driver import UARTDriver
from backends.cocotb_backend import CocotbBackend
from backends.sim_runner import Simulation
import zmq
import json

def init_inputs(dut):
    dut.enc_a.value          = 0
    dut.enc_b.value          = 0
    dut.uart_rx.value        = 1
    dut.uart_addr.value      = 0
    dut.uart_data_in.value   = 0
    dut.uart_wr.value        = 0
    dut.uart_rd.value        = 0
    dut.spi_miso.value       = 0
    dut.spi_cs.value         = 1
    dut.spi_clk.value        = 0
    dut.speed_fb.value       = 0
    dut.sim_current_ma.value = 0

@cocotb.test(timeout_time=3600, timeout_unit='sec')
async def sim_server(dut):
    init_inputs(dut)
    await setup_clock(dut)
    await ClockCycles(dut.clk, 1)
    await reset_dut(dut)
    await ClockCycles(dut.clk, 10)

    backend = CocotbBackend(dut)
    sim     = Simulation(dut, backend)
    uart    = UARTDriver(dut)

    ctx = zmq.Context()
    cmd = ctx.socket(zmq.SUB)
    cmd.connect("tcp://localhost:5556")
    cmd.setsockopt_string(zmq.SUBSCRIBE, "")
    cmd.setsockopt(zmq.RCVTIMEO, 0)

    await uart.write(0x00, 0)

    step_count = 0
    cocotb.log.info("sim_server ready — waiting for commands on port 5556")
    while True:
        try:
            msg = cmd.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(msg)
            setpoint_q97 = int(data["setpoint"] * 128)
            cocotb.log.info(f"CMD received: setpoint={data['setpoint']} Q9.7={setpoint_q97}")
            await uart.write(0x00, setpoint_q97)
        except zmq.Again:
            pass

        await sim.step()
        step_count += 1

        if step_count % 100000 == 0:
            cocotb.log.info(
                f"SIM step={step_count} | "
                f"speed={sim.speed:.3f} rad/s | "
                f"current={sim.current:.4f} A | "
                f"speed_fb={int(sim.speed*128)}"
            )
