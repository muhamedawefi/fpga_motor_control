# test_uart.py - ALL TESTS FIXED
import cocotb
from cocotb.triggers import RisingEdge, ClockCycles
from cocotb_tb.env.clock_reset import setup_clock, reset_dut
from cocotb_tb.drivers.uart_driver import UARTDriver

def init_inputs(dut):
    """Helper: initialize all inputs to prevent metavalues"""
    dut.enc_a.value = 0
    dut.enc_b.value = 0
    dut.uart_rx.value = 1
    dut.uart_addr.value = 0
    dut.uart_data_in.value = 0
    dut.uart_wr.value = 0
    dut.uart_rd.value = 0
    dut.spi_miso.value = 0
    dut.spi_cs.value = 1
    dut.spi_clk.value = 0
    dut.speed_fb.value = 0
    dut.sim_current_ma.value = 0
@cocotb.test(timeout_time=10, timeout_unit='ms')
async def test_write_setpoint(dut):
    init_inputs(dut)
    await setup_clock(dut)         # ← Clock FIRST
    await ClockCycles(dut.clk, 1)  # ← Now clock is toggling, safe to wait
    await reset_dut(dut)
    await ClockCycles(dut.clk, 10)

    uart = UARTDriver(dut)
    await uart.write(0x00, 6400)
    await ClockCycles(dut.clk, 2000)

    sp = dut.speed_setpoint_debug.value.to_signed()
    assert sp == 6400
    cocotb.log.info(f"✓ speed_setpoint = {sp}")

@cocotb.test(timeout_time=20, timeout_unit='ms')
async def test_read_register(dut):
    init_inputs(dut)
    await setup_clock(dut)         # ← Clock FIRST
    await ClockCycles(dut.clk, 1)
    await reset_dut(dut)
    await ClockCycles(dut.clk, 10)

    uart = UARTDriver(dut)
    await uart.write(0x00, 6400)
    await ClockCycles(dut.clk, 60000)  # 🔑 Wait for full TX + RX parsing to complete
    resp = await uart.read(0x00)
    # ✅ RESTORE THESE ASSERTIONS:
    assert resp[0] == 0x55, f"Wrong header: {hex(resp[0])}"
    assert resp[1] == 0x00, f"Wrong addr: {hex(resp[1])}"
    value = (resp[2] << 8) | resp[3]
    assert value == 6400, f"Wrong value: {value}"
    chk = 0x55 ^ resp[1] ^ resp[2] ^ resp[3]
    assert chk == resp[4], f"Checksum failed"
    cocotb.log.info(f"✓ READ response correct, value={value}")

@cocotb.test(timeout_time=100, timeout_unit='ms')
async def test_telemetry_stream(dut):
    init_inputs(dut)
    await setup_clock(dut)
    await ClockCycles(dut.clk, 1)
    await reset_dut(dut)
    await ClockCycles(dut.clk, 10)

    uart = UARTDriver(dut)
    
    # 🔑 STEP 1: Send a non-zero setpoint to excite the system
    await uart.write(0x00, 3200)  # Q9.7: 3200/128 = 25.0 rad/s target
    await ClockCycles(dut.clk, 50000)  # Let controller react (~0.5ms)
    
    # STEP 2: Initialize plant state
    speed = 0.0
    current = 0.0
    dut.speed_fb.value = 0
    dut.sim_current_ma.value = 0

    # STEP 3: Enable streaming with 1ms period
    await uart.enable_streaming(1)  # CMD=0x03, period=1ms
    
    # STEP 4: Run closed-loop simulation
    for _ in range(200000):  # 2ms at 100MHz
        await RisingEdge(dut.clk)
        if not dut.v_cmd_test.value.is_resolvable:
            continue
        v = float(dut.v_cmd_test.value.to_signed()) / 1024.0
        speed += 0.0005 * v - 0.001 * speed
        current = 0.1 * v
        #  🔑 Clamp to 16-bit signed range [-32768, 32767]
        speed_fb_int = int(speed * 128)
        speed_fb_int = max(-32768, min(32767, speed_fb_int))
        dut.speed_fb.value = speed_fb_int

        current_fb_int = int(current * 4096)
        current_fb_int = max(-32768, min(32767, current_fb_int))
        dut.sim_current_ma.value = current_fb_int    
    # STEP 5: Wait for at least one telemetry frame to transmit
    await ClockCycles(dut.clk, 1200000)  # Extra 1ms buffer
    
    # STEP 6: Capture and verify telemetry
    frame = await uart.read_telemetry()
    assert frame is not None, "No telemetry received"

    speed_raw = frame["speed_raw"]
    current_raw = frame["current_raw"]
    speed_measured = speed_raw / 128.0
    current_measured = current_raw / 4096.0

    # Allow larger tolerance since plant model is simplified
    assert abs(speed_measured - speed) < 10, f"Speed mismatch: {speed_measured} vs {speed}"
    assert abs(current_measured - current) < 5, f"Current mismatch: {current_measured} vs {current}"
    cocotb.log.info(f"✓ Telemetry OK | speed={speed_measured:.2f} | current={current_measured:.2f}")
