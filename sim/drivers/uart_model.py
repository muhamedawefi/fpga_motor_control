from cocotb.triggers import RisingEdge

BAUD_CYCLES = 868  # 100MHz / 115200

async def send_byte(dut, byte):
    dut.uart_rx.value = 0
    for _ in range(BAUD_CYCLES): await RisingEdge(dut.clk)
    for i in range(8):
        dut.uart_rx.value = (byte >> i) & 1
        for _ in range(BAUD_CYCLES): await RisingEdge(dut.clk)
    dut.uart_rx.value = 1
    for _ in range(BAUD_CYCLES): await RisingEdge(dut.clk)

async def send_frame(dut, frame_bytes):
    for byte in frame_bytes:
        await send_byte(dut, byte)

async def capture_bytes(dut, count):
    result = bytearray()
    for _ in range(count):
        # 1. Wait for start bit (line goes LOW)
        while int(dut.uart_tx.value) != 0:
            await RisingEdge(dut.clk)

        # 2. Wait to CENTER of start bit
        for _ in range(BAUD_CYCLES // 2):
            await RisingEdge(dut.clk)

        # 3. Wait to CENTER of first data bit
        for _ in range(BAUD_CYCLES):
            await RisingEdge(dut.clk)

        byte = 0
        for i in range(8):
            byte |= (int(dut.uart_tx.value) << i)
            for _ in range(BAUD_CYCLES):
                await RisingEdge(dut.clk)

        result.append(byte)
    return bytes(result)
