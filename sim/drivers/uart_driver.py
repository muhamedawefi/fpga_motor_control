from cocotb.triggers import RisingEdge
from cocotb_tb.drivers.uart_model import send_frame, capture_bytes
from backends.utils import compute_checksum

class UARTDriver:
    def __init__(self, dut):
        self.dut = dut

    async def write(self, addr, value):
        frame = [0xAA, 0x01, addr, (value >> 8) & 0xFF, value & 0xFF]
        frame.append(compute_checksum(frame))
        await send_frame(self.dut, frame)

    async def read(self, addr):
        frame = [0xAA, 0x02, addr, 0x00, 0x00]
        frame.append(compute_checksum(frame))
        await send_frame(self.dut, frame)
        # capture_bytes will automatically wait for the response start bit
        return await capture_bytes(self.dut, 5)

    async def read_telemetry(self):
        # capture_bytes will automatically wait for the telemetry frame start bit
        raw = await capture_bytes(self.dut, 8)

        if raw[0] != 0xFF:
            return None

        chk = 0xFF
        for b in raw[1:7]:   # skip the header byte raw[0]
            chk ^= b
        if chk != raw[7]:
            return None

        return {
             "speed_raw": (((raw[1] << 8) | raw[2]) - 65536) if ((raw[1] << 8) | raw[2]) >= 32768 else ((raw[1] << 8) | raw[2]),
             "current_raw": (((raw[3] << 8) | raw[4]) - 65536) if ((raw[3] << 8) | raw[4]) >= 32768 else ((raw[3] << 8) | raw[4]),
             "vcmd_raw": (((raw[5] << 8) | raw[6]) - 65536) if ((raw[5] << 8) | raw[6]) >= 32768 else ((raw[5] << 8) | raw[6])
        }
    async def enable_streaming(self, period_ms: int):
        dh = (period_ms >> 8) & 0xFF
        dl = period_ms & 0xFF
        frame = [0xAA, 0x03, 0x00, dh, dl]
        frame.append(compute_checksum(frame))
        await send_frame(self.dut, frame)
