from backend_interface import BackendInterface
from uart_handler import UARTHandler
from utils import build_write_frame, build_stream_frame, build_stop_frame

class UARTBackend(BackendInterface):
    def __init__(self, port="COM3", baud=115200):
        self.hw = UARTHandler(port, baud)

        self.last_ref = 0.0
        self.last_frame = None
        self.chk_errors = 0
        self.connected = True

    # =========================
    # COMMANDS
    # =========================

    def set_speed(self, speed_rad_s):
        self.last_ref = speed_rad_s

        raw = int(speed_rad_s * 128)
        frame = build_write_frame(0x00, raw)

        self.hw.send(frame)

    def start_stream(self, period_ms=10):
        frame = build_stream_frame(period_ms)
        self.hw.send(frame)

    def stop_stream(self):
        frame = build_stop_frame()
        self.hw.send(frame)

    # =========================
    # DATA
    # =========================

    def get_data(self):
        parsed = self.hw.get_latest()

        if parsed:
            self.last_frame = None  # optional (you can extend later)

            return {
                "speed": parsed["speed_raw"] / 256,
                "current": parsed["current_raw"] / 4096,
                "vcmd": parsed["vcmd_raw"] / 1024,
                "ref": self.last_ref
            }

        return None