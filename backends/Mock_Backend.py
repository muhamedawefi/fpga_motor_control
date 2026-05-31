from backends.backend_interface import BackendInterface
import time
import math

class MockBackend(BackendInterface):
    def __init__(self):
        self._setpoint = 0.0
        self._t0 = time.time()

    def set_speed(self, value):
        self._setpoint = value

    def start_stream(self):
        pass

    def stop_stream(self):
        pass

    def get_data(self):
        t = time.time() - self._t0
        speed   = self._setpoint * (1 - math.exp(-t * 2))
        current = speed * 0.03
        return {
            "speed_raw":   int(speed   * 128),
            "current_raw": int(current * 4096),
            "vcmd_raw":    int(speed   * 10),
        }
