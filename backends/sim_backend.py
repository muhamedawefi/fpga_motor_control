from .backend_interface import BackendInterface
from core import full_fixed as fp
import time


class SimBackend(BackendInterface):

    def __init__(self):
        self.fp = fp
        self.fp.reset()

        # motor state
        self.w = 0.0
        self.i = 0.0

        # motor model
        self.R = 2.5
        self.L = 0.0005
        self.J = 0.0001
        self.B = 0.0001
        self.KT = 0.05
        self.KE = 0.05

        # simulation step (10 kHz)
        self.dt = 1e-4

        # streaming
        self.stream_enabled = False
        self.period = 0.01

        # real-time sync
        self.last_time = time.time()

    # -------------------------
    # control
    # -------------------------
    def set_speed(self, speed_rad_s):
        self.fp.w_ref_raw = int(speed_rad_s * self.fp.SPEED_SCALE)

    def start_stream(self, period_ms=10):
        self.stream_enabled = True
        self.period = period_ms * 1e-3

        # 🔥 RESET TIME
        self.last_time = time.time()

    def stop_stream(self):
        self.stream_enabled = False

    # -------------------------
    # physics step
    # -------------------------
    def step(self):
        self.fp.clock_tick()

        v = self.fp.v_cmd_raw / self.fp.VOLTAGE_SCALE

        di_dt = (v - self.R * self.i - self.KE * self.w) / self.L
        dw_dt = (self.KT * self.i - self.B * self.w) / self.J

        self.i += di_dt * self.dt
        self.w += dw_dt * self.dt

        # feedback (clamped)
        self.fp.w_meas_raw = self.fp.saturate(
            int(self.w * self.fp.SPEED_SCALE), -32768, 32767
        )
        self.fp.i_meas_raw = self.fp.saturate(
            int(self.i * self.fp.CURRENT_SCALE), -32768, 32767
        )

    # -------------------------
    # telemetry (REAL TIME)
    # -------------------------
    def get_data(self):
        if not self.stream_enabled:
            return None
        now = time.time()
        dt_real = now - self.last_time
        self.last_time = now

        steps = int(dt_real / self.dt)

        # 🔥 CRITICAL FIX
        steps = min(steps, 50)  # NOT 1000

        for _ in range(steps):
            self.step()

        return {
            "speed": self.w,
            "current": self.i,
            "vcmd": self.fp.v_cmd_raw / self.fp.VOLTAGE_SCALE,
            "ref": self.fp.w_ref_raw / self.fp.SPEED_SCALE
        }