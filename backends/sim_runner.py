from cocotb.triggers import RisingEdge

class Simulation:
    def __init__(self, dut, backend):
        self.dut     = dut
        self.backend = backend
        self.speed   = 0.0
        self.current = 0.0
        # Motor parameters
        self.R  = 2.5
        self.L  = 0.0005
        self.J  = 0.0001
        self.B  = 0.001      # increased from 0.0001 — prevents divergence
        self.Kt = 0.05
        self.Ke = 0.05
        self.DT = 10e-9     # 100MHz clock

    def _clamp16(self, val):
        return max(-32768, min(32767, int(val)))

    async def step(self):
        await RisingEdge(self.dut.clk)
        if not self.dut.v_cmd_test.value.is_resolvable:
            return None

        v = self.dut.v_cmd_test.value.to_signed() / 1024.0
        di_dt = (v - self.R * self.current - self.Ke * self.speed) / self.L
        dw_dt = (self.Kt * self.current - self.B * self.speed) / self.J

        self.current += di_dt * self.DT
        self.speed   += dw_dt * self.DT

        self.dut.sim_current_ma.value = self._clamp16(self.current * 4096)
        self.dut.speed_fb.value       = self._clamp16(self.speed   * 128)

        self.backend.sample()
        return self.backend.get_data()
