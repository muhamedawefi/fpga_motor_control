# cocotb_tb/env/dut_wrapper.py

from cocotb.triggers import RisingEdge


class DUTWrapper:
    def __init__(self, dut, signals):
        self.dut = dut
        self.s = signals



    async def set_speed(self, value):
        self.s.w_ref.value = int(value)



    async def step(self, cycles=1):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk)




    def read(self): 
        def safe(v):
            if not v.value.is_resolvable:
               return 0
            return int(v.value)

        return {
        "speed": safe(self.s.w_meas),
        "current": safe(self.s.i_meas),
        "vcmd": safe(self.s.v_cmd),
    }
      
