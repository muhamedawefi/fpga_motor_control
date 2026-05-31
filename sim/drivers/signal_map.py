# cocotb_tb/drivers/signal_map.py

class MotorSignals:
    def __init__(self, dut):
        self.w_ref = dut.w_ref
        self.w_meas = dut.w_meas
        self.i_meas = dut.i_meas
        self.v_cmd = dut.v_cmd