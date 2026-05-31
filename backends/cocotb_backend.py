from backends.zmq_publisher import ZMQPublisher

class CocotbBackend:
    def __init__(self, dut):
        self.dut  = dut
        self._data = {}
        self._pub = ZMQPublisher()

    def start_stream(self):
        pass

    def stop_stream(self):
        pass

    def sample(self):
        if not self.dut.speed_fb.value.is_resolvable:
            return
        self._data = {
            "speed_raw":   int(self.dut.speed_fb.value.to_signed()),
            "current_raw": int(self.dut.sim_current_ma.value.to_signed()),
            "vcmd_raw":    int(self.dut.v_cmd_test.value.to_signed()),
        }
        self._pub.publish(self._data)

    def get_data(self):
        return self._data
