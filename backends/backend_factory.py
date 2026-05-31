from backends.sim_backend import SimBackend

# optional imports (safe)
try:
    from backends.uart_backend import UARTBackend
except:
    UARTBackend = None

try:
    from backends.cocotb_backend import CocotbBackend
except:
    CocotbBackend = None


class BackendFactory:

    @staticmethod
    def create(mode, **kwargs):

        if mode == "Simulation":
            return SimBackend()

        elif mode == "UART":
            if UARTBackend:
                return UARTBackend(**kwargs)
            else:
                raise RuntimeError("UART backend not available")

        elif mode == "Cocotb":
            if CocotbBackend:
                dut = kwargs.get("dut")
                if dut is None:
                    raise ValueError("Cocotb backend requires 'dut'")
                return CocotbBackend(dut)
            else:
                raise RuntimeError("Cocotb backend not available")

        else:
            raise ValueError(f"Unknown mode: {mode}")