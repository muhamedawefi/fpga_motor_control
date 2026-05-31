from cocotb_tools.runner import get_runner
import os

proj_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

runner = get_runner("xsim")

runner.build(
    vhdl_sources=[
        f"{proj_path}/rtl/top.vhd",
        f"{proj_path}/rtl/controller.vhd",
        f"{proj_path}/rtl/pwm.vhd",
        f"{proj_path}/rtl/encoder.vhd",
        f"{proj_path}/rtl/adc.vhd",
        f"{proj_path}/rtl/uart_rx.vhd",
        f"{proj_path}/rtl/uart_tx.vhd",
    ],
    hdl_toplevel="top",
    build_args=["--vhdl2008"],
)

runner.test(
    hdl_toplevel="top",
    test_module="cocotb_tb.tests.test_system",
)