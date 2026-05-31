# =========================================================
# Paths
# =========================================================
PROJECT_ROOT := $(abspath ..)
export PYTHONPATH := $(PROJECT_ROOT)



VHDL_SOURCES = \
    $(PROJECT_ROOT)/rtl/Top_Level.vhd \
    $(PROJECT_ROOT)/rtl/pi_controller.vhd \
    $(PROJECT_ROOT)/rtl/pwm_generator.vhd \
    $(PROJECT_ROOT)/rtl/encoder_interface.vhd \
    $(PROJECT_ROOT)/rtl/adc_interface.vhd \
    $(PROJECT_ROOT)/rtl/uart_interface.vhd \
    $(PROJECT_ROOT)/rtl/Clock_Dividers.vhd \
    $(PROJECT_ROOT)/rtl/fixed_point_pkg.vhd \
    $(PROJECT_ROOT)/rtl/motor_constants_pkg.vhd \
    $(PROJECT_ROOT)/rtl/register_map.vhd# =========================================================
# Cocotb + Vivado XSIM (Windows-safe baseline)
# =========================================================

TOPLEVEL_LANG = vhdl

TOPLEVEL = top
COCOTB_TEST_MODULES = cocotb_tb.tests.sim_server
# -------------------------
# Simulator (IMPORTANT)
# -------------------------



SIM = ghdl
EXTRA_ARGS += --std=08

# -------------------------
# Cocotb include
# -------------------------
include $(shell cocotb-config --makefiles)/Makefile.sim
