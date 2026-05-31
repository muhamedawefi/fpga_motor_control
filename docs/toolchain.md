# Toolchain

## Overview

| Stage | Simulator | Reason |
|-------|-----------|--------|
| Initial RTL development | Vivado xsim | Default Vivado flow |
| Verification infrastructure | GHDL + WSL | cocotb compatibility, stricter checking |

The simulation environment moved from Vivado's built-in xsim to GHDL running under WSL partway through the project. This document explains why, what broke during the transition, and how to set up the current environment.

---

## Why the Transition Happened

### Problems with xsim + cocotb

Vivado's xsim works well for manual waveform-based simulation. For automated cocotb testing it has serious limitations:

**Fragile integration.** cocotb support under xsim requires a specific export flow (`xelab` with shared library flags) that breaks silently across Vivado versions. Setting it up on a fresh machine is non-trivial and not well documented.

**Weak VHDL enforcement.** xsim accepted several constructs that are non-compliant with VHDL-2008. Code that simulated correctly in xsim was actually relying on Vivado-specific tolerances. This matters because synthesis and simulation should agree — if the simulator is lenient, bugs can hide until hardware bring-up.

**Poor automation support.** xsim has no clean CLI interface for scripted regression testing. Every run requires either the Vivado GUI or a TCL script that invokes the full Vivado stack. This makes CI integration painful.

**Slow iteration.** Launching Vivado to run a simulation — even in batch mode — adds significant overhead compared to a lightweight simulator.

---

### Why GHDL

**First-class cocotb support.** GHDL is cocotb's primary supported simulator for VHDL. The integration is stable, documented, and works out of the box.

**Stricter VHDL-2008 parser.** GHDL rejected several constructs that xsim had silently accepted. Each rejection was a real bug:

- `when/else` concurrent signal assignment inside a clocked process — not valid VHDL-93, xsim accepted it, GHDL caught it. Fixed by moving the assignment outside the process as a proper concurrent statement.
- Incompatible array aggregate syntax in a package body — GHDL flagged the type mismatch, xsim did not.
- Signal read before initialization in a combinational process — GHDL warned, xsim was silent.

These weren't compatibility issues — they were bugs. GHDL making them visible was the point.

**Scriptable and CI-ready.** GHDL compiles and simulates from the command line with no GUI dependency. The entire test suite runs with a single `make` command.

**Fast iteration.** Compile + simulate cycle for a unit test is under 2 seconds. This changes how you work — you run tests constantly instead of occasionally.

**Linux-native.** Running under WSL gives a proper Linux environment. No path translation issues, no Windows-specific Vivado licensing quirks during simulation.

---

## What Changed in the RTL

The GHDL migration required fixes in `uart_interface.vhd`:

**`fifo_empty` concurrent assignment** — originally computed inside a clocked process using `when/else`. VHDL-93 does not support `when/else` inside processes. Moved to a concurrent signal assignment outside all processes:
```vhdl
-- WRONG (xsim accepted, GHDL rejected)
process(clk)
begin
    fifo_empty <= '1' when wr_ptr = rd_ptr else '0';  -- not valid here
end process;

-- CORRECT
fifo_empty <= '1' when wr_ptr = rd_ptr else '0';  -- concurrent, outside process
```

**`fifo_full` protection** — added during migration after GHDL's stricter simulation revealed a corner case where the sequencer could write to a full FIFO. xsim had not caught this because the timing in its simulation was slightly different.

These fixes made the RTL more correct, not just more compatible.

---

## Environment Setup

### WSL (Windows Subsystem for Linux)

```bash
# Install WSL2 with Ubuntu 22.04
wsl --install -d Ubuntu-22.04
```

### GHDL

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install ghdl ghdl-mcode

# Verify
ghdl --version
# should show: GHDL 3.x.x
```

### cocotb

```bash
pip install cocotb==1.8.1
pip install cocotb-bus

# Verify
python -c "import cocotb; print(cocotb.__version__)"
```

### Running the test suite

```bash
cd sim/
make          # runs all tests
make WAVES=1  # dumps VCD waveforms to sim_build/
```

### Makefile structure

The `Makefile` compiles all RTL sources with GHDL, then hands control to cocotb. Key variables:

```makefile
TOPLEVEL_LANG = vhdl
TOPLEVEL      = top          # entity name in top_level.vhd
MODULE        = tests.test_system  # default test module
SIM           = ghdl
VHDL_SOURCES  = $(wildcard ../rtl/*.vhd)
```

To add a new RTL file: add it to `VHDL_SOURCES`. To add a new test: add it to `tests/` and run `make MODULE=tests.your_test`.

---

## Vivado (Synthesis Only)

Vivado is still used for synthesis and place-and-route. Simulation has moved to GHDL but the synthesis flow is unchanged:

1. Open Vivado, create project targeting `xc7a35t` (Arty A7-35)
2. Add all sources from `rtl/`
3. Add constraints from `rtl/constraints/arty_a7.xdc`
4. Run synthesis → implementation → generate bitstream

Timing closure at 100 MHz confirmed. No timing violations in the critical path (PI datapath through `fp_mult`).

---

## Summary

| | xsim | GHDL |
|--|------|------|
| cocotb support | Fragile | Reliable |
| VHDL strictness | Lenient | Strict (caught real bugs) |
| CLI automation | Painful | Native |
| Iteration speed | Slow | Fast |
| CI integration | Hard | Easy |
| Synthesis | ✅ (keep Vivado) | N/A |