# Verification Infrastructure

Automated test suite for the FPGA motor control RTL, built on **cocotb + GHDL**.

All tests run without Vivado. No manual waveform inspection required — every test passes or fails with a clear reason.

---

## Toolchain

| Tool | Version | Role |
|------|---------|------|
| GHDL | 3.0+ | VHDL simulator |
| cocotb | 1.8+ | Python test framework |
| Python | 3.10+ | Test logic |

Install GHDL and cocotb: see [`docs/toolchain.md`](../docs/toolchain.md)

---

## Running Tests

```bash
# Run all tests
make

# Run a specific test
make TEST=test_uart

# Run with waveform dump (VCD)
make WAVES=1
```

---

## Folder Structure

```
sim/
├── Makefile              # GHDL compile + cocotb runner
├── conftest.py           # pytest configuration
├── runner.py             # cocotb test runner entry point
│
├── env/
│   ├── clock_reset.py    # Clock generation + reset sequencing utilities
│   └── dut_wrapper.py    # DUT abstraction layer
│
├── drivers/
│   ├── uart_driver.py    # UART frame TX/RX driver
│   ├── uart_model.py     # UART protocol model
│   ├── spi_model.py      # SPI ADC model
│   └── signal_map.py     # DUT signal name mapping
│
└── tests/
    ├── test_uart.py        # UART framing, baud, checksum, commands
    ├── test_controller.py  # PI arithmetic, saturation, anti-windup
    ├── test_system.py      # Full closed-loop system test
    ├── test_app_bridge.py  # GUI↔backend↔FPGA integration
    └── sim_server.py       # Simulation server for backend testing
```

---

## Test Coverage

| Module | Test file | Coverage |
|--------|-----------|---------|
| `uart_interface.vhd` | `test_uart.py` | ✅ Framing, baud accuracy, XOR checksum, all commands (WRITE/READ/STREAM/STOP) |
| `pi_controller.vhd` | `test_controller.py` | ✅ Fixed-point arithmetic, saturation, anti-windup back-calculation |
| Full system | `test_system.py` | ✅ Speed step response, disturbance rejection, telemetry stream |
| GUI integration | `test_app_bridge.py` | ✅ Backend↔simulation bridge |
| `adc_interface.vhd` | — | 🔲 Planned |
| `encoder_interface.vhd` | — | 🔲 Planned |

---

## Test Descriptions

### `test_uart.py`
Verifies the complete UART protocol stack:
- Frame structure: `[0xAA][CMD][ADDR][DATA_H][DATA_L][XOR]`
- All four commands: WRITE (0x01), READ (0x02), STREAM (0x03), STOP (0x04)
- XOR checksum validation — corrupt frames are silently dropped
- Baud rate accuracy at 115200
- Read response format: `[0x55][ADDR][DATA_H][DATA_L][XOR]`
- Telemetry frame format: `[0xFF][SPD_H][SPD_L][CUR_H][CUR_L][VCMD_H][VCMD_L][XOR]`

### `test_controller.py`
Verifies the fixed-point PI controller at the RTL level:
- Q-format arithmetic correctness against Python golden model
- Saturation at `U_MIN` and `U_MAX`
- Anti-windup back-calculation — integrator does not wind up during saturation
- Both speed loop (Q9.7 error, Q0.16 Kp) and current loop (Q4.12 error, Q2.14 Kp) configurations

### `test_system.py`
Full closed-loop test — the most important test in the suite:
- Speed reference written via UART WRITE command
- Plant simulated in Python (motor dynamics, Euler integration)
- Controller runs in RTL (GHDL), plant runs in Python — co-simulation
- Verifies: step response settles, current stays within limits, telemetry stream matches internal signals

### `test_app_bridge.py`
Integration test between the Python backend and the simulation:
- Backend connects to `sim_server.py` instead of real UART
- Verifies the full data path: GUI command → backend → UART frame → RTL → telemetry → backend → GUI

---

## Why GHDL over Vivado xsim

Short answer: cocotb works reliably with GHDL. With xsim it didn't.

Full explanation: [`docs/toolchain.md`](../docs/toolchain.md)