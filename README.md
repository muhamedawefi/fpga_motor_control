# FPGA-Based DC Motor Control Platform

Cascade PI speed/current controller implemented on an **Artix-7 FPGA** in VHDL, with fixed-point arithmetic, SPI ADC interface, UART telemetry, and a full cocotb verification infrastructure.

This repo documents the full engineering journey — from Python modeling through fixed-point migration, RTL design, toolchain transition, and automated verification.

---

## System Overview

```
Speed reference → [Speed PI @ 1kHz] → [Current PI @ 10kHz] → [PWM] → Motor
                        ↑                      ↑
                   Encoder feedback       ADC feedback (SPI)
                        └──────────── UART telemetry ──────────────→ Dashboard
```

**Hardware:** Digilent Arty A7 (Artix-7 XC7A35T) · 24V DC bus · 12-bit SPI ADC · Quadrature encoder  
**Controller:** Cascade PI · 10kHz current loop · 1kHz speed loop · Q-format fixed-point  
**Verification:** cocotb + GHDL · Unit + subsystem + full-system tests  
**Toolchain:** Vivado (synthesis/constraints) · WSL + GHDL (simulation) · Python (modeling + GUI)

---

## Engineering Evolution

This project went through deliberate architectural stages. Each transition happened for a specific reason.

### Stage 1 — Python Modeling
*`modeling/`*

Before touching any VHDL, the full cascade PI system was modeled in Python:
- DC motor dynamics (electrical + mechanical)
- PID tuning: Ziegler-Nichols attempted first → abandoned due to zero ζ control on second-order systems → switched to model-based pole placement
- Stability analysis: Bode plots, phase margin, bandwidth ratio between loops
- Noise and quantization effects characterized

**Why Python first:** Floating-point iteration is fast. Validate the controller mathematically before paying the cost of fixed-point conversion and HDL simulation.

---

### Stage 2 — ISR Simulation (Float Baseline)
*`modeling/isr_controller.py`*

The controller was restructured to mirror FPGA hardware — global variables as registers, functions as combinational logic, one `clock_tick()` per cycle. All signals annotated `# REG`, `# WIRE`, `# CONST`.

This float baseline became the golden reference for all subsequent fixed-point validation.

**Why this matters:** Most student projects go straight from theory to HDL. Structuring Python as a software ISR forces you to think in clock cycles before writing a single `process` statement.

---

### Stage 3 — Hybrid Fixed-Point
*`modeling/hybrid_fixed_point.py`*

Signals converted to Q-format fixed-point. Gains kept as float for debugging.

- Speed: Q8.8 · Current: Q4.12 · Voltage: Q6.10
- Validated against float baseline: RMS speed error < 0.5 rad/s

**Why hybrid first:** Converting everything at once risks cascading precision errors with no clear diagnosis point. Hybrid isolates whether errors come from signal quantization or gain quantization.

---

### Stage 4 — Full Fixed-Point Migration
*`modeling/compare_three_way.py`*

All gains and arithmetic converted to integer Q-format. Three-way comparison: float vs hybrid vs full fixed-point.

Key decisions made here:
- Scaling factors per signal path
- Saturation and overflow strategy
- Integrator word-length (Q8.16 speed, Q16.8 current)
- Quantization error budget per loop

**Why this comparison matters:** It proves the fixed-point implementation doesn't degrade control performance beyond the quantization noise floor — and it gives exact numbers.

---

### Stage 5 — VHDL RTL Architecture
*`rtl/`*

Full hardware implementation on Artix-7:

| Module | Function |
|--------|----------|
| `pid_controller.vhd` | Fixed-point cascade PI with anti-windup |
| `uart_rx.vhd` / `uart_tx.vhd` | Command input / telemetry output |
| `spi_adc.vhd` | SPI interface to 12-bit ADC |
| `pwm_gen.vhd` | PWM generation with dead-time |
| `encoder.vhd` | Quadrature decoder |
| `top.vhd` | System integration |

Synthesized and constrained in Vivado. Timing closure verified at 100 MHz system clock.

---

### Stage 6 — Toolchain Transition: Vivado xsim → WSL + GHDL
*`sim/`*

Midway through verification, the simulation environment moved from Vivado's built-in xsim to GHDL running under WSL.

**Problems with xsim:**
- Fragile cocotb integration requiring workarounds
- Weak VHDL standards enforcement — silent acceptance of non-compliant code
- Poor automation support, difficult CI integration

**Why GHDL:**
- First-class cocotb support
- Stricter VHDL-2008 parser — caught real RTL bugs xsim missed
- Linux-native, scriptable, CI-ready
- Faster iteration cycle

**Cost:** Several RTL fixes were required because GHDL rejected constructs xsim had silently accepted. Those fixes made the code more correct, not just more compatible.

---

### Stage 7 — Verification Infrastructure (cocotb)
*`sim/`*

Automated test suite with three levels:

| Test | What it covers |
|------|---------------|
| `test_uart.py` | UART framing, baud accuracy, command parsing |
| `test_pid.py` | Fixed-point PI arithmetic, saturation, anti-windup |
| `test_adc.py` | SPI timing, data alignment, error cases |
| `test_system.py` | Full closed-loop: speed step, disturbance rejection, telemetry stream |

Every test is automated — no manual waveform inspection required.

---

### Stage 8 — Telemetry + Dashboard
*`gui/`*

Real-time UART telemetry stream decoded by a Python backend and displayed on a live dashboard: speed, current, voltage command, setpoint, integrator state.

---

## Planned Extensions

These are identified next steps, not completed work:

- **Luenberger observer** — state estimation to replace encoder for sensorless operation (Python prototype exists in `modeling/advanced/`)
- **Kalman filter** — optimal estimation under sensor noise
- **ESP32 integration** — wireless telemetry bridge over BLE/WiFi
- **C++ backend** — replace Python telemetry parser for embedded deployment
- **Friction compensation** — Coulomb + viscous model in feedforward path
- **CAN bus telemetry** — industrial protocol extension

---

## Repository Structure

```
fpga-motor-control/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── modeling/               # Python modeling and validation
│   ├── README.md
│   ├── isr_controller.py
│   ├── hybrid_fixed_point.py
│   ├── tuning_ziegler_nichols.py
│   ├── tuning_pole_placement.py
│   ├── compare_baseline_hybrid.py
│   ├── compare_three_way.py
│   ├── quantization_test.py
│   ├── stress_test_combined.py
│   ├── test_bench.py
│   ├── stability_analysis.py
│   └── advanced/
│       └── motor_sim_observer.py
│
├── rtl/                    # VHDL source
│   ├── pid/
│   ├── uart/
│   ├── spi/
│   ├── pwm/
│   ├── encoder/
│   └── top/
│
├── sim/                    # cocotb testbenches
│   ├── test_uart.py
│   ├── test_pid.py
│   ├── test_adc.py
│   ├── test_system.py
│   └── Makefile
│
├── gui/                    # Telemetry dashboard
│
└── docs/
    ├── architecture.md
    ├── fixed_point.md
    ├── verification.md
    └── toolchain.md
```

---

## Setup

```bash
pip install -r requirements.txt   # Python dependencies
# GHDL + cocotb: see docs/toolchain.md
```

**Requirements:** Python 3.10+ · numpy · matplotlib · scipy · pyserial · GHDL 3.0+ · cocotb 1.8+