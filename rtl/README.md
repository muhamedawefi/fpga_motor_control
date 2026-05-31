# RTL Source

VHDL implementation targeting Artix-7 (XC7A35T) at 100 MHz system clock.

---

## Module Overview

| File | Entity | Function |
|------|--------|---------|
| `motor_constants_pkg.vhd` | — | Q-format constants, PI gains, saturation limits |
| `fixed_point_pkg.vhd` | — | `fp_mult` with rounding, `saturate` function |
| `clock_divider.vhd` | `clock_divider` | Generates 10kHz and 1kHz enable pulses from 100MHz |
| `pi_controller.vhd` | `pi_controller_17bit` | Parameterized fixed-point PI with anti-windup |
| `uart_interface.vhd` | `uart_controller` | Full UART stack: RX/TX, frame parser, telemetry sequencer |
| `register_map.vhd` | `register_map` | Address-mapped register file, UART↔control system bridge |
| `adc_interface.vhd` | `adc_interface` | SPI ADC driver, sim/hardware mux, Q4.12 output |
| `encoder_interface.vhd` | `encoder_interface` | Quadrature decoder, speed computation, Q9.7 output |
| `top_level.vhd` | `top` | System integration, sim/hardware mux via `SIM_MODE` generic |

---

## Architecture

```
UART RX ──→ Frame Parser ──→ Register Map ──→ Speed Setpoint
                                                     │
                                               Ref Pre-filter
                                                     │
Encoder ──→ Speed FB ──────────────────────→ Speed PI (1kHz)
                                                     │
                                               Current Ref
                                                     │
SPI ADC ──→ Current FB ─────────────────────→ Current PI (10kHz)
                                                     │
                                               Voltage Cmd
                                                     │
                                               PWM Generator ──→ Motor
                                                     │
UART TX ←── Telemetry Sequencer ←─────────── Register Map
```

---

## Key Design Decisions

**Single parameterized PI entity.** `pi_controller_17bit` is instantiated twice — once for the speed loop, once for the current loop — with different generics. No duplicated logic.

**`SIM_MODE` generic in top-level.** When `true`, the design accepts direct port inputs for speed and current feedback instead of running the encoder and ADC state machines. This allows cocotb testbenches to drive the plant model in Python without simulating SPI/quadrature timing.

**Pre-trigger in clock divider.** `enable_pre_10k` fires at cycle 9000 (of 10000) to give the SPI ADC conversion time to complete before the current loop enable arrives at cycle 10000.

**Fixed-point packages.** All Q-format arithmetic goes through `fp_mult` in `fixed_point_pkg`, which handles rounding correctly (add half-LSB before shift). Saturation is explicit and centralized. See [`docs/fixed_point.md`](../docs/fixed_point.md) for the full Q-format strategy.

---

## Synthesis Results (Vivado, XC7A35T)

| Resource | Used | Available | % |
|----------|------|-----------|---|
| LUT | — | 20800 | — |
| FF | — | 41600 | — |
| BRAM | — | 50 | — |

*Fill in after final synthesis run.*

Timing: 100 MHz, no violations. Critical path: `fp_mult` in PI datapath.

---

## Simulation

RTL is verified using cocotb + GHDL. See [`sim/`](../sim/README.md) for test details.

Vivado is used for synthesis only — not simulation.