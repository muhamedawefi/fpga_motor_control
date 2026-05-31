# Modeling & Simulation

Python-based controller development and validation pipeline. Everything here runs before any VHDL was written.

The goal was to prove the cascade PI controller works mathematically, then progressively convert it to fixed-point arithmetic — validating each step against the previous one.

---

## Workflow

```
Z-N tuning attempt          → fundamental limitation found
      ↓
Pole placement tuning       → gains derived from motor model
      ↓
Float ISR simulation        → golden reference established
      ↓
Hybrid fixed-point          → signal quantization validated
      ↓
Full fixed-point            → integer-only arithmetic validated
      ↓
Quantization + stress test  → hardware non-idealities characterized
      ↓
Stability analysis          → phase margin and bandwidth confirmed
```

---

## Files

### Tuning

| File | Purpose |
|------|---------|
| `tuning_ziegler_nichols.py` | Empirical tuning — attempted first, see note below |
| `tuning_pole_placement.py` | Model-based tuning — final gains used in hardware |

**Why Z-N was abandoned:** Ziegler-Nichols works on first-order + dead-time systems. It gives no control over damping ratio ζ on second-order mechanical plants. For a cascade PI with both an electrical loop (L/R) and a mechanical loop (J/B), the empirical rules produce unpredictable closed-loop poles. Switched to model-based pole placement which takes explicit `bandwidth_hz` and `zeta` per loop.

| Aspect | Ziegler-Nichols | Pole Placement |
|--------|----------------|----------------|
| Damping ratio ζ | Fixed by empirical rules | Explicit parameter |
| Motor model required | No | Yes (R, L, J, Kt) |
| Anti-windup | N/A | Yes — integral clamping |
| Second-order systems | Poor ζ control | Full control |
| Use case | Unknown plant | Known motor, precise requirements |

---

### ISR Simulation

| File | Purpose |
|------|---------|
| `isr_controller.py` | Cascade PI — the float golden reference |
| `hybrid_fixed_point.py` | Same controller, Q-format signals, float gains |
| `compare_baseline_hybrid.py` | Validates hybrid against float: RMS + max error |
| `compare_three_way.py` | Float vs hybrid vs full fixed-point — the key comparison |

`isr_controller.py` is the most important file in this folder. Every variable is annotated `# REG`, `# WIRE`, or `# CONST` to mirror how the logic maps to FPGA hardware. It implements:
- Cascade PI: 10kHz current loop, 1kHz speed loop (10:1 bandwidth ratio)
- First-order IIR filters on speed measurement and speed reference
- Back-calculation anti-windup on both loops
- 1-sample pipeline delay on ADC/encoder measurements

The `compare_three_way.py` output is the core validation result:

```
FULL FP vs BASELINE:
  Speed RMS error:  < 0.5 rad/s     ✅
  Speed MAX error:  < 2.0 rad/s     ✅
→ Fixed-point implementation validated. Ready for HDL.
```

---

### Validation & Stress Testing

| File | Purpose |
|------|---------|
| `quantization_test.py` | ADC (12-bit), encoder (2000 CPR), PWM (10-bit) resolution |
| `stress_test_combined.py` | All non-idealities simultaneously |
| `test_bench.py` | Step response metrics: rise time, overshoot, settling, SS error |
| `stability_analysis.py` | Bode plots, phase margin, bandwidth ratio (scipy) |

`stress_test_combined.py` runs the controller against:
- ±20% parameter mismatch (R, L, J, B)
- 0.03 Nm load disturbance pulse
- 1-cycle measurement delay
- DC bus voltage droop (0.5V/A)
- 12-bit ADC + 2000 CPR encoder + 10-bit PWM quantization

All simultaneously. The controller holds all setpoints (50 / 100 / 150 rad/s) within spec.

`stability_analysis.py` computes:
- Current loop phase margin (target: 45°–70°)
- Speed loop phase margin
- Bandwidth ratio current/speed (target: > 5×)

---

### Advanced

| File | Purpose |
|------|---------|
| `advanced/motor_sim_observer.py` | Luenberger observer prototype + PWM switching model |

This is a forward-looking experiment. It implements a full-order observer that estimates speed from current measurements only — the foundation for sensorless control. Included here as a planned extension, not a finished component.

---

## Q-Format Summary

| Signal | Format | Range | Resolution |
|--------|--------|-------|------------|
| Speed (w) | Q8.8 | ±255 rad/s | 0.0039 rad/s |
| Current (i) | Q4.12 | ±15 A | 0.00024 A |
| Voltage (v) | Q6.10 | ±31 V | 0.001 V |
| Speed integrator | Q8.16 | ±255 | 1.5e-5 |
| Current integrator | Q16.8 | ±32767 | 0.0039 |

---

## Dependencies

```bash
pip install numpy matplotlib scipy
```