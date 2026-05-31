# Fixed-Point Strategy

## Why Fixed-Point

FPGA fabric has no native floating-point units. Implementing IEEE 754 float in LUTs is expensive — it costs hundreds of LUTs per operation and introduces multi-cycle latency. For a real-time control loop running at 10kHz, this is unacceptable.

Fixed-point arithmetic maps directly to DSP48 blocks and basic LUT logic. Multiplications are single-cycle, additions are combinational. The cost is that you have to track precision manually.

---

## Migration Path

The controller went through three implementation stages before RTL:

### Stage 1 — Float baseline (`isr_controller.py`)
Full cascade PI in Python with IEEE 754 doubles. This is the golden reference — any fixed-point implementation is validated against this.

### Stage 2 — Hybrid fixed-point (`hybrid_fixed_point.py`)
Signals converted to Q-format. Gains kept as float. This isolates whether errors come from signal quantization (the Q-format choices) or gain quantization (the integer representation of Kp, Ki).

Result: RMS speed error vs float baseline < 0.1 rad/s. Signal quantization is acceptable.

### Stage 3 — Full fixed-point (`full_fixed_point.py`)
All gains and arithmetic converted to integer Q-format. Three-way comparison run: float vs hybrid vs full fixed.

Result:
```
FULL FP vs BASELINE:
  Speed RMS error:  < 0.5 rad/s   ✅
  Speed MAX error:  < 2.0 rad/s   ✅
→ Ready for HDL implementation
```

---

## Q-Format Choices

Q-format notation: **Qm.n** means m integer bits, n fractional bits, total m+n+1 bits (sign included).

### Signal formats

| Signal | Format | Range | Resolution | Rationale |
|--------|--------|-------|------------|-----------|
| Speed (ω) | Q9.7 | ±511 rad/s | 0.0078 rad/s | Max speed ~150 rad/s, needs headroom |
| Current (i) | Q4.12 | ±15 A | 0.00024 A | Max current 5A, ADC noise floor ~1mA |
| Voltage (v_cmd) | Q6.10 | ±31 V | 0.001 V | Bus voltage 24V, PWM resolution ~23mV |
| Speed integrator | Q9.15 | ±511 | 3e-5 | Needs wider fractional for Ki×Ts product |
| Current integrator | Q16.8 | ±32767 | 0.004 | Ki is large (15707), needs integer headroom |

### Gain formats

| Gain | Value | Format | Raw integer | Verification |
|------|-------|--------|-------------|--------------|
| Speed Kp | 0.12366 | Q0.16 | 8101 | 8101/65536 = 0.12361 ✓ |
| Speed Ki | 7.8957 | Q4.12 | 32334 | 32334/4096 = 7.8940 ✓ |
| Speed Ts | 0.001 | Q0.20 | 1049 | 1049/1048576 = 0.001001 ✓ |
| Current Kp | 3.1416 | Q2.14 | 51472 | 51472/16384 = 3.1416 ✓ |
| Current Ki | 15707.96 | Q14.2 | 62832 | 62832/4 = 15708 ✓ |
| Current Ts | 0.0001 | Q0.20 | 105 | 105/1048576 = 1.001e-4 ✓ |

All raw integers are defined in `motor_constants_pkg.vhd` and verified against the Python pole placement results.

---

## Arithmetic Strategy

### Multiplication

All multiplications go through `fp_mult` in `fixed_point_pkg.vhd`:

```vhdl
function fp_mult(a, b : signed; a_frac, b_frac, out_frac : integer) return signed
```

The function:
1. Computes full-precision product: `product = a × b` (width = a_len + b_len)
2. Determines right-shift: `shift = a_frac + b_frac - out_frac`
3. Adds half-LSB before shifting: `product + 2^(shift-1)` — this is **rounding**, not truncation
4. Shifts right to target precision

**Why rounding matters:** Truncation introduces a consistent negative bias. Over many PI iterations this bias accumulates in the integrator, causing a small but measurable steady-state error. Rounding is unbiased.

### The boundary rule

> Never shrink precision inside math functions. Only shrink at system boundaries.

This is enforced in the code with a comment in `fixed_point_pkg.vhd`. The PI datapath keeps full intermediate precision (32-bit products) and only truncates to 16-bit at the output register. This prevents error accumulation across multiply-accumulate chains.

### Saturation

Saturation is explicit and centralized in `saturate()`:
```vhdl
function saturate(value, min_val, max_val : signed) return signed
```

Used at two points only:
- PI output (before anti-windup check)
- Final `output_cmd` assignment

Never inside intermediate arithmetic.

---

## Anti-Windup

Back-calculation method. When the output saturates:

```
correction = TT_INV × (output_sat - output_unsat)
integral_next = integral_next + correction × Ts
```

The key implementation detail: the correction is applied to `integral_next` (the variable, not yet committed to the register) **before** the `integral <= integral_next` register write. This means the corrected value is what gets stored — not a one-cycle-delayed correction.

This is correct. Many implementations get this wrong by applying the correction to the registered integral, which introduces a one-cycle lag in the anti-windup response.

---

## Quantization Effects

Characterized in `modeling/quantization_test.py` with realistic hardware specs:

| Source | Specification | Quantization error |
|--------|--------------|-------------------|
| Current ADC | 12-bit, ±10A | LSB = 4.88 mA, RMS error < 3 mA |
| Speed encoder | 2000 CPR quadrature | At 50 rad/s: LSB ≈ 0.016 rad/s |
| PWM output | 10-bit, 24V bus | LSB = 23.4 mV |

Controller maintains all setpoints (50 / 100 / 150 rad/s) within spec under combined quantization + parameter mismatch (±20%) + load disturbance. See `modeling/stress_test_combined.py` for the full validation run.

---

## Overflow Analysis

### Speed integrator (Q9.15)

Maximum integrator growth per step: `Ki × Ts × error_max = 7.896 × 0.001 × 511 = 4.03`

At Q9.15, integer range is ±16384. Saturation kicks in at ±5.0 A (current reference limit) long before the integrator can overflow. Safe.

### Current integrator (Q16.8)

Maximum growth per step: `Ki × Ts × error_max = 15708 × 0.0001 × 15 = 23.6`

At Q16.8, integer range is ±8,388,608. Saturation at ±24V kicks in at output, anti-windup prevents runaway. The wide integer range (16 bits) was chosen specifically because Ki is large. Safe.

### Multiplication intermediate width

`fp_mult` produces a full-width product (32-bit inputs → 64-bit intermediate). The `resize` before storing back to a 32-bit signal is safe because the shift brings the value back into range before truncation.