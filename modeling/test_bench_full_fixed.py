# =============================================================================
# TEST_FULL_FIXED_POINT.py
# =============================================================================
# Validates full fixed-point ISR module
# Tests: overflow, range, conversions, simulation
# =============================================================================

import numpy as np
import matplotlib.pyplot as plt
import full_fixed as fp

print("=" * 70)
print("FULL FIXED-POINT VALIDATION TEST")
print("=" * 70)

# =============================================================================
# TEST 1: Q-FORMAT RANGE VALIDATION
# =============================================================================
print("\n" + "=" * 70)
print("TEST 1: Q-FORMAT RANGE VALIDATION")
print("=" * 70)

tests_passed = 0
tests_failed = 0

# Q8.8 Speed
print("\n📊 Q8.8 Speed Format:")
print(f"   Scale: {fp.SPEED_SCALE}")
print(f"   LSB: {1 / fp.SPEED_SCALE:.6f} rad/s")
speed_max = 32767 / fp.SPEED_SCALE
speed_min = -32768 / fp.SPEED_SCALE
print(f"   Range: {speed_min:.3f} to {speed_max:.3f} rad/s")

# Test overflow at 150 rad/s
test_speed = 150.0
speed_raw = int(test_speed * fp.SPEED_SCALE)
print(f"\n   🔍 Testing {test_speed} rad/s:")
print(f"      Raw value: {speed_raw}")
print(f"      Max raw: 32767")
if speed_raw > 32767:
    print(f"      ❌ OVERFLOW! {speed_raw} > 32767")
    print(f"      ⚠️  Need Q9.7 or higher for 150 rad/s")
    tests_failed += 1
else:
    print(f"      ✅ OK")
    tests_passed += 1

# Q4.12 Current
print("\n📊 Q4.12 Current Format:")
print(f"   Scale: {fp.CURRENT_SCALE}")
print(f"   LSB: {1 / fp.CURRENT_SCALE:.6f} A = {1 / fp.CURRENT_SCALE * 1000:.3f} mA")
current_max = 32767 / fp.CURRENT_SCALE
current_min = -32768 / fp.CURRENT_SCALE
print(f"   Range: {current_min:.3f} to {current_max:.3f} A")

test_current = 5.0  # Max expected
current_raw = int(test_current * fp.CURRENT_SCALE)
print(f"\n   🔍 Testing {test_current} A:")
print(f"      Raw value: {current_raw}")
if abs(current_raw) > 32767:
    print(f"      ❌ OVERFLOW!")
    tests_failed += 1
else:
    print(f"      ✅ OK ({current_raw}/{32767} used)")
    tests_passed += 1

# Q6.10 Voltage
print("\n📊 Q6.10 Voltage Format:")
print(f"   Scale: {fp.VOLTAGE_SCALE}")
print(f"   LSB: {1 / fp.VOLTAGE_SCALE:.6f} V = {1 / fp.VOLTAGE_SCALE * 1000:.3f} mV")
voltage_max = 32767 / fp.VOLTAGE_SCALE
voltage_min = -32768 / fp.VOLTAGE_SCALE
print(f"   Range: {voltage_min:.3f} to {voltage_max:.3f} V")

test_voltage = 24.0  # Max expected
voltage_raw = int(test_voltage * fp.VOLTAGE_SCALE)
print(f"\n   🔍 Testing {test_voltage} V:")
print(f"      Raw value: {voltage_raw}")
if abs(voltage_raw) > 32767:
    print(f"      ❌ OVERFLOW!")
    tests_failed += 1
else:
    print(f"      ✅ OK ({voltage_raw}/{32767} used)")
    tests_passed += 1

# Q8.16 Speed Integrator
print("\n📊 Q8.16 Speed Integrator Format:")
print(f"   Scale: {fp.SPEED_INT_SCALE}")
print(f"   LSB: {1 / fp.SPEED_INT_SCALE:.8f}")
speed_int_max = (2 ** 23 - 1) / fp.SPEED_INT_SCALE
print(f"   Range: ±{speed_int_max:.3f}")

# Q16.8 Current Integrator
print("\n📊 Q16.8 Current Integrator Format:")
print(f"   Scale: {fp.CURRENT_INT_SCALE}")
print(f"   LSB: {1 / fp.CURRENT_INT_SCALE:.6f}")
current_int_max = (2 ** 23 - 1) / fp.CURRENT_INT_SCALE
print(f"   Range: ±{current_int_max:.3f}")

# =============================================================================
# TEST 2: FIXED-POINT MULTIPLY FUNCTION
# =============================================================================
print("\n" + "=" * 70)
print("TEST 2: MULTIPLY FUNCTION VALIDATION")
print("=" * 70)

# Test multiply_fixed
a_float = 0.5
b_float = 2.0
expected = a_float * b_float

# Q8.8 × Q8.8 → Q8.8
a_raw = int(a_float * 256)
b_raw = int(b_float * 256)
result_raw = fp.multiply_fixed(a_raw, b_raw, 8, 8, 8)
result_float = result_raw / 256

print(f"\n🔢 Test: {a_float} × {b_float} = {expected}")
print(f"   a_raw = {a_raw} (Q8.8)")
print(f"   b_raw = {b_raw} (Q8.8)")
print(f"   result_raw = {result_raw}")
print(f"   result_float = {result_float}")
print(f"   expected = {expected}")
print(f"   error = {abs(result_float - expected):.6f}")

if abs(result_float - expected) < 0.01:
    print(f"   ✅ PASS")
    tests_passed += 1
else:
    print(f"   ❌ FAIL")
    tests_failed += 1

# =============================================================================
# TEST 3: FULL SIMULATION
# =============================================================================
print("\n" + "=" * 70)
print("TEST 3: FULL SIMULATION")
print("=" * 70)

DT_FAST = 1e-4
DURATION = 5.0  # Shorter test
STEPS = int(DURATION / DT_FAST)

# Motor parameters (with mismatch)
MOTOR_R = 2.5
MOTOR_L = 0.0005
MOTOR_J = 0.0001
MOTOR_B = 0.0001
MOTOR_KT = 0.05
MOTOR_KE = 0.05

MOTOR_R_REAL = MOTOR_R * 1.15
MOTOR_L_REAL = MOTOR_L * 0.90
MOTOR_J_REAL = MOTOR_J * 1.20
MOTOR_B_REAL = MOTOR_B * 1.10

# Logging
t_log = []
w_log = []
i_log = []
v_log = []
w_ref_log = []
overflow_detected = []

# Plant state
w_plant = 0.0
i_plant = 0.0

# Reset
fp.reset()

print(f"\nRunning {DURATION}s simulation...")

for n in range(STEPS):
    t = n * DT_FAST

    # Setpoint trajectory
    if t < 1.0:
        ref = 0.0
    elif t < 2.5:
        ref = 50.0
    else:
        ref = 100.0  # Test up to 100 rad/s (safe for Q8.8)

    # Convert setpoint to fixed-point
    fp.w_ref_raw = int(ref * fp.SPEED_SCALE)

    # Check for overflow
    if abs(fp.w_ref_raw) > 32767:
        overflow_detected.append(('w_ref', t, ref, fp.w_ref_raw))

    # Run controller
    fp.clock_tick()

    # Get voltage (convert to float)
    v_actual = fp.v_cmd_raw / fp.VOLTAGE_SCALE

    # Plant simulation (float)
    load = 0.0

    di_dt = (v_actual - MOTOR_R_REAL * i_plant - MOTOR_KE * w_plant) / MOTOR_L_REAL
    dw_dt = (MOTOR_KT * i_plant - MOTOR_B_REAL * w_plant - load) / MOTOR_J_REAL

    i_plant += di_dt * DT_FAST
    w_plant += dw_dt * DT_FAST

    # Convert measurements to fixed-point
    fp.w_meas_raw = int(w_plant * fp.SPEED_SCALE)
    fp.i_meas_raw = int(i_plant * fp.CURRENT_SCALE)

    # Saturate to 16-bit range
    fp.w_meas_raw = fp.saturate(fp.w_meas_raw, -32768, 32767)
    fp.i_meas_raw = fp.saturate(fp.i_meas_raw, -32768, 32767)

    # Check for overflow in internal registers
    if abs(fp.speed_integral_raw) > (2 ** 23 - 1):
        overflow_detected.append(('speed_integral', t, None, fp.speed_integral_raw))
    if abs(fp.current_integral_raw) > (2 ** 23 - 1):
        overflow_detected.append(('current_integral', t, None, fp.current_integral_raw))

    # Log every 10th sample
    if n % 10 == 0:
        t_log.append(t)
        w_log.append(fp.w_meas_raw / fp.SPEED_SCALE)
        i_log.append(fp.i_meas_raw / fp.CURRENT_SCALE)
        v_log.append(fp.v_cmd_raw / fp.VOLTAGE_SCALE)
        w_ref_log.append(fp.w_ref_raw / fp.SPEED_SCALE)

print(f"✅ Simulation complete!")

# =============================================================================
# TEST 4: OVERFLOW DETECTION
# =============================================================================
print("\n" + "=" * 70)
print("TEST 4: OVERFLOW DETECTION")
print("=" * 70)

if overflow_detected:
    print(f"\n❌ OVERFLOWS DETECTED: {len(overflow_detected)}")
    for signal, time, value, raw in overflow_detected[:5]:  # Show first 5
        print(f"   {signal} at t={time:.3f}s: raw={raw}")
    tests_failed += 1
else:
    print(f"\n✅ NO OVERFLOWS DETECTED")
    tests_passed += 1

# =============================================================================
# TEST 5: STEADY-STATE PERFORMANCE
# =============================================================================
print("\n" + "=" * 70)
print("TEST 5: STEADY-STATE PERFORMANCE")
print("=" * 70)

# Check last 1 second at 100 rad/s
steady_start = int(len(t_log) * 0.8)  # Last 20%
w_steady = np.array(w_log[steady_start:])
w_ref_steady = np.array(w_ref_log[steady_start:])

if len(w_steady) > 0:
    error_steady = np.abs(w_steady - w_ref_steady)
    error_rms = np.sqrt(np.mean(error_steady ** 2))
    error_max = np.max(error_steady)

    print(f"\n📊 Steady-state (last 20% of simulation):")
    print(f"   Reference: {w_ref_steady[0]:.1f} rad/s")
    print(f"   RMS error: {error_rms:.4f} rad/s")
    print(f"   Max error: {error_max:.4f} rad/s")

    if error_rms < 1.0:
        print(f"   ✅ PASS (RMS < 1.0 rad/s)")
        tests_passed += 1
    else:
        print(f"   ❌ FAIL (RMS too high)")
        tests_failed += 1

# =============================================================================
# TEST 6: QUANTIZATION CHECK
# =============================================================================
print("\n" + "=" * 70)
print("TEST 6: QUANTIZATION EFFECTS")
print("=" * 70)

# Check for limit cycles at low speed
low_speed_start = int(len(t_log) * 0.3)  # Middle section at 50 rad/s
low_speed_end = int(len(t_log) * 0.5)
w_low = np.array(w_log[low_speed_start:low_speed_end])

if len(w_low) > 10:
    # Calculate variation (should be small, not oscillating)
    w_diff = np.diff(w_low)
    oscillation = np.std(w_diff)

    print(f"\n📊 Low-speed quantization (50 rad/s):")
    print(f"   Speed variation std: {oscillation:.4f} rad/s")

    if oscillation < 0.5:
        print(f"   ✅ PASS (no significant limit cycles)")
        tests_passed += 1
    else:
        print(f"   ⚠️  WARNING: possible limit cycles")
        tests_failed += 1

# =============================================================================
# RESULTS SUMMARY
# =============================================================================
print("\n" + "=" * 70)
print("VALIDATION SUMMARY")
print("=" * 70)

total_tests = tests_passed + tests_failed
print(f"\n📊 Test Results:")
print(f"   Passed: {tests_passed}/{total_tests}")
print(f"   Failed: {tests_failed}/{total_tests}")

if tests_failed == 0:
    print(f"\n✅ ALL TESTS PASSED!")
    print(f"🎉 Full fixed-point implementation is VALIDATED!")
    print(f"✅ Ready for HDL conversion!")
else:
    print(f"\n⚠️  {tests_failed} TESTS FAILED")
    print(f"🔍 Review Q-format choices and fix overflows")

# =============================================================================
# PLOT RESULTS
# =============================================================================
print(f"\nGenerating plots...")

fig, axes = plt.subplots(3, 1, figsize=(12, 10))

# Speed
ax = axes[0]
ax.plot(t_log, w_log, 'b-', label='Speed (fixed-point)', linewidth=2)
ax.plot(t_log, w_ref_log, 'r--', label='Reference', linewidth=1.5)
ax.set_ylabel('Speed (rad/s)')
ax.set_title('Full Fixed-Point Simulation (Integer Arithmetic)')
ax.legend()
ax.grid(True, alpha=0.3)

# Current
ax = axes[1]
ax.plot(t_log, i_log, 'g-', label='Current', linewidth=2)
ax.set_ylabel('Current (A)')
ax.legend()
ax.grid(True, alpha=0.3)

# Voltage
ax = axes[2]
ax.plot(t_log, v_log, 'r-', label='Voltage', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Voltage (V)')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('full_fixed_point_test.png', dpi=150)
print(f"✅ Plot saved: full_fixed_point_test.png")
plt.show()

print("\n" + "=" * 70)
print("TEST COMPLETE")
print("=" * 70)