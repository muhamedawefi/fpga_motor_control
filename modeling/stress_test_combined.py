# =============================================================================
# ULTIMATE STRESS TEST - ALL NON-IDEALITIES COMBINED
# =============================================================================
# Tests controller with:
# - Parameter mismatch (±20%)
# - Load disturbance (0.03 Nm)
# - 1-cycle measurement delay
# - DC bus voltage droop
# - ADC/Encoder/PWM quantization (12-bit, 2000 CPR, 10-bit)
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import ISR_fpga_1khz as ctrl

# =============================================================================
# HARDWARE SPECIFICATIONS (Quantization)
# =============================================================================
ADC_BITS = 12  # 12-bit ADC for current measurement
ADC_RANGE = 10.0  # ±10A range
ENCODER_CPR = 2000  # 2000 counts per revolution (quadrature)
PWM_BITS = 10  # 10-bit PWM resolution
VDC_BUS = 24.0  # 24V DC bus

# Calculate LSBs
ADC_LSB = (2 * ADC_RANGE) / (2 ** ADC_BITS)  # 0.0049 A
PWM_LSB = VDC_BUS / (2 ** PWM_BITS)  # 0.0234 V

print("=" * 70)
print("ULTIMATE STRESS TEST CONFIGURATION")
print("=" * 70)
print(f"Hardware Quantization:")
print(f"  ADC: {ADC_BITS}-bit, ±{ADC_RANGE}A, LSB={ADC_LSB * 1000:.3f}mA")
print(f"  Encoder: {ENCODER_CPR} CPR, {ENCODER_CPR / (2 * np.pi):.1f} counts/rad")
print(f"  PWM: {PWM_BITS}-bit, {VDC_BUS}V, LSB={PWM_LSB * 1000:.3f}mV")


# =============================================================================
# QUANTIZATION FUNCTIONS
# =============================================================================

def quantize_adc_current(i_true):
    """Simulate 12-bit ADC measuring current"""
    i_saturated = np.clip(i_true, -ADC_RANGE, ADC_RANGE)
    counts = round(i_saturated / ADC_LSB)
    return counts * ADC_LSB


def quantize_pwm_voltage(v_cmd):
    """Simulate 10-bit PWM duty cycle quantization"""
    v_saturated = np.clip(v_cmd, -VDC_BUS, VDC_BUS)
    counts = round(v_saturated / PWM_LSB)
    return counts * PWM_LSB


# =============================================================================
# MOTOR PARAMETERS
# =============================================================================
MOTOR_R = 2.5
MOTOR_L = 0.0005
MOTOR_J = 0.0001
MOTOR_B = 0.0001
MOTOR_KT = 0.05
MOTOR_KE = 0.05

# Real parameters (with mismatch) - 15-20% variation
MOTOR_R_REAL = MOTOR_R * 1.15  # +15% resistance (copper heating)
MOTOR_L_REAL = MOTOR_L * 0.90  # -10% inductance (saturation)
MOTOR_J_REAL = MOTOR_J * 1.20  # +20% inertia (load uncertainty)
MOTOR_B_REAL = MOTOR_B * 1.10  # +10% friction

print(f"\nParameter Mismatch:")
print(f"  Resistance:  {MOTOR_R:.4f} Ω → {MOTOR_R_REAL:.4f} Ω ({(MOTOR_R_REAL / MOTOR_R - 1) * 100:+.1f}%)")
print(
    f"  Inductance:  {MOTOR_L * 1e6:.1f} µH → {MOTOR_L_REAL * 1e6:.1f} µH ({(MOTOR_L_REAL / MOTOR_L - 1) * 100:+.1f}%)")
print(
    f"  Inertia:     {MOTOR_J * 1e6:.1f} µkg·m² → {MOTOR_J_REAL * 1e6:.1f} µkg·m² ({(MOTOR_J_REAL / MOTOR_J - 1) * 100:+.1f}%)")
print(
    f"  Friction:    {MOTOR_B * 1e4:.1f}e-4 → {MOTOR_B_REAL * 1e4:.1f}e-4 ({(MOTOR_B_REAL / MOTOR_B - 1) * 100:+.1f}%)")

# =============================================================================
# DISTURBANCE & SIMULATION SETTINGS
# =============================================================================
DISTURBANCE_TIME = 7.0
DISTURBANCE_DURATION = 0.5
DISTURBANCE_VALUE = 0.03

VDC_NOMINAL = 24.0
DT_FAST = 1e-4
SIM_TIME = 30.0
STEPS = int(SIM_TIME / DT_FAST)

print(f"\nDisturbance: {DISTURBANCE_VALUE} Nm at t={DISTURBANCE_TIME}s for {DISTURBANCE_DURATION}s")
print(f"Measurement Delay: 1 control cycle (100 µs)")
print(f"DC Bus Droop: 0.5V per Ampere")
print("=" * 70)


def get_setpoint(t):
    if t < 5.0:
        return 0.0
    elif t < 12.0:
        return 50.0
    elif t < 19.0:
        return 100.0
    elif t < 26.0:
        return 150.0
    else:
        return 0.0


ctrl.reset()

# Logging
t_log = []
w_log = []
w_true_log = []
i_log = []
i_true_log = []
v_log = []
v_actual_log = []
current_ref_log = []
speed_int_log = []
setpoint_log = []
load_log = []
quantization_error_speed = []
quantization_error_current = []

print("\nRunning ULTIMATE stress test...")
print("(Parameter mismatch + Disturbance + Delay + Bus droop + Quantization)")

# Encoder simulation
encoder_position = 0.0
previous_encoder_counts = 0

for n in range(STEPS):
    t = n * DT_FAST

    # 1. Update setpoint
    ctrl.w_ref = get_setpoint(t)

    # 2. Run controller (uses quantized + delayed measurements)
    ctrl.clock_tick()

    # 3. Quantize voltage command (PWM resolution)
    v_quantized = quantize_pwm_voltage(ctrl.v_cmd)

    # 4. Plant simulation
    i_old = ctrl.i_meas
    w_old = ctrl.w_meas

    # Load disturbance
    if DISTURBANCE_TIME <= t < DISTURBANCE_TIME + DISTURBANCE_DURATION:
        load = DISTURBANCE_VALUE
    else:
        load = 0.0

    # DC bus droop
    VDC_effective = VDC_NOMINAL - 0.5 * abs(i_old)
    V_LIMIT = 0.95 * VDC_effective
    v_actual = np.clip(v_quantized, -V_LIMIT, V_LIMIT)

    # Motor equations (with parameter mismatch - using REAL parameters)
    di_dt = (v_actual - MOTOR_R_REAL * i_old - MOTOR_KE * w_old) / MOTOR_L_REAL
    dw_dt = (MOTOR_KT * i_old - MOTOR_B_REAL * w_old - load) / MOTOR_J_REAL

    # TRUE states (before quantization)
    i_true = i_old + di_dt * DT_FAST
    w_true = w_old + dw_dt * DT_FAST

    # Update encoder position
    encoder_position += w_true * DT_FAST

    # 5. QUANTIZE MEASUREMENTS (what controller sees)
    # Current: 12-bit ADC
    i_quantized = quantize_adc_current(i_true)

    # Speed: Encoder (position → count difference → speed)
    encoder_counts = round(encoder_position * ENCODER_CPR / (2 * np.pi))

    # Speed measurement updates at 1kHz (every 10 fast cycles)
    if n % 10 == 0:
        delta_counts = encoder_counts - previous_encoder_counts
        w_quantized = (delta_counts / (ENCODER_CPR / (2 * np.pi))) / (10 * DT_FAST)
        previous_encoder_counts = encoder_counts
    else:
        w_quantized = ctrl.w_meas  # Hold previous value

    # Write quantized measurements to controller
    # (Note: ctrl already has delay built in via i_meas_delayed, w_meas_delayed)
    ctrl.i_meas = i_quantized
    ctrl.w_meas = w_quantized

    # 6. Logging (every 10th sample = 1 kHz)
    if n % 10 == 0:
        t_log.append(t)
        w_log.append(w_quantized)  # Quantized speed (what controller sees)
        w_true_log.append(w_true)  # True speed (actual motor)
        i_log.append(i_quantized)  # Quantized current
        i_true_log.append(i_true)  # True current
        v_log.append(ctrl.v_cmd)  # Commanded voltage
        v_actual_log.append(v_actual)  # Actual voltage (after PWM quantization + saturation)
        current_ref_log.append(ctrl.current_ref)
        speed_int_log.append(ctrl.speed_integral)
        setpoint_log.append(ctrl.w_ref)
        load_log.append(load)
        quantization_error_speed.append(w_true - w_quantized)
        quantization_error_current.append(i_true - i_quantized)

print("Simulation complete!")

# =============================================================================
# ANALYSIS
# =============================================================================
print("\n" + "=" * 70)
print("ULTIMATE STRESS TEST RESULTS")
print("=" * 70)

# Quantization statistics
speed_qerr = np.array(quantization_error_speed)
current_qerr = np.array(quantization_error_current)

print(f"\n1. QUANTIZATION ERRORS:")
print(f"   Speed:")
print(f"     RMS: {np.sqrt(np.mean(speed_qerr ** 2)):.6f} rad/s")
print(f"     Max: {np.max(np.abs(speed_qerr)):.6f} rad/s")
print(f"   Current:")
print(f"     RMS: {np.sqrt(np.mean(current_qerr ** 2)):.6f} A")
print(f"     Max: {np.max(np.abs(current_qerr)):.6f} A")
print(f"     ADC LSB: {ADC_LSB:.6f} A")

# Steady-state performance
print(f"\n2. STEADY-STATE PERFORMANCE:")
segments = [
    (50.0, 10.0, 11.5),
    (100.0, 17.0, 19.0),
    (150.0, 24.0, 26.0)
]

for sp, t_start, t_end in segments:
    speeds_in_window = []
    for i, t in enumerate(t_log):
        if t_start <= t <= t_end:
            speeds_in_window.append(w_true_log[i])

    if speeds_in_window:
        avg_speed = np.mean(speeds_in_window)
        std_speed = np.std(speeds_in_window)
        error = sp - avg_speed
        error_pct = (error / sp) * 100 if sp != 0 else 0
        print(
            f"   {sp:3.0f} rad/s: avg={avg_speed:7.3f}, error={error:+7.4f} ({error_pct:+.3f}%), ripple={std_speed:.4f}")

# Disturbance rejection
print(f"\n3. DISTURBANCE REJECTION:")
dist_start_idx = next(i for i, t in enumerate(t_log) if t >= DISTURBANCE_TIME)
dist_end_idx = next(i for i, t in enumerate(t_log) if t >= DISTURBANCE_TIME + DISTURBANCE_DURATION)

speeds_before = w_true_log[dist_start_idx ]