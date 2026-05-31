# =============================================================================
# TESTBENCH – Multi-setpoint validation WITH STRESS TESTS
# This testbench simulates the ENVIRONMENT around the controller
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import ISR_fpga_1khz as ctrl  # Updated ISR module

# =============================================================================
# DISTURBANCE PARAMETERS (Test Configuration - not part of controller)
# =============================================================================
LOAD_TORQUE = 0.0  # Nominal (no load)
DISTURBANCE_TIME = 7.0  # Apply disturbance at t=7s (during 50 rad/s plateau)
DISTURBANCE_DURATION = 0.5  # Duration of disturbance pulse
DISTURBANCE_VALUE = 0.03  # 0.03 Nm load torque

# =============================================================================
# MOTOR PARAMETERS (Controller Design Values)
# These are what the CONTROLLER thinks the motor is
# =============================================================================
MOTOR_R = 2.5  # Resistance (Ω)
MOTOR_L = 0.0005  # Inductance (H)
MOTOR_J = 0.0001  # Inertia (kg·m²)
MOTOR_B = 0.0001  # Viscous friction (N·m·s/rad)
MOTOR_KT = 0.05  # Torque constant (N·m/A)
MOTOR_KE = 0.05  # Back-EMF constant (V·s/rad)

# =============================================================================
# REAL MOTOR PARAMETERS (Plant Mismatch - 15-20% variation)
# These are the ACTUAL motor parameters (what the physics simulation uses)
# In real life: controller model ≠ actual plant
# =============================================================================
MOTOR_R_REAL = MOTOR_R * 1.15  # +15% resistance (copper heating)
MOTOR_L_REAL = MOTOR_L * 0.90  # -10% inductance (magnetic saturation)
MOTOR_J_REAL = MOTOR_J * 1.20  # +20% inertia (load uncertainty)
MOTOR_B_REAL = MOTOR_B * 1.10  # +10% friction (wear, temperature)

print("=" * 70)
print("STRESS TEST CONFIGURATION")
print("=" * 70)
print(f"Parameter Mismatch:")
print(
    f"  Resistance:  {MOTOR_R:.4f} Ω (design) → {MOTOR_R_REAL:.4f} Ω (actual) [{(MOTOR_R_REAL / MOTOR_R - 1) * 100:+.1f}%]")
print(
    f"  Inductance:  {MOTOR_L * 1e6:.1f} µH (design) → {MOTOR_L_REAL * 1e6:.1f} µH (actual) [{(MOTOR_L_REAL / MOTOR_L - 1) * 100:+.1f}%]")
print(
    f"  Inertia:     {MOTOR_J * 1e6:.1f} µkg·m² (design) → {MOTOR_J_REAL * 1e6:.1f} µkg·m² (actual) [{(MOTOR_J_REAL / MOTOR_J - 1) * 100:+.1f}%]")
print(
    f"  Friction:    {MOTOR_B * 1e4:.1f}e-4 (design) → {MOTOR_B_REAL * 1e4:.1f}e-4 (actual) [{(MOTOR_B_REAL / MOTOR_B - 1) * 100:+.1f}%]")
print(f"\nDisturbance:")
print(f"  Time: {DISTURBANCE_TIME}s")
print(f"  Magnitude: {DISTURBANCE_VALUE} Nm")
print(f"  Duration: {DISTURBANCE_DURATION}s")
print(f"\nMeasurement Delay: 1 control cycle (100 µs)")
print(f"DC Bus Droop: 0.5V per Ampere")
print("=" * 70)

# =============================================================================
# DC BUS PARAMETERS (Power supply characteristics)
# =============================================================================
VDC_NOMINAL = 24.0  # Nominal DC bus voltage [V]

# =============================================================================
# SIMULATION SETTINGS
# =============================================================================
DT_FAST = 1e-4  # 10 kHz timestep (matches controller clock)
SIM_TIME = 30.0  # 30 seconds total simulation
STEPS = int(SIM_TIME / DT_FAST)  # Number of simulation steps


# =============================================================================
# TRAJECTORY GENERATOR
# In FPGA: This would be a separate state machine or lookup table
# =============================================================================
def get_setpoint(t):
    """
    Generate reference speed [rad/s] based on simulation time.
    This is external to the controller - represents user commands.

    Args:
        t: WIRE (current time, input)
    Returns:
        WIRE (reference speed)
    """
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


# =============================================================================
# RESET AND INITIALIZE
# =============================================================================
ctrl.reset()  # Reset all controller REGISTERS to zero

# Logging arrays (NOT part of hardware - just for analysis)
t_log = []
w_log = []
i_log = []
v_log = []
v_actual_log = []  # Voltage after saturation
current_ref_log = []
speed_int_log = []
setpoint_log = []
load_log = []

# =============================================================================
# SIMULATION LOOP
# Each iteration = one clock cycle in the FPGA (100 µs @ 10 kHz)
# =============================================================================
print("\nRunning simulation...")

for n in range(STEPS):
    t = n * DT_FAST  # WIRE: current simulation time

    # =========================================================================
    # 1. UPDATE SETPOINT (Trajectory Generator)
    # In FPGA: External module writes to w_ref register
    # =========================================================================
    ctrl.w_ref = get_setpoint(t)  # REG WRITE: update setpoint register

    # =========================================================================
    # 2. RUN CONTROLLER (One Clock Tick)
    # This represents ONE CLOCK CYCLE in the FPGA
    # - Reads from input registers (w_meas, i_meas, w_ref)
    # - Executes combinational logic
    # - Writes to output register (v_cmd)
    # =========================================================================
    ctrl.clock_tick()

    # =========================================================================
    # 3. PLANT SIMULATION (Motor Physics)
    # In real hardware: This is the ACTUAL MOTOR responding to voltage
    # We simulate it here to test the controller
    # =========================================================================

    # Save current state (these are the "old" values for Euler integration)
    i_old = ctrl.i_meas  # WIRE: current value before update
    w_old = ctrl.w_meas  # WIRE: speed value before update

    # -------------------------------------------------------------------------
    # A. LOAD DISTURBANCE (External torque applied to motor shaft)
    # -------------------------------------------------------------------------
    if DISTURBANCE_TIME <= t < DISTURBANCE_TIME + DISTURBANCE_DURATION:
        load = DISTURBANCE_VALUE  # WIRE: disturbance torque
    else:
        load = LOAD_TORQUE  # WIRE: nominal load

    # -------------------------------------------------------------------------
    # B. DC BUS DROOP (Power supply voltage sag under load)
    # Real power supplies have internal resistance
    # -------------------------------------------------------------------------
    VDC_effective = VDC_NOMINAL - 0.5 * abs(i_old)  # WIRE: effective bus voltage
    V_LIMIT = 0.95 * VDC_effective  # WIRE: PWM saturation limit (95% duty max)

    # Apply voltage saturation (PWM can't exceed bus voltage)
    v_actual = max(min(ctrl.v_cmd, V_LIMIT), -V_LIMIT)  # WIRE: actual applied voltage

    # -------------------------------------------------------------------------
    # C. ELECTRICAL DYNAMICS (with parameter mismatch)
    # Motor equation: L*di/dt = V - R*i - Ke*ω
    # Uses REAL parameters (not what controller thinks)
    # -------------------------------------------------------------------------
    di_dt = (v_actual - MOTOR_R_REAL * i_old - MOTOR_KE * w_old) / MOTOR_L_REAL  # WIRE: current derivative

    # -------------------------------------------------------------------------
    # D. MECHANICAL DYNAMICS (with mismatch + load)
    # Motor equation: J*dω/dt = Kt*i - B*ω - T_load
    # Uses REAL parameters (not what controller thinks)
    # -------------------------------------------------------------------------
    dw_dt = (MOTOR_KT * i_old - MOTOR_B_REAL * w_old - load) / MOTOR_J_REAL  # WIRE: speed derivative

    # -------------------------------------------------------------------------
    # E. UPDATE PLANT STATES (Euler integration)
    # In real hardware: This is the actual motor physics responding
    # We write back to the controller's measurement registers
    # (simulating ADC and encoder reading the real values)
    # -------------------------------------------------------------------------
    ctrl.i_meas = i_old + di_dt * DT_FAST  # REG WRITE: update current measurement
    ctrl.w_meas = w_old + dw_dt * DT_FAST  # REG WRITE: update speed measurement

    # =========================================================================
    # 4. LOGGING (every 10th sample = 1 kHz logging rate)
    # NOT part of hardware - just for analysis
    # =========================================================================
    if n % 10 == 0:
        t_log.append(t)
        w_log.append(ctrl.w_meas)  # Log actual speed
        i_log.append(ctrl.i_meas)  # Log actual current
        v_log.append(ctrl.v_cmd)  # Log commanded voltage
        v_actual_log.append(v_actual)  # Log actual voltage (after saturation)
        current_ref_log.append(ctrl.current_ref)  # Log current reference
        speed_int_log.append(ctrl.speed_integral)  # Log integrator state
        setpoint_log.append(ctrl.w_ref)  # Log setpoint
        load_log.append(load)  # Log load torque

print(f"Simulation complete: {SIM_TIME}s simulated")

# =============================================================================
# PLOTTING
# =============================================================================
print("\nGenerating plots...")

fig, axes = plt.subplots(4, 1, figsize=(14, 12))

# Speed
ax = axes[0]
ax.plot(t_log, w_log, 'b-', label='Measured speed', linewidth=2)
ax.plot(t_log, setpoint_log, 'r--', label='Setpoint', linewidth=1.5, alpha=0.8)
ax.axvspan(DISTURBANCE_TIME, DISTURBANCE_TIME + DISTURBANCE_DURATION,
           alpha=0.2, color='red', label='Disturbance')
ax.set_ylabel('Speed (rad/s)', fontsize=11)
ax.set_title('Cascaded PI Control – Stress Test (Parameter Mismatch + Disturbance + Delay + Bus Droop)',
             fontsize=13, fontweight='bold')
ax.legend(loc='upper left')
ax.grid(True, alpha=0.3)

# Current
ax = axes[1]
ax.plot(t_log, i_log, 'g-', label='Actual Current', linewidth=2)
ax.plot(t_log, current_ref_log, 'm--', label='Current reference', linewidth=1.5, alpha=0.8)
ax.axvspan(DISTURBANCE_TIME, DISTURBANCE_TIME + DISTURBANCE_DURATION,
           alpha=0.2, color='red')
ax.set_ylabel('Current (A)', fontsize=11)
ax.legend(loc='upper left')
ax.grid(True, alpha=0.3)

# Voltage
ax = axes[2]
ax.plot(t_log, v_log, 'r-', label='Commanded Voltage', linewidth=2)
ax.plot(t_log, v_actual_log, 'orange', label='Actual Voltage (after saturation)',
        linewidth=1.5, linestyle='--', alpha=0.8)
ax.axvspan(DISTURBANCE_TIME, DISTURBANCE_TIME + DISTURBANCE_DURATION,
           alpha=0.2, color='red')
ax.set_ylabel('Voltage (V)', fontsize=11)
ax.legend(loc='upper left')
ax.grid(True, alpha=0.3)

# Load Torque
ax = axes[3]
ax.plot(t_log, load_log, 'k-', label='Load Torque', linewidth=2)
ax.set_xlabel('Time (s)', fontsize=11)
ax.set_ylabel('Torque (Nm)', fontsize=11)
ax.legend(loc='upper left')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# =============================================================================
# ANALYSIS
# =============================================================================
print("\n" + "=" * 70)
print("STEADY-STATE PERFORMANCE (with all non-idealities)")
print("=" * 70)

segments = [
    (50.0, 10.0, 11.5),  # Before disturbance
    (100.0, 17.0, 19.0),
    (150.0, 24.0, 26.0)
]

for sp, t_start, t_end in segments:
    speeds_in_window = []
    for i, t in enumerate(t_log):
        if t_start <= t <= t_end:
            speeds_in_window.append(w_log[i])

    if speeds_in_window:
        avg_speed = np.mean(speeds_in_window)
        std_speed = np.std(speeds_in_window)
        error = sp - avg_speed
        error_pct = (error / sp) * 100 if sp != 0 else 0
        print(f"Setpoint {sp:3.0f} rad/s → avg={avg_speed:7.3f} rad/s, "
              f"error={error:+7.4f} rad/s ({error_pct:+.3f}%), ripple={std_speed:.4f} rad/s")

# =============================================================================
# DISTURBANCE REJECTION ANALYSIS
# =============================================================================
print("\n" + "=" * 70)
print("DISTURBANCE REJECTION PERFORMANCE")
print("=" * 70)

# Find disturbance period
dist_start_idx = next(i for i, t in enumerate(t_log) if t >= DISTURBANCE_TIME)
dist_end_idx = next(i for i, t in enumerate(t_log) if t >= DISTURBANCE_TIME + DISTURBANCE_DURATION)

# Speed before disturbance (steady-state)
speeds_before = w_log[dist_start_idx - 100:dist_start_idx]
speed_ss = np.mean(speeds_before)

# Find maximum speed dip during and after disturbance
speeds_during = w_log[dist_start_idx:dist_end_idx + 500]  # Include recovery
min_speed = np.min(speeds_during)
speed_dip = speed_ss - min_speed
speed_dip_pct = (speed_dip / speed_ss) * 100

# Find recovery time (back to ±2% of steady-state)
recovery_threshold = 0.02 * speed_ss
recovered = False
recovery_time = None
for i in range(dist_start_idx, min(dist_end_idx + 1000, len(w_log))):
    if abs(w_log[i] - speed_ss) <= recovery_threshold:
        if all(abs(w_log[j] - speed_ss) <= recovery_threshold
               for j in range(i, min(i + 50, len(w_log)))):  # Sustained recovery
            recovery_time = t_log[i] - DISTURBANCE_TIME
            recovered = True
            break

# Find peak current during disturbance response
currents_during = i_log[dist_start_idx:dist_end_idx + 200]
peak_current = np.max(np.abs(currents_during))

print(f"Disturbance magnitude:    {DISTURBANCE_VALUE} Nm")
print(f"Steady-state speed:       {speed_ss:.3f} rad/s")
print(f"Maximum speed dip:        {speed_dip:.3f} rad/s ({speed_dip_pct:.2f}%)")
if recovery_time:
    print(f"Recovery time (±2%):      {recovery_time * 1000:.1f} ms")
else:
    print(f"Recovery time (±2%):      >1000ms or did not recover")
print(f"Peak current response:    {peak_current:.3f} A")


# =============================================================================
# STEP RESPONSE ANALYSIS
# =============================================================================
def analyze_step_response(t_log, w_log, step_time, step_value, settle_tol=0.02):
    """Compute step response metrics."""
    try:
        step_idx = next(i for i, t in enumerate(t_log) if t >= step_time)
    except StopIteration:
        return None

    try:
        end_idx = next(i for i, t in enumerate(t_log) if t >= step_time + 5.0)
    except StopIteration:
        end_idx = len(t_log) - 1

    t_step = [t - step_time for t in t_log[step_idx:end_idx]]
    y = w_log[step_idx:end_idx]

    if len(y) == 0:
        return None

    ref = step_value

    # Rise time (10% to 90%)
    try:
        idx_10 = next(i for i, val in enumerate(y) if val >= 0.1 * ref)
        idx_90 = next(i for i, val in enumerate(y) if val >= 0.9 * ref)
        rise_time = t_step[idx_90] - t_step[idx_10]
    except StopIteration:
        rise_time = None

    # Overshoot
    peak = max(y) if y else 0
    overshoot = (peak - ref) / ref * 100 if ref != 0 else 0

    # Settling time (±2%)
    try:
        settled_idx = None
        for i in range(len(y)):
            if all(abs(v - ref) <= settle_tol * ref for v in y[i:]):
                settled_idx = i
                break
        settling_time = t_step[settled_idx] if settled_idx is not None else None
    except:
        settling_time = None

    # Steady-state error
    n_ss = min(100, len(y))
    if n_ss > 0:
        ss_error = ref - np.mean(y[-n_ss:])
    else:
        ss_error = None

    # Print
    print(f"\n--- Step Response at t={step_time}s (ref={ref} rad/s) ---")
    if rise_time:
        print(f"Rise time (10-90%):   {rise_time:.4f} s ({rise_time * 1000:.1f} ms)")
    else:
        print("Rise time (10-90%):   N/A")
    print(f"Overshoot:            {overshoot:.2f} %")
    if settling_time:
        print(f"Settling time (±2%):  {settling_time:.4f} s ({settling_time * 1000:.1f} ms)")
    else:
        print("Settling time (±2%):  N/A")
    if ss_error is not None:
        print(f"Steady-state error:   {ss_error:.6f} rad/s")

    return {
        'rise_time': rise_time,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'ss_error': ss_error
    }


print("\n" + "=" * 70)
print("STEP RESPONSE METRICS (with all non-idealities)")
print("=" * 70)

analyze_step_response(t_log, w_log, step_time=5.0, step_value=50.0)
analyze_step_response(t_log, w_log, step_time=12.0, step_value=100.0)
analyze_step_response(t_log, w_log, step_time=19.0, step_value=150.0)

print("\n" + "=" * 70)
print("STRESS TEST COMPLETE")
print("=" * 70)