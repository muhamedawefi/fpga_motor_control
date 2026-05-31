import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass

print("=" * 70)
print(" " * 15 + "MODEL-BASED PI CONTROLLER TUNING")
print(" " * 20 + "For DC Motor Control")
print("=" * 70)


@dataclass
class MotorParams:
    """Motor parameters"""
    R: float = 2.5  # Resistance (Ohm)
    L: float = 0.5e-3  # Inductance (H)
    Kt: float = 0.05  # Torque constant (Nm/A)
    Ke: float = 0.05  # Back-EMF constant (V·s/rad)
    J: float = 0.0001  # Inertia (kg·m²)
    Bf: float = 0.0001  # Friction (Nm·s/rad)

    Ts_current: float = 100e-6  # Current loop: 10 kHz
    Ts_speed: float = 1e-3  # Speed loop: 1 kHz

    V_max: float = 24.0  # Max voltage
    I_max: float = 5.0  # Max current


class DCMotor:
    """DC motor model"""

    def __init__(self, params: MotorParams):
        self.params = params
        self.current = 0.0
        self.omega = 0.0
        self.theta = 0.0

    def step(self, voltage, load_torque=0.0, dt=100e-6):
        p = self.params
        back_emf = p.Ke * self.omega
        di_dt = (voltage - p.R * self.current - back_emf) / p.L
        self.current += di_dt * dt

        motor_torque = p.Kt * self.current
        friction_torque = p.Bf * self.omega
        dw_dt = (motor_torque - friction_torque - load_torque) / p.J
        self.omega += dw_dt * dt

        self.theta += self.omega * dt
        return self.current, self.omega


class PIController:
    """PI controller with anti-windup"""

    def __init__(self, Kp, Ki, Ts, output_limits):
        self.Kp = Kp
        self.Ki = Ki
        self.Ts = Ts
        self.output_limits = output_limits
        self.Tt = 0.5 * Ts  # Tracking time constant
        self.integral = 0.0

    def update(self, error):
        P = self.Kp * error
        I = self.integral
        output_raw = P + I
        output = np.clip(output_raw, self.output_limits[0], self.output_limits[1])

        # Anti-windup
        saturation_error = output - output_raw
        self.integral += self.Ki * error * self.Ts + (saturation_error / self.Tt)

        return output

    def reset(self):
        self.integral = 0.0


def calculate_current_loop_gains(params, bandwidth_hz=1000):
    """
    Model-based tuning for current loop

    Method: Pole-zero cancellation + desired bandwidth placement

    Args:
        params: MotorParams
        bandwidth_hz: Desired closed-loop bandwidth (Hz)

    Returns:
        Kp, Ki: PI controller gains
    """
    print("\n" + "=" * 70)
    print(" " * 20 + "CURRENT LOOP TUNING")
    print("=" * 70)

    # System parameters
    R = params.R
    L = params.L
    tau_elec = L / R  # Electrical time constant

    print(f"\nElectrical System:")
    print(f"  Resistance R = {R} Ω")
    print(f"  Inductance L = {L * 1e3} mH")
    print(f"  Time constant τ = {tau_elec * 1e6:.1f} µs")
    print(f"  Natural bandwidth = {1 / (2 * np.pi * tau_elec):.0f} Hz")

    # Desired closed-loop bandwidth
    omega_c = 2 * np.pi * bandwidth_hz  # rad/s

    print(f"\nDesign Target:")
    print(f"  Bandwidth = {bandwidth_hz} Hz ({omega_c:.0f} rad/s)")

    # PI controller gains (pole-zero cancellation method)
    # Place closed-loop pole at -ωc
    Kp = L * omega_c  # Proportional gain
    Ki = R * omega_c  # Integral gain

    print(f"\nCalculated PI Gains:")
    print(f"  Kp = L × ωc = {L} × {omega_c:.0f} = {Kp:.4f}")
    print(f"  Ki = R × ωc = {R} × {omega_c:.0f} = {Ki:.2f}")

    # Closed-loop characteristics
    settling_time = 4.6 / omega_c  # 1% settling time
    rise_time = 1.8 / omega_c  # 10-90% rise time

    print(f"\nExpected Performance:")
    print(f"  Rise time (10-90%): {rise_time * 1e3:.2f} ms")
    print(f"  Settling time (1%): {settling_time * 1e3:.2f} ms")
    print(f"  Overshoot: 0% (critically damped)")
    print(f"  Steady-state error: 0 (integral action)")

    print("=" * 70)

    return Kp, Ki


def calculate_speed_loop_gains(params, Kp_current, Ki_current, bandwidth_hz=10, zeta=0.5):
    """
    Model-based tuning for speed loop WITH DAMPING RATIO ζ

    Args:
        params: MotorParams
        Kp_current, Ki_current: Inner loop gains
        bandwidth_hz: Desired speed loop bandwidth (Hz)
        zeta: Damping ratio (0.5=fast/16% overshoot, 0.707=optimal, 1.0=no overshoot)

    Returns:
        Kp, Ki: Speed PI controller gains
    """
    print("\n" + "=" * 70)
    print(" " * 15 + f"SPEED LOOP TUNING (ζ = {zeta})")
    print("=" * 70)

    # Parameters
    Kt, J, Bf = params.Kt, params.J, params.Bf
    tau_mech = J / Bf

    print(f"\nMechanical System:")
    print(f"  Kt = {Kt} Nm/A, J = {J} kg·m², Bf = {Bf} Nm·s/rad")
    print(f"  τ_mech = {tau_mech:.3f} s, f_nat = {1 / (2 * np.pi * tau_mech):.1f} Hz")

    # Design specs (USE THE PARAMETER!)
    omega_n = 2 * np.pi * bandwidth_hz  # ✅ Use bandwidth_hz parameter!

    print(f"\nDesign Specifications:")
    print(f"  Bandwidth = {bandwidth_hz} Hz (ω_n = {omega_n:.1f} rad/s)")
    print(f"  Damping ratio ζ = {zeta}")

    # Check bandwidth separation
    current_bw = Ki_current / Kp_current
    separation_ratio = current_bw / omega_n
    print(f"  Current BW ≈ {current_bw / (2 * np.pi):.0f} Hz")
    print(f"  Separation = {separation_ratio:.1f}:1", "✓" if separation_ratio > 10 else "⚠️")

    # POLE PLACEMENT GAINS with ζ ✅
    Kp_speed = (2 * zeta * omega_n * J - Bf) / Kt
    Ki_speed = (omega_n ** 2 * J) / Kt

    print(f"\nPI Controller Gains (Pole Placement):")
    print(f"  Kp = (2ζω_n J - Bf) / Kt = {Kp_speed:.6f}")
    print(f"  Ki = (ω_n² J) / Kt      = {Ki_speed:.6f}")

    # Expected performance
    if zeta < 1:
        overshoot_pct = np.exp(-zeta * np.pi / np.sqrt(1 - zeta ** 2)) * 100
    else:
        overshoot_pct = 0

    rise_time = 1.8 / omega_n
    settling_time = 4.6 / (zeta * omega_n)

    print(f"\nExpected Closed-Loop Performance:")
    print(f"  Rise time (10-90%):  {rise_time * 1e3:.1f} ms")
    print(f"  Overshoot:           {overshoot_pct:.1f}%")
    print(f"  Settling (±1%):      {settling_time * 1e3:.1f} ms")

    # Reference filter recommendation for ζ < 0.707
    if zeta < 0.707:
        Tf_ref = 1 / (3 * omega_n)
        print(f"\n💡 RECOMMENDATION: ζ={zeta} → Add reference prefilter!")
        print(f"   Suggested filter time constant: Tf = {Tf_ref * 1e3:.1f} ms")
    else:
        print(f"\n✅ No reference filter needed (ζ ≥ 0.707)")

    print("=" * 70)
    return Kp_speed, Ki_speed


def test_current_loop(Kp, Ki, params, duration=2.0):
    """Test current loop with step response"""
    motor = DCMotor(params)
    controller = PIController(Kp, Ki, params.Ts_current, (-24.0, 24.0))

    dt = params.Ts_current
    n_steps = int(duration / dt)

    log = {'time': [], 'current': [], 'current_ref': [], 'voltage': []}
    current_ref = 0.0

    for step in range(n_steps):
        t = step * dt

        if t >= 0.5:
            current_ref = 2.0

        error = current_ref - motor.current
        voltage = controller.update(error)
        motor.step(voltage, load_torque=0.0, dt=dt)

        if step % 10 == 0:
            log['time'].append(t)
            log['current'].append(motor.current)
            log['current_ref'].append(current_ref)
            log['voltage'].append(voltage)

    return log


def test_cascaded_system(Kp_speed, Ki_speed, Kp_current, Ki_current, params, duration=3.0):
    """Test full cascaded control"""
    motor = DCMotor(params)

    speed_pi = PIController(Kp_speed, Ki_speed, params.Ts_speed, (-5.0, 5.0))
    current_pi = PIController(Kp_current, Ki_current, params.Ts_current, (-24.0, 24.0))

    dt = params.Ts_current
    n_steps = int(duration / dt)

    log = {
        'time': [], 'omega': [], 'omega_ref': [],
        'current': [], 'current_ref': [], 'voltage': []
    }

    omega_ref = 0.0
    current_ref = 0.0
    speed_counter = 0

    for step in range(n_steps):
        t = step * dt

        if t >= 1.0:
            omega_ref = 100.0

        # Speed loop (1 kHz)
        if speed_counter == 0:
            speed_error = omega_ref - motor.omega
            current_ref = speed_pi.update(speed_error)

        speed_counter = (speed_counter + 1) % 10

        # Current loop (10 kHz)
        current_error = current_ref - motor.current
        voltage = current_pi.update(current_error)

        motor.step(voltage, load_torque=0.0, dt=dt)

        if step % 10 == 0:
            log['time'].append(t)
            log['omega'].append(motor.omega)
            log['omega_ref'].append(omega_ref)
            log['current'].append(motor.current)
            log['current_ref'].append(current_ref)
            log['voltage'].append(voltage)

    return log


def analyze_step_response(time, signal, ref, step_time):
    """Analyze step response metrics"""
    time = np.array(time)
    signal = np.array(signal)

    step_idx = np.argmin(np.abs(time - step_time))
    time_post = time[step_idx:]
    signal_post = signal[step_idx:]

    # Rise time (10-90%)
    val_10 = 0.1 * ref
    val_90 = 0.9 * ref
    try:
        idx_10 = np.argmax(signal_post >= val_10)
        idx_90 = np.argmax(signal_post >= val_90)
        rise_time = time_post[idx_90] - time_post[idx_10]
    except:
        rise_time = 0

    # Overshoot
    peak = np.max(signal_post[:min(len(signal_post), 1000)])
    overshoot = ((peak - ref) / ref) * 100 if ref > 0 else 0

    # Settling time (2% band)
    try:
        settled = np.abs(signal_post - ref) < 0.02 * ref
        if np.any(settled):
            settling_idx = np.where(settled)[0][0]
            settling_time = time_post[settling_idx] - time_post[0]
        else:
            settling_time = time_post[-1] - time_post[0]
    except:
        settling_time = 0

    # SS error
    ss_error = np.mean(signal_post[-100:]) - ref

    return {
        'rise_time': rise_time,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'ss_error': ss_error
    }


def plot_results(log_current, log_full, metrics_current, metrics_speed):
    """Plot tuning results"""
    fig = plt.figure(figsize=(16, 10))

    # Current loop response
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(log_current['time'], log_current['current'], linewidth=2, label='Current')
    ax1.plot(log_current['time'], log_current['current_ref'], '--', linewidth=1.5, label='Reference')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Current (A)')
    ax1.set_title('Current Loop Step Response')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_xlim([0.4, 1.0])

    # Current loop voltage
    ax2 = plt.subplot(3, 2, 2)
    ax2.plot(log_current['time'], log_current['voltage'], linewidth=2, color='orange')
    ax2.axhline(24, color='red', linestyle='--', alpha=0.5)
    ax2.axhline(-24, color='red', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Voltage (V)')
    ax2.set_title('Current Loop Control Voltage')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0.4, 1.0])

    # Speed response
    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(log_full['time'], log_full['omega'], linewidth=2, label='Speed')
    ax3.plot(log_full['time'], log_full['omega_ref'], '--', linewidth=1.5, label='Reference')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Speed (rad/s)')
    ax3.set_title('Speed Loop Step Response')
    ax3.grid(True, alpha=0.3)
    ax3.legend()

    # Current tracking
    ax4 = plt.subplot(3, 2, 4)
    ax4.plot(log_full['time'], log_full['current'], linewidth=2, color='green', label='Current')
    ax4.plot(log_full['time'], log_full['current_ref'], '--', linewidth=1.5, color='red', label='Reference')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Current (A)')
    ax4.set_title('Current Tracking (Inner Loop)')
    ax4.grid(True, alpha=0.3)
    ax4.legend()

    # Voltage
    ax5 = plt.subplot(3, 2, 5)
    ax5.plot(log_full['time'], log_full['voltage'], linewidth=2, color='purple')
    ax5.axhline(24, color='red', linestyle='--', alpha=0.5)
    ax5.axhline(-24, color='red', linestyle='--', alpha=0.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Voltage (V)')
    ax5.set_title('Control Voltage')
    ax5.grid(True, alpha=0.3)

    # Metrics table
    ax6 = plt.subplot(3, 2, 6)
    ax6.axis('off')

    metrics_text = f"""
    CURRENT LOOP PERFORMANCE:
      Rise time:     {metrics_current['rise_time'] * 1000:.2f} ms
      Overshoot:     {metrics_current['overshoot']:.1f} %
      Settling:      {metrics_current['settling_time'] * 1000:.2f} ms
      SS error:      {abs(metrics_current['ss_error']) * 1000:.2f} mA

    SPEED LOOP PERFORMANCE:
      Rise time:     {metrics_speed['rise_time'] * 1000:.1f} ms
      Overshoot:     {metrics_speed['overshoot']:.1f} %
      Settling:      {metrics_speed['settling_time'] * 1000:.1f} ms
      SS error:      {abs(metrics_speed['ss_error']):.4f} rad/s
    """

    ax6.text(0.1, 0.5, metrics_text, fontsize=11, family='monospace',
             verticalalignment='center',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plt.savefig('model_based_tuning_results.png', dpi=150)
    print("\n✓ Plot saved: model_based_tuning_results.png")
    plt.show()


# ==================================================================
# MAIN EXECUTION
# ==================================================================

params = MotorParams()

print("\n" + "=" * 70)
print(" " * 25 + "STARTING TUNING")
print("=" * 70)

# STEP 1: Tune current loop
Kp_current, Ki_current = calculate_current_loop_gains(params, bandwidth_hz=1000)

# STEP 2: Test current loop
print("\n📊 Testing current loop...")
log_current = test_current_loop(Kp_current, Ki_current, params)
metrics_current = analyze_step_response(
    log_current['time'], log_current['current'], 2.0, 0.5
)

print(f"\nCurrent Loop Results:")
print(f"  Rise time: {metrics_current['rise_time'] * 1000:.2f} ms")
print(f"  Overshoot: {metrics_current['overshoot']:.1f}%")
print(f"  SS error: {abs(metrics_current['ss_error']) * 1000:.2f} mA")

# STEP 3: Tune speed loop
Kp_speed, Ki_speed = calculate_speed_loop_gains(
    params, Kp_current, Ki_current, bandwidth_hz=10
)

# STEP 4: Test full system
print("\n📊 Testing cascaded system...")
log_full = test_cascaded_system(Kp_speed, Ki_speed, Kp_current, Ki_current, params)
metrics_speed = analyze_step_response(
    log_full['time'], log_full['omega'], 100.0, 1.0
)

print(f"\nSpeed Loop Results:")
print(f"  Rise time: {metrics_speed['rise_time'] * 1000:.1f} ms")
print(f"  Overshoot: {metrics_speed['overshoot']:.1f}%")
print(f"  SS error: {abs(metrics_speed['ss_error']):.4f} rad/s")

# STEP 5: Plot
plot_results(log_current, log_full, metrics_current, metrics_speed)

# STEP 6: Save gains
print("\n" + "=" * 70)
print(" " * 25 + "FINAL TUNED GAINS")
print("=" * 70)
print(f"\nCurrent Loop (10 kHz):")
print(f"  Kp_current = {Kp_current:.6f}")
print(f"  Ki_current = {Ki_current:.2f}")
print(f"\nSpeed Loop (1 kHz):")
print(f"  Kp_speed = {Kp_speed:.6f}")
print(f"  Ki_speed = {Ki_speed:.6f}")
print("=" * 70)

# Save to file
with open('tuned_gains_model_based.txt', 'w') as f:
    f.write("# Model-Based Tuned Gains\n")
    f.write(f"# Current Loop Bandwidth: 1000 Hz\n")
    f.write(f"# Speed Loop Bandwidth: 10 Hz\n\n")
    f.write(f"Kp_current = {Kp_current:.6f}\n")
    f.write(f"Ki_current = {Ki_current:.2f}\n")
    f.write(f"Kp_speed = {Kp_speed:.6f}\n")
    f.write(f"Ki_speed = {Ki_speed:.6f}\n")

print("\n✓ Gains saved to 'tuned_gains_model_based.txt'")
print("\n✓ Tuning complete! Ready for fixed-point conversion.\n")
