import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass



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


print("✓ Motor parameters defined")


class DCMotor:
    """Simple DC motor model"""

    def __init__(self, params: MotorParams):
        self.params = params
        self.current = 0.0
        self.omega = 0.0
        self.theta = 0.0

    def step(self, voltage, load_torque=0.0, dt=100e-6):
        """Euler integration step"""
        p = self.params

        # Electrical dynamics
        back_emf = p.Ke * self.omega
        di_dt = (voltage - p.R * self.current - back_emf) / p.L
        self.current += di_dt * dt

        # Mechanical dynamics
        motor_torque = p.Kt * self.current
        friction_torque = p.Bf * self.omega
        dw_dt = (motor_torque - friction_torque - load_torque) / p.J
        self.omega += dw_dt * dt

        # Position
        self.theta += self.omega * dt

        return self.current, self.omega


print("✓ DCMotor class defined")


class PController:
    """Proportional-only controller"""

    def __init__(self, Kp, output_limits):
        self.Kp = Kp
        self.output_limits = output_limits

    def update(self, error):
        """P control with saturation"""
        output = self.Kp * error
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        return output


print("✓ PController class defined")


def test_current_loop_p_only(Kp_test, duration=1.0):
    """Test current loop with P-only controller"""
    print(f"  Running simulation with Kp={Kp_test}...", end='')

    params = MotorParams()
    motor = DCMotor(params)
    controller = PController(Kp=Kp_test, output_limits=(-24.0, 24.0))

    dt = params.Ts_current
    n_steps = int(duration / dt)

    log = {
        'time': [],
        'current': [],
        'current_ref': [],
        'voltage': [],
        'omega': []
    }

    current_ref = 0.0

    for step in range(n_steps):
        t = step * dt

        if t >= 0.2:
            current_ref = 2.0

        error = current_ref - motor.current
        voltage = controller.update(error)
        motor.step(voltage, load_torque=0.0, dt=dt)

        if step % 10 == 0:
            log['time'].append(t)
            log['current'].append(motor.current)
            log['current_ref'].append(current_ref)
            log['voltage'].append(voltage)
            log['omega'].append(motor.omega)

    print(" Done!")
    return log


print("✓ test_current_loop_p_only() defined")


def detect_oscillation_simple(signal, dt, threshold_ratio=0.03):
    """
    Simple oscillation detection
    """
    if len(signal) < 20:
        return False, None

    # Calculate statistics
    signal_mean = np.mean(signal)
    signal_std = np.std(signal)

    # Amplitude ratio
    if abs(signal_mean) > 1e-6:
        amplitude_ratio = signal_std / abs(signal_mean)
    else:
        amplitude_ratio = signal_std

    # Count zero crossings
    signal_detrended = signal - signal_mean
    zero_crossings = np.sum(np.abs(np.diff(np.sign(signal_detrended))) > 0)

    # Simple period estimation from zero crossings
    period = None
    if zero_crossings >= 4:
        # Average time between crossings × 2 = period
        signal_length_time = len(signal) * dt
        period = (signal_length_time / zero_crossings) * 2

    # Oscillating if sufficient amplitude AND multiple crossings
    is_oscillating = (amplitude_ratio > threshold_ratio) and (zero_crossings >= 4)

    return is_oscillating, period


print("✓ detect_oscillation_simple() defined")


def find_ultimate_gain_current():
    """Find ultimate gain where oscillation starts"""
    print("\n" + "=" * 70)
    print(" " * 15 + "FINDING ULTIMATE GAIN FOR CURRENT LOOP")
    print("=" * 70)

    # Test range
    Kp_values = [55, 100, 200, 400, 600, 800, 1000, 1500, 2000, 3000, 4000, 5000, 7500, 100000]

    results = {}

    for Kp in Kp_values:
        print(f"\n[Test {Kp_values.index(Kp) + 1}/{len(Kp_values)}] Kp = {Kp}")

        # Run simulation
        log = test_current_loop_p_only(Kp_test=Kp, duration=1.5)

        # Analyze steady-state (after t=0.5s)
        start_idx = int(0.5 / 1e-4)
        current_ss = np.array(log['current'][start_idx:])

        # Detect oscillation
        is_oscillating, period = detect_oscillation_simple(current_ss, dt=1e-4)

        results[Kp] = {
            'log': log,
            'oscillating': is_oscillating,
            'period': period
        }

        if is_oscillating:
            print(f"  → STATUS: OSCILLATING! ⚠️")
            if period:
                print(f"     Period Tu = {period * 1000:.2f} ms")
        else:
            ss_error = np.mean(current_ss[-100:]) - 2.0
            ss_std = np.std(current_ss)
            print(f"  → STATUS: Stable ✓")
            print(f"     SS error = {ss_error:.4f} A, Std dev = {ss_std:.4f} A")

    print("\n" + "=" * 70)
    print(" " * 20 + "SWEEP COMPLETE")
    print("=" * 70)

    return results


print("✓ find_ultimate_gain_current() defined")


def plot_results(results):
    """Plot test results"""
    print("\n📊 Generating plots...")

    Kp_list = sorted(results.keys())
    n_plots = min(12, len(Kp_list))

    fig, axes = plt.subplots(4, 3, figsize=(16, 12))
    axes = axes.flatten()

    for idx, Kp in enumerate(Kp_list[:n_plots]):
        ax = axes[idx]
        log = results[Kp]['log']

        ax.plot(log['time'], log['current'], label='Current', linewidth=1.5)
        ax.plot(log['time'], log['current_ref'], '--', label='Ref', linewidth=1)
        ax.axhline(2.0, color='gray', linestyle=':', linewidth=0.8)

        if results[Kp]['oscillating']:
            ax.set_title(f"Kp={Kp} - OSCILLATING", fontweight='bold', color='red')
        else:
            ax.set_title(f"Kp={Kp} - Stable", color='green')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Current (A)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_xlim([0.15, 1.0])
        ax.set_ylim([0, 3.5])

    for idx in range(n_plots, len(axes)):
        axes[idx].axis('off')

    plt.tight_layout()
    plt.savefig('oscillation_test.png', dpi=150)
    print("✓ Plot saved: oscillation_test.png")
    plt.show()


print("✓ plot_results() defined")


def determine_ultimate_parameters():
    """Main function to find Ku and Tu"""
    print("\n" + "=" * 70)
    print(" " * 10 + "ZIEGLER-NICHOLS TUNING - ULTIMATE GAIN METHOD")
    print("=" * 70)

    # Run sweep
    results = find_ultimate_gain_current()

    # Find first oscillating Kp
    Ku = None
    Tu = None

    for Kp in sorted(results.keys()):
        if results[Kp]['oscillating']:
            Ku = Kp
            Tu = results[Kp]['period']
            break

    # Report results
    print("\n" + "=" * 70)
    if Ku is None:
        print(" " * 20 + "❌ NO OSCILLATION FOUND")
        print("=" * 70)
        print("\nPossible reasons:")
        print("  1. System is very stable (high damping)")
        print("  2. Need to test higher Kp values (>1000)")
        print("  3. Detection threshold too strict")
        print("\nRecommendation: Use model-based tuning instead")
        print("  Kp_current ≈ L × ωc = 0.5e-3 × 6283 ≈ 3.14")
        print("  Ki_current ≈ R × ωc = 2.5 × 6283 ≈ 15707")
        print("  (for 1 kHz bandwidth)")
    else:
        print(" " * 20 + "✅ ULTIMATE PARAMETERS FOUND")
        print("=" * 70)
        print(f"\n  Ku (Ultimate Gain):    {Ku}")
        if Tu:
            print(f"  Tu (Ultimate Period):  {Tu * 1000:.2f} ms")
            print(f"  ωu (Critical Freq):    {2 * np.pi / Tu:.2f} rad/s")
            print(f"\n  Ziegler-Nichols PI Gains:")
            Kp_zn = 0.45 * Ku
            Ki_zn = 0.54 * Ku / Tu if Tu else 0
            print(f"    Kp = 0.45 × {Ku} = {Kp_zn:.4f}")
            print(f"    Ki = 0.54 × {Ku} / {Tu:.6f} = {Ki_zn:.2f}")
        else:
            print(f"  Tu (Ultimate Period):  Could not determine")
            print(f"\n  Warning: Period estimation failed")

    print("=" * 70)

    # Generate plots
    plot_results(results)

    return Ku, Tu, results


print("✓ determine_ultimate_parameters() defined")

# ==================================================================
# MAIN EXECUTION
# ==================================================================
print("\n" + "=" * 70)
print(" " * 25 + "STARTING TUNING")
print("=" * 70)

try:
    Ku_current, Tu_current, results = determine_ultimate_parameters()

    print("\n" + "=" * 70)
    print(" " * 25 + "TUNING COMPLETE")
    print("=" * 70)

    if Ku_current:
        print(f"\n✅ Success! Ultimate gain found: Ku = {Ku_current}")
        if Tu_current:
            print(f"✅ Period measured: Tu = {Tu_current * 1000:.2f} ms")
            print(f"\n📋 Recommended PI gains:")
            print(f"   Kp_current = {0.45 * Ku_current:.4f}")
            print(f"   Ki_current = {0.54 * Ku_current / Tu_current:.2f}")
    else:
        print("\n⚠️  No oscillation detected in tested range")
        print("    Consider using model-based tuning")

    print("\n" + "=" * 70)

except Exception as e:
    print("\n" + "=" * 70)
    print(" " * 25 + "ERROR OCCURRED")
    print("=" * 70)
    print(f"\nException: {type(e).__name__}")
    print(f"Message: {str(e)}")
    print("\nFull traceback:")
    import traceback

    traceback.print_exc()
    print("=" * 70)

print("\n✓ Script finished")
