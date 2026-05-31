import numpy as np
import matplotlib.pyplot as plt
import scipy as sc
import scipy.signal as signal
class MotorParams:
    """DC Motor physical parameters"""

    def __init__(self):
        # Electrical
        self.R = 2.5  # Armature resistance (Ω)
        self.L = 0.0005  # Armature inductance (H)
        self.Ke = 0.05  # Back-EMF constant (V/(rad/s))

        # Mechanical
        self.J = 0.0001  # Rotor inertia (kg·m²)
        self.Bf = 0.0001  # Viscous friction (Nm·s/rad)
        self.Kt = 0.05  # Torque constant (Nm/A)

        # Sampling
        self.Ts = 0.001  # Sample time (1ms = 1kHz for speed loop)
        self.Ts_current = 0.0001  # Current loop sample time (100µs = 10kHz)


def analyze_stability_offline(params: MotorParams,
                              Kp_speed: float, Ki_speed: float,
                              Kp_current: float, Ki_current: float):
    """
    Complete offline stability analysis using ONLY scipy.signal
    """

    print("\n" + "=" * 80)
    print("OFFLINE STABILITY ANALYSIS")
    print("=" * 80)

    # ============================================
    # 1. CURRENT LOOP ANALYSIS (INNER LOOP)
    # ============================================
    print("\n>>> CURRENT LOOP (Inner, 10 kHz)")
    print("-" * 40)

    # Plant: G_i(s) = 1/(L*s + R)
    L = params.L
    R = params.R

    # Controller: C_i(s) = Kp + Ki/s
    Kp_i = Kp_current
    Ki_i = Ki_current

    # Create transfer functions
    num_plant_i = [1]
    den_plant_i = [L, R]  # L*s + R

    num_controller_i = [Kp_i, Ki_i]  # Kp*s + Ki
    den_controller_i = [1, 0]  # s

    # Convert to transfer functions using scipy.signal
    G_i = signal.TransferFunction(num_plant_i, den_plant_i)
    C_i = signal.TransferFunction(num_controller_i, den_controller_i)
    num_Li = np.convolve(C_i.num, G_i.num)
    den_Li = np.convolve(C_i.den, G_i.den)
    L_i = signal.TransferFunction(num_Li, den_Li)

    # Frequency range (rad/s) - wider range for current loop
    w_i = np.logspace(1, 7, 5000)  # 10^1 to 10^7 rad/s

    # Compute frequency response
    w_i, mag_i, phase_i = signal.bode(L_i, w_i)

    # Find gain crossover (where |L(jw)| = 1 = 0 dB)
    # More robust method: find where magnitude crosses 0 dB
    cross_indices = np.where(np.diff(np.sign(mag_i)))[0]

    pm_i = None
    wc_i = None
    f_crossover_i = None
    tau_i = None

    if len(cross_indices) > 0:
        # Use first crossover
        idx = cross_indices[0]

        # Linear interpolation for more accurate crossover
        if idx < len(w_i) - 1:
            w1, w2 = w_i[idx], w_i[idx + 1]
            m1, m2 = mag_i[idx], mag_i[idx + 1]
            p1, p2 = phase_i[idx], phase_i[idx + 1]

            # Interpolate to find exact 0 dB crossover
            t = -m1 / (m2 - m1)
            wc_i = w1 + t * (w2 - w1)
            phase_at_crossover = p1 + t * (p2 - p1)

            pm_i = phase_at_crossover + 180
            f_crossover_i = wc_i / (2 * np.pi)
            tau_i = 1.0 / wc_i  # Time constant for current loop

            print(f"Gain Crossover Frequency: {f_crossover_i:.1f} Hz")
            print(f"Phase Margin: {pm_i:.1f}°")

            # Rule of thumb check
            if pm_i < 30:
                print("⚠️  WARNING: PM < 30° - RISKY!")
            elif pm_i < 45:
                print("⚠️  WARNING: PM < 45° - Marginal stability")
            elif pm_i > 70:
                print("⚠️  WARNING: PM > 70° - Too sluggish")
            else:
                print("✅ Good stability margin")
    else:
        print("❌ No crossover found in frequency range")

    # ============================================
    # 2. SPEED LOOP ANALYSIS (OUTER LOOP)
    # ============================================
    print("\n>>> SPEED LOOP (Outer, 1 kHz)")
    print("-" * 40)

    # Initialize speed loop variables
    pm_w = None
    wc_w = None
    f_crossover_w = None
    bandwidth_ratio = None

    # Only analyze speed loop if current loop analysis succeeded
    if wc_i is not None and tau_i is not None:
        # Speed plant: G_w(s) = Kt / (J*s + Bf)
        Kt = params.Kt
        J = params.J
        Bf = params.Bf

        # Speed controller: C_w(s) = Kp_w + Ki_w/s
        Kp_w = Kp_speed
        Ki_w = Ki_speed

        # Current closed-loop approximated as 1/(tau_i*s + 1)
        # Complete speed plant: [Kt/(J*s + Bf)] * [1/(tau_i*s + 1)]

        # Numerator: Kt
        # Denominator: (J*s + Bf)*(tau_i*s + 1) = J*tau_i*s^2 + (J + Bf*tau_i)*s + Bf
        num_speed_plant = [Kt]
        den_speed_plant = [J * tau_i, J + Bf * tau_i, Bf]

        num_controller_w = [Kp_w, Ki_w]
        den_controller_w = [1, 0]

        # Create transfer functions
        G_w = signal.TransferFunction(num_speed_plant, den_speed_plant)
        C_w = signal.TransferFunction(num_controller_w, den_controller_w)
        num_Lw = np.convolve(C_w.num, G_w.num)
        den_Lw = np.convolve(C_w.den, G_w.den)
        L_w = signal.TransferFunction(num_Lw, den_Lw)

        # Frequency range for speed loop (slower)
        w_w = np.logspace(-1, 4, 1000)  # 0.1 to 10,000 rad/s

        # Compute frequency response
        w_w, mag_w, phase_w = signal.bode(L_w, w_w)

        # Find crossover for speed loop
        cross_indices_w = np.where(np.diff(np.sign(mag_w)))[0]

        if len(cross_indices_w) > 0:
            # Use first crossover
            idx_w = cross_indices_w[0]

            if idx_w < len(w_w) - 1:
                # Linear interpolation
                w1, w2 = w_w[idx_w], w_w[idx_w + 1]
                m1, m2 = mag_w[idx_w], mag_w[idx_w + 1]
                p1, p2 = phase_w[idx_w], phase_w[idx_w + 1]

                t_w = -m1 / (m2 - m1)
                wc_w = w1 + t_w * (w2 - w1)
                phase_at_crossover_w = p1 + t_w * (p2 - p1)

                pm_w = phase_at_crossover_w + 180
                f_crossover_w = wc_w / (2 * np.pi)

                # Check speed vs current bandwidth ratio
                bandwidth_ratio = wc_i / wc_w

                print(f"Gain Crossover Frequency: {f_crossover_w:.1f} Hz")
                print(f"Phase Margin: {pm_w:.1f}°")
                print(f"Bandwidth Ratio (current/speed): {bandwidth_ratio:.1f}")

                if bandwidth_ratio < 5:
                    print("⚠️  WARNING: Bandwidth ratio < 5 - loops may interfere")
                else:
                    print("✅ Good separation between loops")
        else:
            print("❌ No crossover found in speed loop")
    else:
        print("❌ Skipped - current loop analysis failed")

    # ============================================
    # 3. PLOT BODE DIAGRAMS
    # ============================================
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Current loop Bode
    ax = axes[0, 0]
    ax.semilogx(w_i, mag_i, 'b-', linewidth=1.5, alpha=0.7)
    ax.set_title('Current Loop - Magnitude', fontweight='bold')
    ax.set_ylabel('Magnitude (dB)')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='r', linestyle='--', alpha=0.5, linewidth=1)
    if wc_i is not None:
        ax.axvline(x=wc_i, color='g', linestyle='--', alpha=0.5, linewidth=1)
        ax.text(wc_i, 5, f'{f_crossover_i:.0f} Hz',
                verticalalignment='bottom', horizontalalignment='center')

    ax = axes[0, 1]
    ax.semilogx(w_i, phase_i, 'b-', linewidth=1.5, alpha=0.7)
    ax.set_title('Current Loop - Phase', fontweight='bold')
    ax.set_ylabel('Phase (deg)')
    ax.set_xlabel('Frequency (rad/s)')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=-180, color='r', linestyle='--', alpha=0.5, linewidth=1)
    if wc_i is not None and pm_i is not None:
        ax.axvline(x=wc_i, color='g', linestyle='--', alpha=0.5, linewidth=1)
        ax.plot(wc_i, -180 + pm_i, 'ro', markersize=8, markerfacecolor='none')
        ax.text(wc_i, -180 + pm_i + 10, f'PM={pm_i:.1f}°',
                verticalalignment='bottom', horizontalalignment='center')

    # Speed loop Bode
    if pm_w is not None:
        ax = axes[1, 0]
        ax.semilogx(w_w, mag_w, 'g-', linewidth=1.5, alpha=0.7)
        ax.set_title('Speed Loop - Magnitude', fontweight='bold')
        ax.set_ylabel('Magnitude (dB)')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='r', linestyle='--', alpha=0.5, linewidth=1)
        if wc_w is not None:
            ax.axvline(x=wc_w, color='g', linestyle='--', alpha=0.5, linewidth=1)
            ax.text(wc_w, 5, f'{f_crossover_w:.0f} Hz',
                    verticalalignment='bottom', horizontalalignment='center')

        ax = axes[1, 1]
        ax.semilogx(w_w, phase_w, 'g-', linewidth=1.5, alpha=0.7)
        ax.set_title('Speed Loop - Phase', fontweight='bold')
        ax.set_ylabel('Phase (deg)')
        ax.set_xlabel('Frequency (rad/s)')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=-180, color='r', linestyle='--', alpha=0.5, linewidth=1)
        if wc_w is not None and pm_w is not None:
            ax.axvline(x=wc_w, color='g', linestyle='--', alpha=0.5, linewidth=1)
            ax.plot(wc_w, -180 + pm_w, 'ro', markersize=8, markerfacecolor='none')
            ax.text(wc_w, -180 + pm_w + 10, f'PM={pm_w:.1f}°',
                    verticalalignment='bottom', horizontalalignment='center')
    else:
        # Hide empty subplots
        axes[1, 0].axis('off')
        axes[1, 1].axis('off')
        axes[1, 0].text(0.5, 0.5, 'Speed loop analysis\nnot available\n(Current loop failed)',
                        ha='center', va='center', transform=axes[1, 0].transAxes)
        axes[1, 1].text(0.5, 0.5, 'Speed loop analysis\nnot available\n(Current loop failed)',
                        ha='center', va='center', transform=axes[1, 1].transAxes)

    plt.tight_layout()
    plt.show()

    # ============================================
    # 4. PRINT STABILITY REPORT
    # ============================================
    print("\n" + "=" * 80)
    print("STABILITY REPORT")
    print("=" * 80)

    print("\nREQUIREMENTS CHECKLIST:")
    print("-" * 40)

    checks = []

    # Current loop checks
    if pm_i is not None:
        checks.append(("Current loop PM > 45°", pm_i > 45))
        checks.append(("Current loop PM < 70°", pm_i < 70))
        if f_crossover_i is not None:
            checks.append(("Current BW > 500 Hz", f_crossover_i > 500))

    # Speed loop checks
    if pm_w is not None:
        checks.append(("Speed loop PM > 45°", pm_w > 45))
        if bandwidth_ratio is not None:
            checks.append(("Bandwidth ratio > 5", bandwidth_ratio > 5))

    for check_name, passed in checks:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {check_name}")

    print("\nRECOMMENDATIONS:")
    print("-" * 40)

    if pm_i is not None:
        if pm_i < 45:
            print("1. Reduce current Ki to increase phase margin")
        elif pm_i > 70:
            print("1. Increase current Kp to reduce phase margin (faster response)")

    if pm_w is not None:
        if pm_w < 45:
            print("2. Reduce speed Ki or add derivative action")

        if bandwidth_ratio is not None and bandwidth_ratio < 5:
            print("3. Make current loop faster or speed loop slower")

    # Return results
    results = {
        'current_pm': pm_i,
        'current_bw': f_crossover_i,
        'speed_pm': pm_w,
        'speed_bw': f_crossover_w,
        'bandwidth_ratio': bandwidth_ratio
    }

    return results


# ============================================
# USAGE
# ============================================
if __name__ == "__main__":
    params = MotorParams()

    # Run stability analysis
    print("Running stability analysis with gains:")
    print(f"  Speed: Kp={0.123660}, Ki={7.895680}")
    print(f"  Current: Kp={3.141593}, Ki={15707.96}")

    results = analyze_stability_offline(
        params=params,
        Kp_speed=0.123660,
        Ki_speed=7.895680,
        Kp_current=3.141593,
        Ki_current=15707.96
    )

    # Save results
    with open('stability_report.txt', 'w') as f:
        f.write("STABILITY ANALYSIS REPORT\n")
        f.write("=" * 50 + "\n")
        f.write(f"Current Loop Phase Margin: {results['current_pm']:.1f}°\n")
        f.write(f"Current Loop Bandwidth: {results['current_bw']:.1f} Hz\n")
        f.write(f"Speed Loop Phase Margin: {results['speed_pm']:.1f}°\n")
        f.write(f"Speed Loop Bandwidth: {results['speed_bw']:.1f} Hz\n")
        f.write(f"Bandwidth Ratio: {results['bandwidth_ratio']:.1f}\n")

    print("\nReport saved to 'stability_report.txt'")