
# =============================================================================
# CONSTANTS (ROM) – Never change in hardware
# =============================================================================
CURRENT_KP = 2.0
CURRENT_KI = 200.0
CURRENT_TS = 1e-4       # 10 kHz
CURRENT_U_MIN = -24.0
CURRENT_U_MAX = 24.0
CURRENT_TT = 0.01       # anti‑windup time constant

SPEED_KP = 0.5
SPEED_KI = 50.0
SPEED_TS = 1e-3         # 1 kHz
SPEED_U_MIN = -5.0
SPEED_U_MAX = 5.0
SPEED_TT = 0.05

# motor parameters
MOTOR_R = 1.0      # Resistance (Ω)
MOTOR_L = 0.01     # Inductance (H)
MOTOR_J = 0.001    # Inertia (kg·m²)
MOTOR_B = 0.0001   # Viscous friction (N·m·s/rad)
MOTOR_KT = 0.1     # Torque constant (N·m/A)
MOTOR_KE = 0.1     # Back-EMF constant (V·s/rad)

# =============================================================================
# REGISTER FILE – All persistent state (global variables)
# =============================================================================
# ---- Plant simulation (would be ADC/encoder registers in real FPGA) ----
w_meas = 0.0        # REG: measured speed [rad/s]
i_meas = 0.0        # REG: measured current [A]
w_ref = 100.0       # REG: speed setpoint [rad/s] (written by trajectory)

# ---- Controller state ----
speed_integral = 0.0        # REG: speed PI integrator
current_integral = 0.0      # REG: current PI integrator
current_ref = 0.0           # REG: current setpoint [A] (from speed loop)
slow_cnt = 0                # REG: downsampling counter (1 kHz from 10 kHz)

# ---- Output ----
v_cmd = 0.0                 # REG: voltage command to PWM [V]

# =============================================================================
# PURE COMBINATIONAL LOGIC (WIRES) – No state, only computation
# =============================================================================
def pi_step(integral, error, Kp, Ki, Ts, u_min, u_max, Tt):
    """
    One clock tick of a PI controller with back‑calculation anti‑windup.
    All inputs and outputs are WIRES (except integral which is a register value).
    Returns (new_integral, output).
    """
    # Wires
    p = Kp * error
    integral_next = integral + Ki * Ts * error
    u_unsat = p + integral_next
    u = max(u_min, min(u_max, u_unsat))

    # Anti‑windup (only active if saturated)
    if u != u_unsat:
        integral_next -= (1.0 / Tt) * (u_unsat - u) * Ts

    return integral_next, u


def speed_controller():
    """
    Speed PI controller – runs at 1 kHz.
    Reads global w_ref, w_meas, speed_integral.
    Writes speed_integral and current_ref.
    All locals are WIRES.
    """
    global speed_integral, current_ref

    error = w_ref - w_meas                     # WIRE
    new_integral, i_ref = pi_step(
        speed_integral,
        error,
        SPEED_KP, SPEED_KI, SPEED_TS,
        SPEED_U_MIN, SPEED_U_MAX,
        SPEED_TT
    )
    # Register updates happen HERE (at the "clock edge")
    speed_integral = new_integral
    current_ref = i_ref


def current_controller():
    """
    Current PI controller – runs at 10 kHz.
    Reads global current_integral, current_ref, i_meas.
    Writes current_integral and v_cmd.
    All locals are WIRES.
    """
    global current_integral, v_cmd

    error = current_ref - i_meas               # WIRE
    new_integral, voltage = pi_step(
        current_integral,
        error,
        CURRENT_KP, CURRENT_KI, CURRENT_TS,
        CURRENT_U_MIN, CURRENT_U_MAX,
        CURRENT_TT
    )
    # Register updates
    current_integral = new_integral
    v_cmd = voltage


def clock_tick():
    """
    Main clock function – called every 100 µs (10 kHz).
    Reads global registers, writes global registers.
    No arguments, no return value (v_cmd is a global register).
    """
    global slow_cnt

    # ---- FAST LOOP (10 kHz) – always runs ----
    current_controller()

    # ---- SLOW LOOP (1 kHz) – runs every 10th tick ----
    slow_cnt += 1
    if slow_cnt >= 10:
        slow_cnt = 0
        speed_controller()


# =============================================================================
# SIMULATION LOOP – not part of hardware; tests the controller
# =============================================================================
# Plant model (simple first‑order approximations)
PLANT_CURRENT_GAIN = 0.01
PLANT_SPEED_GAIN = 0.001

# Logging
w_log = []
i_log = []
v_log = []
t_log = []
# Reset registers (already done above, but explicit for clarity)
w_meas = 0.0
i_meas = 0.0
w_ref = 100.0
speed_integral = 0.0
current_integral = 0.0
current_ref = 0.0
slow_cnt = 0
v_cmd = 0.0

# Run simulation
for n in range(100_000):          # 10 seconds at 10 kHz
    clock_tick()                  # controller step

    # Plant update (simulates physics – in FPGA this is the real motor)
    # Electrical dynamics: di/dt = (V - R*i - Ke*w)/L
    i_meas += (CURRENT_TS / MOTOR_L) * (v_cmd - MOTOR_R * i_meas - MOTOR_KE * w_meas)

    # Mechanical dynamics: dw/dt = (Kt*i - B*w - T_load)/J
    w_meas += (CURRENT_TS / MOTOR_J) * (MOTOR_KT * i_meas - MOTOR_B * w_meas - 0.0)  # T_load = 0
    # Log every 10th sample
    if n % 10 == 0:
        t_log.append(n * 1e-4)
        w_log.append(w_meas)
        i_log.append(i_meas)
        v_log.append(v_cmd)


# Optional: plot or analyze
# =============================================================================
# PRINT RESULTS TO CONSOLE
# =============================================================================
print("\n" + "=" * 60)
print("SIMULATION RESULTS - First 20 samples")
print("=" * 60)
print(f"{'Step':>6} {'Time(s)':>10} {'Speed':>10} {'Current':>10} {'Voltage':>10}")
print("-" * 60)

# Print first 20 logged samples
for i in range(min(20, len(t_log))):
    print(f"{i * 10:6d} {t_log[i]:10.4f} {w_log[i]:10.3f} {i_log[i]:10.3f} {v_log[i]:10.3f}")

print("\n" + "=" * 60)
print("FINAL STEADY STATE VALUES")
print("=" * 60)
print(f"Final speed    : {w_log[-1]:10.3f} rad/s  (target: {w_ref:.1f})")
print(f"Final current  : {i_log[-1]:10.3f} A")
print(f"Final voltage  : {v_log[-1]:10.3f} V")
print(f"Speed integral : {speed_integral:10.3f}")
print(f"Current integral: {current_integral:10.3f}")
print(f"Current ref    : {current_ref:10.3f} A")

# =============================================================================
# PLOT RESULTS (if matplotlib is available)
# =============================================================================
try:
    import matplotlib.pyplot as plt

    plt.figure(figsize=(12, 8))

    # Speed plot
    plt.subplot(3, 1, 1)
    plt.plot(t_log, w_log, 'b-', label='Speed (rad/s)')
    plt.axhline(y=w_ref, color='r', linestyle='--', label='Setpoint')
    plt.ylabel('Speed (rad/s)')
    plt.title('DC Motor Cascaded Control Response')
    plt.legend()
    plt.grid(True)

    # Current plot
    plt.subplot(3, 1, 2)
    plt.plot(t_log, i_log, 'g-', label='Current (A)')
    plt.ylabel('Current (A)')
    plt.legend()
    plt.grid(True)

    # Voltage plot
    plt.subplot(3, 1, 3)
    plt.plot(t_log, v_log, 'r-', label='Voltage (V)')
    plt.ylabel('Voltage (V)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    print("\n✓ Plot displayed successfully")

except ImportError:
    print("\n⚠ matplotlib not installed - skipping plots")
    print("  Install with: pip install matplotlib")

except Exception as e:
    print(f"\n⚠ Could not display plot: {e}")