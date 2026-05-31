import numpy as np

# =============================================================================
# CONSTANTS (ROM) – Never change in hardware
# =============================================================================
CURRENT_KP = 3.141593
CURRENT_KI = 15707.96
CURRENT_TS = 1e-4  # 10 kHz
CURRENT_U_MIN = -24.0
CURRENT_U_MAX = 24.0
CURRENT_TT = 0.0001  # anti‑windup time constant

SPEED_KP = 0.123660
SPEED_KI = 7.895680
SPEED_TS = 1e-3  # 1 kHz
SPEED_U_MIN = -5.0
SPEED_U_MAX = 5.0
SPEED_TT = 0.001566  # from Ti/10

# motor parameters
MOTOR_R = 2.5  # Resistance (Ω)
MOTOR_L = 0.0005  # Inductance (H)
MOTOR_J = 0.0001  # Inertia (kg·m²)
MOTOR_B = 0.0001  # Viscous friction (N·m·s/rad)
MOTOR_KT = 0.05  # Torque constant (N·m/A)
MOTOR_KE = 0.05  # Back-EMF constant (V·s/rad)

# =============================================================================
# REGISTER FILE – All persistent state (global variables)
# These are REGISTERS in FPGA - they hold state between clock cycles
# =============================================================================
# ---- Plant simulation (would be ADC/encoder registers in real FPGA) ----
w_meas = 0.0  # REG: measured speed [rad/s] from encoder
i_meas = 0.0  # REG: measured current [A] from ADC
w_ref = 100.0  # REG: speed setpoint [rad/s] (written by trajectory generator)

# ---- Delayed measurements (pipeline registers - 1 clock delay) ----
i_meas_delayed = 0.0  # REG: previous cycle current (ADC pipeline delay)
w_meas_delayed = 0.0  # REG: previous cycle speed (encoder pipeline delay)

# ---- Controller state (accumulator registers) ----
speed_integral = 0.0  # REG: speed PI integrator accumulator
current_integral = 0.0  # REG: current PI integrator accumulator
current_ref = 0.0  # REG: current setpoint [A] (output of speed loop)
slow_cnt = 0  # REG: downsampling counter (divides 10 kHz → 1 kHz)

# ---- Output register ----
v_cmd = 0.0  # REG: voltage command to PWM [V] (output register)

# ===== FILTER CONSTANTS (ROM - Read-Only Memory) =====
SPEED_FILTER_TC = 0.01  # time constant [s]
SPEED_REF_FILTER_TC = 0.050  # 50ms prefilter

ALPHA_SPEED = np.exp(-SPEED_TS / SPEED_FILTER_TC)  # 1 kHz filter coefficient
ALPHA_REF = np.exp(-SPEED_TS / SPEED_REF_FILTER_TC)  # 1 kHz filter coefficient

# ===== FILTER STATE REGISTERS =====
speed_filter_prev = 0.0  # REG: previous filtered speed (filter state register)
ref_filter_prev = 0.0  # REG: previous filtered reference (filter state register)


# ===== FILTER PURE FUNCTION (Combinational Logic) =====
def first_order_filter(prev, input_val, alpha):
    """
    First-order IIR filter (combinational logic)
    y[n] = alpha*y[n-1] + (1-alpha)*x[n]

    Args:
        prev: REG (previous output, from register)
        input_val: WIRE (current input, combinational)
        alpha: CONST (filter coefficient, ROM)

    Returns:
        (new_state, filtered_value): both are WIRES (combinational outputs)
    """
    output = alpha * prev + (1 - alpha) * input_val  # WIRE: combinational calculation
    return output, output  # (new_state, filtered_value) - both WIRES


# =============================================================================
# PURE COMBINATIONAL LOGIC (WIRES) – No state, only computation
# In FPGA: This is implemented as combinational logic or DSP blocks
# =============================================================================
def pi_step(integral, error, Kp, Ki, Ts, u_min, u_max, Tt):
    """
    One clock tick of a PI controller with back‑calculation anti‑windup.
    This is PURE COMBINATIONAL LOGIC - no internal state.

    Args:
        integral: REG (current integrator value from register)
        error: WIRE (error signal, combinational)
        Kp, Ki, Ts, u_min, u_max, Tt: CONST (from ROM)

    Returns:
        (new_integral, output): both WIRES (combinational outputs)
        - new_integral will be stored back to REG on clock edge
        - output goes directly to next stage (wire)
    """
    # All of these are WIRES (combinational signals within one clock cycle)
    p = Kp * error  # WIRE: proportional term
    integral_next = integral + Ki * Ts * error  # WIRE: new integrator value
    u_unsat = p + integral_next  # WIRE: unsaturated output
    u = max(u_min, min(u_max, u_unsat))  # WIRE: saturated output

    # Anti‑windup (only active if saturated) - still combinational
    if u != u_unsat:
        integral_next -= (1.0 / Tt) * (u_unsat - u) * Ts  # WIRE: modified integrator

    return integral_next, u  # Both WIRES (will be latched to registers on clock edge)


def speed_controller():
    """
    Speed PI controller – runs at 1 kHz.

    READS from REGISTERS:
        - w_ref (global REG)
        - speed_integral (global REG)
        - ref_filter_prev (global REG)
        - speed_filter_prev (global REG)

    WRITES to REGISTERS:
        - speed_integral (global REG)
        - current_ref (global REG)
        - ref_filter_prev (global REG)

    All local variables are WIRES (combinational)
    """
    global speed_integral, current_ref, ref_filter_prev, speed_filter_prev

    # ---- Filter reference (prefilter) ----
    # Function call produces WIRES (combinational outputs)
    ref_filtered, ref_filter_prev = first_order_filter(
        ref_filter_prev,  # REG: previous filter state (read)
        w_ref,  # REG: current setpoint (read)
        ALPHA_REF  # CONST: filter coefficient
    )
    # ref_filtered is a WIRE (combinational)
    # ref_filter_prev is updated (REG write happens here via assignment)

    # ---- PI on filtered signals ----
    error = ref_filtered - speed_filter_prev  # WIRE: error calculation (combinational)

    # PI controller (pure combinational logic)
    new_integral, i_ref = pi_step(
        speed_integral,  # REG: current integrator value (read)
        error,  # WIRE: error signal
        SPEED_KP, SPEED_KI, SPEED_TS,  # CONST: PI gains
        SPEED_U_MIN, SPEED_U_MAX, SPEED_TT  # CONST: limits
    )
    # new_integral is a WIRE (combinational output from pi_step)
    # i_ref is a WIRE (combinational output from pi_step)

    # Write back to REGISTERS (on clock edge in real hardware)
    speed_integral = new_integral  # REG write: update integrator
    current_ref = i_ref  # REG write: update current reference


def current_controller():
    """
    Current PI controller – runs at 10 kHz.

    READS from REGISTERS:
        - current_integral (global REG)
        - current_ref (global REG)
        - i_meas (global REG - actually i_meas_delayed via clock_tick logic)

    WRITES to REGISTERS:
        - current_integral (global REG)
        - v_cmd (global REG)

    All locals are WIRES
    """
    global current_integral, v_cmd

    error = current_ref - i_meas  # WIRE: error calculation (combinational)

    # PI controller (pure combinational logic)
    new_integral, voltage = pi_step(
        current_integral,  # REG: current integrator value (read)
        error,  # WIRE: error signal
        CURRENT_KP, CURRENT_KI, CURRENT_TS,  # CONST: PI gains
        CURRENT_U_MIN, CURRENT_U_MAX,  # CONST: limits
        CURRENT_TT  # CONST: anti-windup TC
    )
    # new_integral is a WIRE (combinational)
    # voltage is a WIRE (combinational)

    # Register updates (written on clock edge)
    current_integral = new_integral  # REG write: update integrator
    v_cmd = voltage  # REG write: update voltage command


def clock_tick():
    """
    Main clock function – called every 100 µs (10 kHz).
    This represents ONE CLOCK CYCLE in the FPGA.

    Sequence within one clock cycle:
    1. Read from registers (inputs)
    2. Combinational logic executes
    3. Write to registers (outputs) - happens at END of cycle

    NOW WITH 1-SAMPLE MEASUREMENT DELAY
    """
    global slow_cnt, speed_filter_prev
    global i_meas, w_meas, i_meas_delayed, w_meas_delayed

    # ===== USE DELAYED MEASUREMENTS (simulates ADC pipeline delay) =====
    # In real FPGA: ADC data arrives 1 cycle late due to conversion time
    # Controller sees measurements from PREVIOUS cycle
    i_meas_used = i_meas_delayed  # WIRE: use delayed register value
    w_meas_used = w_meas_delayed  # WIRE: use delayed register value

    # ---- FILTER SPEED (10 kHz) – always runs ----
    # Combinational logic that produces wires
    speed_filtered, speed_filter_prev = first_order_filter(
        speed_filter_prev,  # REG: previous filter state (read)
        w_meas_used,  # WIRE: delayed measurement
        ALPHA_SPEED  # CONST: filter coefficient
    )
    # speed_filtered is a WIRE (not stored, just used for feedback)
    # speed_filter_prev is updated REG (written here)

    # ---- FAST LOOP (10 kHz) – always runs ----
    # Temporarily swap globals so current controller sees delayed measurement
    # (In real FPGA: this would be a MUX selecting delayed vs. current)
    i_meas_temp = i_meas  # WIRE: save current value
    i_meas = i_meas_used  # REG: temporarily use delayed value
    current_controller()  # Executes combinational logic, updates REGs
    i_meas = i_meas_temp  # REG: restore current value

    # ---- SLOW LOOP (1 kHz) – runs every 10th tick ----
    slow_cnt += 1  # REG: increment counter
    if slow_cnt >= 10:  # WIRE: comparison (combinational)
        slow_cnt = 0  # REG: reset counter
        speed_controller()  # Executes combinational logic, updates REGs

    # ===== UPDATE DELAY REGISTERS (at end of cycle) =====
    # In real FPGA: these are D flip-flops that capture on clock edge
    # Next cycle will see these values as i_meas_delayed and w_meas_delayed
    i_meas_delayed = i_meas  # REG write: pipeline register
    w_meas_delayed = w_meas  # REG write: pipeline register


def reset():
    """
    Bring all registers to a known initial state.
    In FPGA: This is the reset signal that clears all flip-flops.
    """
    global w_meas, i_meas, w_ref, speed_integral, current_integral, current_ref
    global slow_cnt, v_cmd, speed_filter_prev, ref_filter_prev
    global i_meas_delayed, w_meas_delayed

    # Reset all REGISTERS to zero
    w_meas = 0.0  # REG: encoder measurement
    i_meas = 0.0  # REG: ADC measurement
    w_ref = 0.0  # REG: setpoint
    speed_integral = 0.0  # REG: speed integrator
    current_integral = 0.0  # REG: current integrator
    current_ref = 0.0  # REG: current reference
    slow_cnt = 0  # REG: downsample counter
    v_cmd = 0.0  # REG: voltage output
    speed_filter_prev = 0.0  # REG: speed filter state
    ref_filter_prev = 0.0  # REG: reference filter state
    i_meas_delayed = 0.0  # REG: pipeline delay
    w_meas_delayed = 0.0  # REG: pipeline delay