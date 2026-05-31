import numpy as np

# =============================================================================
# CONSTANTS
# =============================================================================
CURRENT_KP = 3.141593
CURRENT_KI = 15707.96
CURRENT_TS = 1e-4  # 10 kHz
CURRENT_U_MIN = -24.0
CURRENT_U_MAX = 24.0
CURRENT_TT = 0.0001

SPEED_KP = 0.123660
SPEED_KI = 7.895680
SPEED_TS = 1e-2  # 100 Hz (FIXED!)
SPEED_U_MIN = -5.0
SPEED_U_MAX = 5.0
SPEED_TT = 0.001566

# Motor parameters
MOTOR_R = 2.5
MOTOR_L = 0.0005
MOTOR_J = 0.0001
MOTOR_B = 0.0001
MOTOR_KT = 0.05
MOTOR_KE = 0.05

# Filter constants
SPEED_FILTER_TC = 0.01  # 10ms for measurement
SPEED_REF_FILTER_TC = 1 / (3 * 2 * np.pi * 10)  # ~5.3ms for reference

# IMPORTANT: Filters still update at their designed rates
# Speed measurement filter at 10 kHz (fast)
ALPHA_SPEED_FAST = np.exp(-1e-4 / SPEED_FILTER_TC)  # For 10 kHz updates

# Reference filter at 100 Hz (matches speed loop)
ALPHA_REF = np.exp(-SPEED_TS / SPEED_REF_FILTER_TC)

# =============================================================================
# REGISTERS
# =============================================================================
w_meas = 0.0
i_meas = 0.0
w_ref = 100.0

speed_integral = 0.0
current_integral = 0.0
current_ref = 0.0
slow_cnt = 0

v_cmd = 0.0

speed_filter_prev = 0.0  # Updated at 10 kHz
ref_filter_prev = 0.0  # Updated at 100 Hz


# =============================================================================
# FUNCTIONS
# =============================================================================
def first_order_filter(prev, input_val, alpha):
    """y[n] = alpha*y[n-1] + (1-alpha)*x[n]"""
    output = alpha * prev + (1 - alpha) * input_val
    return output, output


def pi_step(integral, error, Kp, Ki, Ts, u_min, u_max, Tt):
    """PI controller with anti-windup"""
    p = Kp * error
    integral_next = integral + Ki * Ts * error
    u_unsat = p + integral_next
    u = max(u_min, min(u_max, u_unsat))

    if u != u_unsat:
        integral_next -= (1.0 / Tt) * (u_unsat - u) * Ts

    return integral_next, u


def speed_controller():
    """Speed PI controller – runs at 100 Hz"""
    global speed_integral, current_ref, ref_filter_prev, speed_filter_prev

    # Filter reference (at 100 Hz)
    ref_filtered, ref_filter_prev = first_order_filter(
        ref_filter_prev, w_ref, ALPHA_REF
    )

    # Use the filtered speed (updated at 10 kHz in clock_tick)
    error = ref_filtered - speed_filter_prev

    new_integral, i_ref = pi_step(
        speed_integral, error,
        SPEED_KP, SPEED_KI, SPEED_TS,
        SPEED_U_MIN, SPEED_U_MAX, SPEED_TT
    )
    speed_integral = new_integral
    current_ref = i_ref


def current_controller():
    """Current PI controller – runs at 10 kHz"""
    global current_integral, v_cmd

    error = current_ref - i_meas
    new_integral, voltage = pi_step(
        current_integral, error,
        CURRENT_KP, CURRENT_KI, CURRENT_TS,
        CURRENT_U_MIN, CURRENT_U_MAX, CURRENT_TT
    )
    current_integral = new_integral
    v_cmd = voltage


def clock_tick():
    """Main clock – 10 kHz (100 µs)"""
    global slow_cnt, speed_filter_prev

    # ---- FILTER SPEED at 10 kHz (always) ----
    speed_filtered, speed_filter_prev = first_order_filter(
        speed_filter_prev, w_meas, ALPHA_SPEED_FAST
    )

    # ---- CURRENT LOOP at 10 kHz (always) ----
    current_controller()

    # ---- SPEED LOOP at 100 Hz (every 100 ticks) ----
    slow_cnt += 1
    if slow_cnt >= 100:  # CHANGED from 10 to 100
        slow_cnt = 0
        speed_controller()


def reset():
    """Reset all registers"""
    global w_meas, i_meas, w_ref, speed_integral, current_integral
    global current_ref, slow_cnt, v_cmd, speed_filter_prev, ref_filter_prev

    w_meas = 0.0
    i_meas = 0.0
    w_ref = 0.0
    speed_integral = 0.0
    current_integral = 0.0
    current_ref = 0.0
    slow_cnt = 0
    v_cmd = 0.0
    speed_filter_prev = 0.0
    ref_filter_prev = 0.0