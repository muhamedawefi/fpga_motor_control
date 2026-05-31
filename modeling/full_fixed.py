# =============================================================================
# FULL FIXED-POINT ISR MODULE
# =============================================================================
# Everything: INTEGER arithmetic (bit-exact HDL)
# Structure: Pure ISR style (import and call clock_tick())
# =============================================================================

import numpy as np

# =============================================================================
# FIXED-POINT PARAMETERS
# =============================================================================
SPEED_SCALE = 256   # REG: Q9.7 (was Q8.8)
CURRENT_SCALE = 4096  # 2^12 for Q4.12
VOLTAGE_SCALE = 1024  # 2^10 for Q6.10
SPEED_INT_SCALE = 65536  # 2^16 for Q8.16
CURRENT_INT_SCALE = 256  # 2^8 for Q16.8

# =============================================================================
# CONSTANTS - Fixed-Point Integers
# =============================================================================
SPEED_KP_RAW = 8101  # Q0.16: 0.123660 * 2^16
SPEED_KI_RAW = 32334  # Q4.12: 7.895680 * 2^12  ........
SPEED_TS_RAW = 1049  # Q0.20: 0.001 * 2^20
SPEED_TT_INV_RAW = 638  # Q16.0: 1/0.001566

CURRENT_KP_RAW = 51472  # Q2.14: 3.141593 * 2^14
CURRENT_KI_RAW = 62832  # Q14.2: 15707.96 * 2^2
CURRENT_TS_RAW = 105  # Q0.20: 0.0001 * 2^20
CURRENT_TT_INV_RAW = 10000  # Q16.0: 1/0.0001

SPEED_U_MIN_RAW = -20480  # Q4.12: -5.0 * 4096
SPEED_U_MAX_RAW = 20480  # Q4.12: +5.0 * 4096
CURRENT_U_MIN_RAW = -24576  # Q6.10: -24.0 * 1024
CURRENT_U_MAX_RAW = 24576  # Q6.10: +24.0 * 1024

ALPHA_SPEED_RAW = 59294  # Q0.16: 0.904837 * 2^16
ALPHA_REF_RAW = 64238  # Q0.16: 0.980199 * 2^16

# =============================================================================
# REGISTER FILE - Integers
# =============================================================================
w_meas_raw = 0   # REG: Q9.7 (was Q8.8)
i_meas_raw = 0  # REG: Q4.12
w_ref_raw = 0   # REG: Q9.7 (was Q8.8)

w_meas_delayed_raw = 0   # REG: Q9.7 (was Q8.8)
i_meas_delayed_raw = 0  # REG: Q4.12

speed_integral_raw = 0  # REG: Q8.16
current_integral_raw = 0  # REG: Q16.8
current_ref_raw = 0  # REG: Q4.12

speed_filter_prev_raw = 0  # REG: Q9.7 (was Q8.8)
ref_filter_prev_raw = 0   # REG: Q9.7 (was Q8.8)

v_cmd_raw = 0  # REG: Q6.10
slow_cnt = 0  # REG: counter


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================
def saturate(value, min_val, max_val):
    """Saturate integer value"""
    return max(min_val, min(max_val, int(value)))


def multiply_fixed(a_raw, b_raw, a_frac, b_frac, out_frac):
    """Multiply two fixed-point integers with rounding"""
    product = int(a_raw) * int(b_raw)
    shift = a_frac + b_frac - out_frac

    if shift > 0:
        # Round: add half LSB before shifting
        result = (product + (1 << (shift - 1))) >> shift
    elif shift < 0:
        result = product << (-shift)
    else:
        result = product

    return int(result)


# =============================================================================
# COMBINATIONAL LOGIC (WIRES)
# =============================================================================
def filter_fixed(prev_raw, input_raw, alpha_raw, frac_bits):
    """First-order IIR filter (integer)"""
    # alpha * prev: Q0.16 × Q9.7 → Q9.7
    term1 = multiply_fixed(alpha_raw, prev_raw, 16, frac_bits, frac_bits)

    # (1-alpha) * input
    one_minus_alpha = (1 << 16) - alpha_raw
    term2 = multiply_fixed(one_minus_alpha, input_raw, 16, frac_bits, frac_bits)

    return int(term1 + term2)


def pi_step_fixed(integral_raw, error_raw,
                  Kp_raw, Ki_raw, Ts_raw, Tt_inv_raw,
                  u_min_raw, u_max_raw,
                  error_frac, integral_frac, output_frac,
                  Kp_frac, Ki_frac, Ts_frac):
    """PI controller (pure integer)"""

    # Proportional: Kp * error
    p_raw = multiply_fixed(Kp_raw, error_raw, Kp_frac, error_frac, output_frac)

    # Integral update: Ki * error
    ki_error = multiply_fixed(Ki_raw, error_raw, Ki_frac, error_frac, integral_frac)
    # (Ki * error) * Ts
    delta_i = multiply_fixed(ki_error, Ts_raw, integral_frac, Ts_frac, integral_frac)
    integral_next_raw = integral_raw + delta_i

    # Unsaturated output: P + I (align fractional bits!)
    if integral_frac > output_frac:
        integral_shifted = integral_next_raw >> (integral_frac - output_frac)
    elif integral_frac < output_frac:
        integral_shifted = integral_next_raw << (output_frac - integral_frac)
    else:
        integral_shifted = integral_next_raw

    u_unsat_raw = p_raw + integral_shifted
    u_sat_raw = saturate(u_unsat_raw, u_min_raw, u_max_raw)

    # Anti-windup
    if u_sat_raw != u_unsat_raw:
        sat_error = u_unsat_raw - u_sat_raw
        correction = multiply_fixed(Tt_inv_raw, sat_error, 0, output_frac, output_frac)
        correction_scaled = multiply_fixed(correction, Ts_raw, output_frac, Ts_frac, integral_frac)
        integral_next_raw = integral_next_raw - correction_scaled

    return int(integral_next_raw), int(u_sat_raw)


# =============================================================================
# CONTROLLERS
# =============================================================================
def speed_controller():
    """Speed PI controller (full fixed-point)"""
    global speed_integral_raw, current_ref_raw, ref_filter_prev_raw

    # Filter reference (Q9.7
    ref_filtered_raw = filter_fixed(ref_filter_prev_raw, w_ref_raw,
                                    ALPHA_REF_RAW, 7)
    ref_filter_prev_raw = ref_filtered_raw

    # Error
    error_raw = ref_filtered_raw - speed_filter_prev_raw

    # PI controller
    speed_integral_raw, current_ref_raw = pi_step_fixed(
        speed_integral_raw, error_raw,
        SPEED_KP_RAW, SPEED_KI_RAW, SPEED_TS_RAW, SPEED_TT_INV_RAW,
        SPEED_U_MIN_RAW, SPEED_U_MAX_RAW,
        error_frac=7, integral_frac=15, output_frac=12,
        Kp_frac=16, Ki_frac=12, Ts_frac=20
    )


def current_controller():
    """Current PI controller (full fixed-point)"""
    global current_integral_raw, v_cmd_raw

    # Error
    error_raw = current_ref_raw - i_meas_raw

    # PI controller
    current_integral_raw, v_cmd_raw = pi_step_fixed(
        current_integral_raw, error_raw,
        CURRENT_KP_RAW, CURRENT_KI_RAW, CURRENT_TS_RAW, CURRENT_TT_INV_RAW,
        CURRENT_U_MIN_RAW, CURRENT_U_MAX_RAW,
        error_frac=12, integral_frac=8, output_frac=10,
        Kp_frac=14, Ki_frac=2, Ts_frac=20
    )


def clock_tick():
    """Main clock function (full fixed-point)"""
    global slow_cnt, speed_filter_prev_raw
    global w_meas_raw, i_meas_raw, w_meas_delayed_raw, i_meas_delayed_raw

    # Use delayed measurements
    w_meas_used_raw = w_meas_delayed_raw
    i_meas_used_raw = i_meas_delayed_raw

    # Filter speed (Q9.7)
    speed_filtered_raw = filter_fixed(speed_filter_prev_raw, w_meas_used_raw,
                                      ALPHA_SPEED_RAW, 7)
    speed_filter_prev_raw = speed_filtered_raw

    # Current loop (10 kHz)
    current_controller()

    # Speed loop (1 kHz)
    slow_cnt += 1
    if slow_cnt >= 10:
        slow_cnt = 0
        speed_controller()

    # Update delay registers
    w_meas_delayed_raw = w_meas_raw
    i_meas_delayed_raw = i_meas_raw


def reset():
    """Reset all registers"""
    global w_meas_raw, i_meas_raw, w_ref_raw
    global w_meas_delayed_raw, i_meas_delayed_raw
    global speed_integral_raw, current_integral_raw, current_ref_raw
    global speed_filter_prev_raw, ref_filter_prev_raw, v_cmd_raw
    global slow_cnt

    w_meas_raw = 0
    i_meas_raw = 0
    w_ref_raw = 0
    w_meas_delayed_raw = 0
    i_meas_delayed_raw = 0
    speed_integral_raw = 0
    current_integral_raw = 0
    current_ref_raw = 0
    speed_filter_prev_raw = 0
    ref_filter_prev_raw = 0
    v_cmd_raw = 0
    slow_cnt = 0