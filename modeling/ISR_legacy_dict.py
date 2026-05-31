current_params = {
    "Kp": 2.0,
    "Ki": 200.0,
    "Ts": 1e-4,      # 10 kHz
    "u_min": -24.0,
    "u_max":  24.0,
    "Tt": 0.01
}

speed_params = {
    "Kp": 0.5,
    "Ki": 50.0,
    "Ts": 1e-3,      # 1 kHz
    "u_min": -5.0,
    "u_max":  5.0,
    "Tt": 0.05
}


def pi_step(state, error, params):
    """
    ONE clock tick of a PI with back-calculation anti-windup
    """

    # unpack state
    integral = state["integral"]

    # proportional
    p = params["Kp"] * error

    # integrate
    integral_next = integral + params["Ki"] * params["Ts"] * error

    # unsaturated output
    u_unsat = p + integral_next

    # saturation
    u = max(params["u_min"], min(params["u_max"], u_unsat))

    # anti-windup
    if u != u_unsat:
        integral_next -= (1.0 / params["Tt"]) * (u_unsat - u) * params["Ts"]

    # pack next state
    next_state = {
        "integral": integral_next
    }

    return next_state, u


def speed_step(state, w_ref, w_meas, params):
    """
    Runs at 1 kHz
    """

    error = w_ref - w_meas

    next_state, i_ref = pi_step(
        state,
        error,
        params
    )

    return next_state, i_ref
def current_step(state, i_ref, i_meas, params):
    """
    Runs at 10 kHz
    """

    error = i_ref - i_meas

    next_state, v_cmd = pi_step(
        state,
        error,
        params
    )

    return next_state, v_cmd
# ================= RESET (FPGA reset) =================

speed_state   = {"integral": 0.0}
current_state = {"integral": 0.0}

current_ref = 0.0
slow_cnt = 0

# ================= CLOCKED LOGIC ======================

def clock_tick(w_ref, w_meas, i_meas):
    global speed_state, current_state, current_ref, slow_cnt

    # ---- FAST LOOP (10 kHz) ----

    current_state, v_cmd = current_step(
        current_state,
        current_ref,
        i_meas,
        current_params
    )

    # ---- SLOW LOOP (1 kHz) ----
    slow_cnt += 1
    if slow_cnt == 10:
        slow_cnt = 0

        speed_state, current_ref = speed_step(
            speed_state ,
            i_meas ,
            w_meas ,
            speed_params
        )
    return v_cmd

w_meas = 0.0
i_meas = 0.0
w_ref = 100.0

for n in range(100_000):   # 10 kHz ticks
    v_cmd = clock_tick(w_ref, w_meas, i_meas)

    # plant update (VERY simple for now)
    i_meas += 0.01 * (v_cmd - i_meas)
    w_meas += 0.001 * (i_meas - w_meas)
