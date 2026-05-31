import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# Motor & Drive parameters
# ---------------------------
R = 2.0           # Ohm
L = 0.5e-3        # H
Ke = 0.1          # V/(rad/s)
Kt = 0.1          # Nm/A
J = 1e-3          # kg.m^2
b = 1e-4          # viscous friction N.m.s
Fc = 0.01         # Coulomb friction torque (N.m)
Vdc = 24.0        # DC bus voltage (V)
TL0 = 0.0         # base load torque

# ---------------------------
# Simulation parameters
# ---------------------------
Tsim = 0.6         # seconds
dt = 1e-5          # simulation time step (10 microseconds) - fine for PWM switching
N = int(Tsim/dt)

# Control sample rates
fsw = 20000        # PWM switching frequency (Hz)
Ts_pwm = 1/fsw
Ts_current = 1e-4  # current controller sample time (100 us)
Ts_speed = 1e-3    # speed controller sample time (1 ms)
adc_fs = 10000     # ADC sampling rate for current (Hz)
Ts_adc = 1/adc_fs

# Anti-windup back-calculation constant
K_aw_current = 50.0
K_aw_speed = 10.0

# Desired bandwidths for auto tuning (rad/s)
#"How fast can the controller respond to changes?"
#Controller Bandwidth → DYNAMICS
#Determines: How fast controller reacts to true signals

#Location: Inside the PID algorithm

#Effect: Changes PI gains

#Goal: Make controller fast/slow based on physical system
wc_current_des = 2*np.pi*1000  # 1 kHz
wc_speed_des = 2*np.pi*50  # 50 Hz

# ---------------------------
# Automatic (heuristic) PI tuning
# ---------------------------
def auto_tune_current(L, R, wc):
    # Heuristic: for plant G(s)=1/(L s + R), choose PI such that
    # closed-loop bandwidth approximates wc.
    # This function returns a safe heuristic Kp, Ki.
    Kp = 0.8 * R * 1.0 / 1.0  # fallback safe value
    Ki = Kp * (wc/10.0)       # integrator break somewhat below wc
    return max(0.01, Kp), max(1e-6, Ki)

def auto_tune_speed(J, b, Kt, Ke, wc):
    # Plant from torque (Kt*i) to speed: G_w(s)=Kt/(J s + b)
    # Use simple DC gain approx K = Kt/b
    K = Kt / b
    Kp = 0.5 / K
    Ki = Kp * (wc/10.0)
    return max(1e-4, Kp), max(1e-6, Ki)

# Auto-tune (heuristic)
Kp_i_auto, Ki_i_auto = auto_tune_current(L, R, wc_current_des)
Kp_w_auto, Ki_w_auto = auto_tune_speed(J, b, Kt, Ke, wc_speed_des)

# Scale heuristics for reasonable speed/robustness
Kp_current = Kp_i_auto * 3.0
Ki_current = Ki_i_auto * 1.0

Kp_speed = Kp_w_auto * 2.5
Ki_speed = Ki_w_auto * 1.0

Kp_current = max(1e-4, Kp_current)
Ki_current = max(1e-6, Ki_current)
Kp_speed = max(1e-6, Kp_speed)
Ki_speed = max(1e-6, Ki_speed)

print("Tuned gains (heuristic):")
print("Current PI:  Kp =", Kp_current, " Ki =", Ki_current)
print("Speed PI:    Kp =", Kp_speed, " Ki =", Ki_speed)

# ---------------------------
# Controller state initialization
# ---------------------------
Ierr_speed = 0.0
Ierr_current = 0.0

# ---------------------------
# Observer (Luenberger) for [i, w]
# x_dot = A x + B V + E TL ; y = C x (we measure current only)
# ---------------------------
A = np.array([[-R/L, -Ke/L],
               [Kt/J, -b/J]])
B = np.array([1.0/L, 0.0]).reshape((2,1))
C = np.array([[1.0, 0.0]])  # measure current only

# Observer gain (manual heuristic)
L_gain = np.array([20000.0, 10000.0]).reshape((2,1))

# Observer state
xhat = np.array([0.0, 0.0])  # [i_hat, w_hat]

# ---------------------------
# Data arrays for plotting
# ---------------------------
time = np.linspace(0, Tsim, N)
speed = np.zeros(N)
current = np.zeros(N)
i_ref_hist = np.zeros(N)
vcmd_hist = np.zeros(N)
duty_hist = np.zeros(N)
speed_ref_hist = np.zeros(N)
obs_current = np.zeros(N)
obs_speed = np.zeros(N)
torque = np.zeros(N)
TL_hist = np.zeros(N)

# ---------------------------
# Helper: load torque profile (step + transient)
# ---------------------------
def TL_profile(t):
    # Step at t=0.25s, and a ramp between 0.4-0.5
    if t < 0.25:
        return 0.0
    elif t < 0.4:
        return 0.02  # 20 mNm
    elif t < 0.5:
        return 0.02 + (t-0.4)*(-0.015)  # ramp down
    else:
        return 0.005

# ---------------------------
# Speed reference (step + some changes)
# ---------------------------
def speed_ref(t):
    if t < 0.05:
        return 0.0
    elif t < 0.2:
        return 50.0  # rad/s
    elif t < 0.35:
        return 80.0
    else:
        return 30.0

# ---------------------------
# ADC quantization for current measurement
# ---------------------------
adc_bits = 12
adc_range = 30.0  # +/-30A range for current sensing
adc_lsb = (2*adc_range) / (2**adc_bits)
def adc_quantize(x):
    x = np.clip(x, -adc_range, adc_range)
    q = np.round(x/adc_lsb)*adc_lsb
    return q

# ---------------------------
# Simulation initial states
# ---------------------------
i = 0.0
w = 0.0

Ierr_speed = 0.0
Ierr_current = 0.0

next_current_time = 0.0
next_speed_time = 0.0
next_adc_time = 0.0

# PWM deadtime (fraction of period to remove from duty edges)
deadtime = 1e-6  # 1 microsecond deadtime

# initial measured current
i_meas = 0.0

# Start simulation
for k in range(N):
    t = k*dt
    # sample references and loads
    w_ref = speed_ref(t)
    TL = TL_profile(t)
    TL_hist[k] = TL
    speed_ref_hist[k] = w_ref

    # ADC sampling of current at adc_fs
    if t >= next_adc_time - 1e-15:
        i_meas = adc_quantize(i)  # measure actual current with quantization
        next_adc_time += Ts_adc

    # Speed controller (outer loop) runs at Ts_speed
    if t >= next_speed_time - 1e-15:
        # Compute speed error using observer estimate (sensorless-ish)
        error_speed = w_ref - xhat[1]
        Ierr_speed += error_speed * Ts_speed

        # PI output = current reference (A) - UNSATURATED
        i_ref_unsat = Kp_speed * error_speed + Ki_speed * Ierr_speed

        # SATURATE the current reference
        i_ref = np.clip(i_ref_unsat, -30.0, 30.0)

        # ⭐⭐⭐ ANTI-WINDUP FOR SPEED LOOP ⭐⭐⭐
        Ierr_speed += K_aw_speed * (i_ref - i_ref_unsat) * Ts_speed / max(1e-6, Kp_speed)
    i_ref_hist[k] = i_ref

    # Current controller runs at Ts_current
    if t >= next_current_time - 1e-15:
        # Current error uses measured current (with ADC quantization)
        error_i = i_ref - i_meas
        Ierr_current += error_i * Ts_current
        # PI to compute voltage command (unsaturated)
        v_unsat = Kp_current * error_i + Ki_current * Ierr_current
        duty_unsat = v_unsat / Vdc
        duty_sat = np.clip(duty_unsat, -1.0, 1.0)
        # anti-windup back-calculation for current integrator
        # Anti-windup does NOTHING:
        # += 50.0 * (0.00) * Ts_current * Vdc / Kp_current
        #                            ↑
        #                         Zero → NO EFFECT on integrator!
        Ierr_current += K_aw_current * (duty_sat - duty_unsat) * Ts_current * Vdc / max(1e-6, Kp_current)
        vcmd = duty_sat * Vdc
        duty = duty_sat
        next_current_time += Ts_current

    vcmd_hist[k] = vcmd
    duty_hist[k] = duty

    # PWM switching: simple bipolar PWM producing instantaneous voltage applied to motor
    pwm_phase = (t % Ts_pwm) / Ts_pwm
    if duty >= 0:
        on_fraction = duty
        on_fraction = np.clip(on_fraction - deadtime*fsw, 0.0, 1.0)
        motor_V = Vdc if pwm_phase < on_fraction else -Vdc
    else:
        on_fraction = -duty
        on_fraction = np.clip(on_fraction - deadtime*fsw, 0.0, 1.0)
        motor_V = -Vdc if pwm_phase < on_fraction else Vdc

    # Motor differential equations (Euler integration)...............................
    di_dt = (motor_V - R*i - Ke*w)/L
    w_sign = np.tanh(w*1000.0)  # smooth sign function to avoid discontinuity
    T_friction = b*w + Fc * w_sign
    dw_dt = (Kt * i - T_friction - TL)/J

    i += di_dt * dt
    w += dw_dt * dt

    # Observer update (forward-Euler).......................
    y = i_meas
    xhat_dot = A.dot(xhat) + (B.flatten()*motor_V) + (L_gain.flatten() * (y - C.dot(xhat)))
    xhat += xhat_dot * dt

    # store results
    current[k] = i
    speed[k] = w
    obs_current[k] = xhat[0]
    obs_speed[k] = xhat[1]
    torque[k] = Kt * i

# ---------------------------
# Plots
# ---------------------------
plt.rcParams["figure.figsize"] = (14,10)

plt.subplot(3,2,1)
plt.plot(time, speed_ref_hist, label="speed_ref")
plt.plot(time, speed, label="speed (true)")
plt.plot(time, obs_speed, '--', label="speed (observer)")
plt.title("Motor Speed")
plt.xlabel("Time (s)"); plt.ylabel("rad/s"); plt.legend()

plt.subplot(3,2,2)
plt.plot(time, current, label="i (true)")
plt.plot(time, i_ref_hist, label="i_ref")
plt.plot(time, obs_current, '--', label="i_hat")
plt.title("Current (A)")
plt.xlabel("Time (s)"); plt.ylabel("A"); plt.legend()

plt.subplot(3,2,3)
plt.plot(time, vcmd_hist)
plt.title("Voltage Command (vcmd)")
plt.xlabel("Time (s)"); plt.ylabel("V")

plt.subplot(3,2,4)
plt.plot(time, duty_hist)
plt.title("PWM Duty (-1..1)")
plt.xlabel("Time (s)"); plt.ylabel("duty")

plt.subplot(3,2,5)
plt.plot(time, torque, label="electromagnetic torque")
plt.plot(time, TL_hist, label="load torque")
plt.title("Torques (Nm)")
plt.xlabel("Time (s)"); plt.ylabel("Nm"); plt.legend()

plt.subplot(3,2,6)
plt.plot(time, speed - obs_speed, label="speed estimation error")
plt.plot(time, current - obs_current, label="current estimation error")
plt.title("Observer Estimation Errors")
plt.xlabel("Time (s)"); plt.legend()

plt.tight_layout()
plt.show()

# ---------------------------
# Print final stats
# ---------------------------
print("Final speed (rad/s):", speed[-1])
print("Final current (A):", current[-1])
