import os
import sys
import streamlit as st
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)


from backends.backend_factory import BackendFactory




# =========================
# MODE
# =========================
mode = st.sidebar.selectbox("Mode", ["Simulation", "UART", "Cocotb"])


# =========================
# BACKEND INIT
# =========================
if "mode" not in st.session_state:
    st.session_state.mode = mode

if "backend" not in st.session_state or st.session_state.mode != mode:

    try:
        st.session_state.backend = BackendFactory.create(mode)
    except Exception as e:
        st.warning(f"{e} → fallback to Simulation")
        st.session_state.backend = BackendFactory.create("Simulation")

    st.session_state.mode = mode

backend = st.session_state.backend


# =========================
# STATE
# =========================
if "last_speed" not in st.session_state:
    st.session_state.last_speed = None

if "running" not in st.session_state:
    st.session_state.running = False

if "data_log" not in st.session_state:
    st.session_state.data_log = {
        "speed": [],
        "current": [],
        "vcmd": [],
        "ref": []
    }


# =========================
# UI
# =========================
st.title("⚙️ FPGA Motor Control Platform")

st.sidebar.header("Control Panel")

speed_ref = st.sidebar.slider("Speed Reference (rad/s)", 0, 150, 50)
period = st.sidebar.slider("Stream Period (ms)", 1, 100, 10)


# send command only when changed
if st.session_state.last_speed != speed_ref:
    backend.set_speed(speed_ref)
    st.session_state.last_speed = speed_ref


col1, col2 = st.sidebar.columns(2)

if col1.button("Start"):
    st.session_state.running = True
    backend.start_stream(period)

if col2.button("Stop"):
    st.session_state.running = False
    backend.stop_stream()

# =========================
# DATA
# =========================
data = backend.get_data()

if data:
    for k in ["speed", "current", "vcmd", "ref"]:
        st.session_state.data_log[k].append(data[k])

    # limit buffer
    for k in st.session_state.data_log:
        st.session_state.data_log[k] = st.session_state.data_log[k][-200:]


# =========================
# DISPLAY
# =========================
if data:
    c1, c2, c3 = st.columns(3)

    c1.metric("Speed", f"{data['speed']:.2f}")
    c2.metric("Current", f"{data['current']:.2f}")
    c3.metric("Voltage", f"{data['vcmd']:.2f}")


st.line_chart({
    "Speed": st.session_state.data_log["speed"],
    "Reference": st.session_state.data_log["ref"]
})

st.line_chart({
    "Current": st.session_state.data_log["current"]
})

st.line_chart({
    "Voltage": st.session_state.data_log["vcmd"]
})


# =========================
# LOOP
# =========================
time.sleep(0.03)
st.rerun()