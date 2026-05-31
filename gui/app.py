import os
import sys
import streamlit as st
import time
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ── Path setup ───────────────────────────────────────────
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from backends.backend_factory import BackendFactory  # Your factory from code #2

# ── Constants ────────────────────────────────────────────
SAMPLE_PERIOD_MS = 30  # Match time.sleep(0.03) for smooth streaming
MAX_BUFFER = 400       # Live plot buffer
CAPTURE_MIN_SAMPLES = 50
CAPTURE_MAX_SAMPLES = 800
SETTLING_BAND_PCT = 0.05
SETTLING_CONSECUTIVE = 25  # Require 25/30 samples within band

# ── Page config + Styling ────────────────────────────────
st.set_page_config(
    page_title="FPGA Motor Control",
    page_icon="⚡",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.markdown("""
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');
html, body, [class*="css"] { font-family: 'Rajdhani', sans-serif; background-color: #080C10; }
.stApp { background-color: #080C10; }
.title-block {
    border-left: 4px solid #00FFD1; padding: 8px 20px; margin-bottom: 24px;
    background: linear-gradient(90deg, rgba(0,255,209,0.05) 0%, transparent 100%);
}
.title-block h1 {
    font-family: 'Rajdhani', sans-serif; font-weight: 700; font-size: 2.2rem;
    color: #E8F4F8; margin: 0; letter-spacing: 2px;
}
.title-block p { color: #4A7A8A; font-family: 'Share Tech Mono', monospace; font-size: 0.78rem; margin: 4px 0 0 0; }
.badge { display: inline-block; padding: 2px 10px; border-radius: 2px; font-family: 'Share Tech Mono', monospace; font-size: 0.7rem; }
.badge-live { background: rgba(0,255,100,0.15); color: #00FF64; border: 1px solid #00FF6440; }
.badge-idle { background: rgba(255,100,0,15); color: #FF6400; border: 1px solid #FF640040; }
.badge-capture { background: rgba(255,184,0,0.15); color: #FFB800; border: 1px solid #FFB80040; }
.metric-card {
    background: #0D1318; border: 1px solid #1A2830; border-top: 2px solid;
    padding: 16px 20px; border-radius: 4px; font-family: 'Share Tech Mono', monospace; margin-bottom: 8px;
}
.metric-card.speed { border-top-color: #00FFD1; }
.metric-card.current { border-top-color: #FF4B6E; }
.metric-card.vcmd { border-top-color: #FFB800; }
.metric-label { font-size: 0.68rem; color: #4A7A8A; letter-spacing: 2px; text-transform: uppercase; }
.metric-value { font-size: 2rem; font-weight: 700; color: #E8F4F8; margin: 4px 0; font-family: 'Rajdhani'; }
.metric-unit { font-size: 0.7rem; color: #4A7A8A; }
.metric-delta-pos { color: #00FF64; font-size: 0.75rem; }
.metric-delta-neg { color: #FF4B6E; font-size: 0.75rem; }
.perf-card {
    background: #0D1318; border: 1px solid #1A2830; padding: 14px 16px;
    border-radius: 4px; font-family: 'Share Tech Mono', monospace; text-align: center; margin-bottom: 8px;
}
.perf-label { font-size: 0.65rem; color: #4A7A8A; letter-spacing: 2px; text-transform: uppercase; }
.perf-value { font-size: 1.5rem; font-weight: 700; color: #00FFD1; font-family: 'Rajdhani'; }
.perf-unit { font-size: 0.65rem; color: #4A7A8A; }
.capture-status {
    font-family: 'Share Tech Mono', monospace; font-size: 0.8rem; color: #FFB800;
    text-align: center; padding: 12px; background: rgba(255,184,0,0.05);
    border: 1px solid rgba(255,184,0,0.2); border-radius: 4px; margin: 12px 0;
}
section[data-testid="stSidebar"] { background-color: #0A0F14; border-right: 1px solid #1A2830; }
section[data-testid="stSidebar"] * { color: #C8D8E0 !important; }
.stButton > button {
    background: transparent; border: 1px solid #1A3040; color: #00FFD1 !important;
    font-family: 'Share Tech Mono', monospace; font-size: 0.75rem;
    letter-spacing: 1px; border-radius: 2px; transition: all 0.2s; width: 100%; margin-bottom: 4px;
}
.stButton > button:hover { background: rgba(0,255,209,0.08); border-color: #00FFD1; }
hr { border-color: #1A2830 !important; }
</style>
""", unsafe_allow_html=True)

# ── Session State Init ───────────────────────────────────
defaults = {
    "mode": None,
    "backend": None,
    "running": False,
    "capturing": False,
    "last_speed": 0.0,
    "stream_buffer": {"speed": [], "current": [], "vcmd": [], "ref": [], "time_ms": []},
    "capture_buffer": {"speed": [], "current": [], "vcmd": [], "ref": [], "time_ms": []},
    "step_response": None,
    "capture_setpoint": 0.0,
    "stream_period_ms": 30,
}
for k, v in defaults.items():
    if k not in st.session_state:
        st.session_state[k] = v

# ── Mode Selection + Backend Init ────────────────────────
with st.sidebar:
    st.markdown("### 🎛 CONTROL PANEL")
    st.markdown("---")
    
    mode = st.selectbox("Backend Mode", ["Simulation", "UART", "Cocotb"])
    
    # Re-init backend if mode changed
    if st.session_state.mode != mode or st.session_state.backend is None:
        try:
            st.session_state.backend = BackendFactory.create(mode)
            st.session_state.mode = mode
            st.success(f"✓ Connected: {mode}")
        except Exception as e:
            st.warning(f"⚠ {e} → Fallback to Simulation")
            st.session_state.backend = BackendFactory.create("Simulation")
            st.session_state.mode = "Simulation"
    
    backend = st.session_state.backend
    
    # Speed reference
    speed_ref = st.slider("Speed Reference (rad/s)", -100.0, 100.0, st.session_state.last_speed, 0.5)
    if speed_ref != st.session_state.last_speed:
        backend.set_speed(speed_ref)
        st.session_state.last_speed = speed_ref
    
    # Stream period
    period = st.slider("Stream Period (ms)", 10, 100, st.session_state.stream_period_ms, 5)
    if period != st.session_state.stream_period_ms:
        st.session_state.stream_period_ms = period
        backend.start_stream(period)  # Update backend period if supported
    
    st.markdown("")
    
    # Mode buttons
    col1, col2 = st.columns(2)
    if col1.button("▶ LIVE"):
        st.session_state.running = True
        st.session_state.capturing = False
        st.session_state.step_response = None
        backend.start_stream(period)
    
    if col2.button("📈 CAPTURE"):
        st.session_state.capturing = True
        st.session_state.running = False
        st.session_state.step_response = None
        st.session_state.capture_buffer = {k: [] for k in defaults["capture_buffer"]}
        st.session_state.capture_setpoint = speed_ref
        backend.set_speed(speed_ref)
        backend.start_stream(period)
    
    if st.button("■ STOP"):
        st.session_state.running = False
        st.session_state.capturing = False
        backend.set_speed(0)
        st.session_state.last_speed = 0.0
        backend.stop_stream()
    
    if st.button("↺ CLEAR"):
        for k in ["stream_buffer", "capture_buffer"]:
            st.session_state[k] = {kb: [] for kb in defaults[k]}
        st.session_state.step_response = None
    
    st.markdown("---")
    
    # Export live data
    if st.session_state.stream_buffer["speed"]:
        import pandas as pd
        df_live = pd.DataFrame(st.session_state.stream_buffer)
        st.download_button("↓ Export LIVE CSV", df_live.to_csv(index=False), "live_telemetry.csv", "text/csv")
    
    st.markdown("---")
    st.markdown("""
    <div style='font-family: Share Tech Mono; font-size: 0.65rem; color: #2A4A5A; line-height: 1.8'>
    PLATFORM · FPGA Artix-7<br>
    CONTROL  · Cascade PI<br>
    COMM     · UART 115200 / ZMQ<br>
    SIM      · GHDL + Cocotb<br>
    </div>
    """, unsafe_allow_html=True)

# ── Header with Status Badge ─────────────────────────────
if st.session_state.capturing:
    n_cap = len(st.session_state.capture_buffer["speed"])
    badge = f'<span class="badge badge-capture">◉ CAPTURING {n_cap} samples</span>'
elif st.session_state.running:
    badge = '<span class="badge badge-live">● LIVE</span>'
else:
    badge = '<span class="badge badge-idle">○ IDLE</span>'

st.markdown(f"""
<div class="title-block">
    <h1>FPGA MOTOR CONTROL {badge}</h1>
    <p>REAL-TIME DC MOTOR SUPERVISION · VHDL CASCADE PI · ARTIX-7</p>
</div>
""", unsafe_allow_html=True)

# ── Data Ingestion ───────────────────────────────────────
data = None
if st.session_state.running or st.session_state.capturing:
    data = backend.get_data()

if data:
    # Normalize raw values (adjust divisors to match your FPGA fixed-point format)
    spd_val = data.get("speed_raw", data.get("speed", 0)) / 128
    cur_val = data.get("current_raw", data.get("current", 0)) / 4096
    vcmd_val = data.get("vcmd_raw", data.get("vcmd", 0)) / 1024
    ref_val = data.get("ref", st.session_state.last_speed)
    
    # Track elapsed time
    elapsed_ms = len(st.session_state.stream_buffer["time_ms"]) * st.session_state.stream_period_ms if st.session_state.running else len(st.session_state.capture_buffer["time_ms"]) * st.session_state.stream_period_ms
    
    if st.session_state.running:
        buf = st.session_state.stream_buffer
        buf["speed"].append(spd_val)
        buf["current"].append(cur_val)
        buf["vcmd"].append(vcmd_val)
        buf["ref"].append(ref_val)
        buf["time_ms"].append(elapsed_ms)
        for k in buf:
            if len(buf[k]) > MAX_BUFFER:
                buf[k] = buf[k][-MAX_BUFFER:]
    
    if st.session_state.capturing:
        buf = st.session_state.capture_buffer
        buf["speed"].append(spd_val)
        buf["current"].append(cur_val)
        buf["vcmd"].append(vcmd_val)
        buf["ref"].append(ref_val)
        buf["time_ms"].append(elapsed_ms)
        
        sp = st.session_state.capture_setpoint
        n = len(buf["speed"])
        
        # Robust settling detection
        if n > CAPTURE_MIN_SAMPLES and sp != 0:
            recent = buf["speed"][-30:] if n >= 30 else buf["speed"]
            errors = [abs(v - sp) / abs(sp) for v in recent]
            in_band = sum(1 for e in errors if e < SETTLING_BAND_PCT)
            
            if in_band >= SETTLING_CONSECUTIVE or n >= CAPTURE_MAX_SAMPLES:
                st.session_state.capturing = False
                st.session_state.step_response = {k: list(v) for k, v in buf.items()}
                backend.stop_stream()

# ── Live Metric Cards ────────────────────────────────────
buf = st.session_state.stream_buffer if st.session_state.running else st.session_state.capture_buffer
spd = buf["speed"] if buf["speed"] else [0.0]
cur = buf["current"] if buf["current"] else [0.0]
vcmd = buf["vcmd"] if buf["vcmd"] else [0.0]

def delta_html(vals):
    if len(vals) < 2: return ""
    d = vals[-1] - vals[-2]
    cls = "metric-delta-pos" if d >= 0 else "metric-delta-neg"
    return f'<span class="{cls}">{"▲" if d >= 0 else "▼"} {abs(d):.3f}</span>'

mc1, mc2, mc3 = st.columns(3)
with mc1:
    st.markdown(f"""<div class="metric-card speed">
        <div class="metric-label">⚙ SPEED</div>
        <div class="metric-value">{spd[-1]:.2f}</div>
        <div class="metric-unit">rad/s {delta_html(spd)}</div>
    </div>""", unsafe_allow_html=True)
with mc2:
    st.markdown(f"""<div class="metric-card current">
        <div class="metric-label">⚡ CURRENT</div>
        <div class="metric-value">{cur[-1]:.3f}</div>
        <div class="metric-unit">A {delta_html(cur)}</div>
    </div>""", unsafe_allow_html=True)
with mc3:
    st.markdown(f"""<div class="metric-card vcmd">
        <div class="metric-label">🔋 V_CMD</div>
        <div class="metric-value">{vcmd[-1]:.3f}</div>
        <div class="metric-unit">V {delta_html(vcmd)}</div>
    </div>""", unsafe_allow_html=True)

st.markdown("<br>", unsafe_allow_html=True)

# ── Performance Analysis Helper ──────────────────────────
def compute_performance(buf, setpoint):
    spd = buf["speed"]
    n = len(spd)
    if n < 10 or setpoint == 0:
        return None
    
    sp = abs(setpoint)
    # Peak in direction of setpoint
    if setpoint > 0:
        peak = max(spd)
        overshoot = max(0.0, (peak - sp) / sp * 100)
    else:
        peak = min(spd)
        overshoot = max(0.0, (abs(peak) - sp) / sp * 100)
    
    # Rise time: 10% to 90% of step change from initial value
    baseline = spd[0]
    step_mag = abs(setpoint - baseline)
    t10 = next((i for i, v in enumerate(spd) if abs(v - baseline) >= 0.1 * step_mag), None)
    t90 = next((i for i, v in enumerate(spd) if abs(v - baseline) >= 0.9 * step_mag), None)
    rise_samples = (t90 - t10) if (t10 is not None and t90 is not None) else None
    
    # Settling time: last exit from ±5% band
    band = SETTLING_BAND_PCT * sp
    settle_idx = n - 1
    for i in range(n - 1, -1, -1):
        if abs(abs(spd[i]) - sp) > band:
            settle_idx = i + 1
            break
    
    time_ms = buf.get("time_ms", list(range(n) * SAMPLE_PERIOD_MS))
    
    return {
        "overshoot": overshoot,
        "rise_samples": rise_samples,
        "rise_ms": rise_samples * SAMPLE_PERIOD_MS if rise_samples else None,
        "settle_ms": time_ms[settle_idx] if settle_idx < len(time_ms) else settle_idx * SAMPLE_PERIOD_MS,
        "peak": peak,
        "final": spd[-1],
        "peak_idx": spd.index(peak) if setpoint > 0 else spd.index(min(spd)),
    }

# ── Step Response Plot (after capture) ───────────────────
if st.session_state.step_response:
    sr = st.session_state.step_response
    sp = st.session_state.capture_setpoint
    perf = compute_performance(sr, sp)
    x_ms = sr.get("time_ms", [i * SAMPLE_PERIOD_MS for i in range(len(sr["speed"]))])
    
    st.markdown("---")
    st.markdown("### 📊 STEP RESPONSE ANALYSIS")
    
    if perf:
        p1, p2, p3, p4 = st.columns(4)
        p1.markdown(f"""<div class="perf-card">
            <div class="perf-label">OVERSHOOT</div>
            <div class="perf-value">{perf['overshoot']:.1f}</div>
            <div class="perf-unit">%</div>
        </div>""", unsafe_allow_html=True)
        p2.markdown(f"""<div class="perf-card">
            <div class="perf-label">RISE TIME</div>
            <div class="perf-value">{perf['rise_ms'] or '—'}</div>
            <div class="perf-unit">ms</div>
        </div>""", unsafe_allow_html=True)
        p3.markdown(f"""<div class="perf-card">
            <div class="perf-label">SETTLE TIME</div>
            <div class="perf-value">{perf['settle_ms']}</div>
            <div class="perf-unit">ms</div>
        </div>""", unsafe_allow_html=True)
        p4.markdown(f"""<div class="perf-card">
            <div class="perf-label">FINAL SPEED</div>
            <div class="perf-value">{perf['final']:.2f}</div>
            <div class="perf-unit">rad/s</div>
        </div>""", unsafe_allow_html=True)
    
    st.markdown("<br>", unsafe_allow_html=True)
    
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, row_heights=[0.5, 0.25, 0.25],
                        vertical_spacing=0.06, subplot_titles=["SPEED (rad/s)", "CURRENT (A)", "V_CMD (V)"])
    
    # Reference line + settling band
    fig.add_trace(go.Scatter(x=x_ms, y=[sp]*len(x_ms), mode="lines", name="Reference",
                            line=dict(color="#00FFD1", width=1, dash="dot"), opacity=0.6), row=1, col=1)
    if perf:
        band = SETTLING_BAND_PCT * abs(sp)
        fig.add_hrect(y0=sp-band, y1=sp+band, fillcolor="rgba(0,255,209,0.04)",
                     line=dict(color="#00FFD1", width=0.5, dash="dot"), row=1, col=1)
    
    # Speed trace
    fig.add_trace(go.Scatter(x=x_ms, y=sr["speed"], mode="lines", name="Speed",
                            line=dict(color="#00FFD1", width=2), fill="tozeroy", fillcolor="rgba(0,255,209,0.06)"), row=1, col=1)
    
    # Peak annotation
    if perf and perf["overshoot"] > 0.5:
        fig.add_annotation(x=x_ms[perf["peak_idx"]], y=perf["peak"],
                          text=f"  PEAK {perf['peak']:.2f}", showarrow=True, arrowcolor="#FFB800",
                          font=dict(color="#FFB800", size=10, family="Share Tech Mono"), row=1, col=1)
    
    # Settling annotation
    if perf:
        settle_idx = min(perf["settle_ms"] // SAMPLE_PERIOD_MS, len(x_ms)-1)
        fig.add_vline(x=x_ms[settle_idx], line=dict(color="#4A7A8A", width=1, dash="dot"), row=1, col=1)
        fig.add_annotation(x=x_ms[settle_idx], y=0, text=f"  SETTLED {perf['settle_ms']}ms",
                          showarrow=False, font=dict(color="#4A7A8A", size=9), xanchor="left", row=1, col=1)
    
    # Current & Vcmd
    fig.add_trace(go.Scatter(x=x_ms, y=sr["current"], mode="lines", name="Current",
                            line=dict(color="#FF4B6E", width=2), fill="tozeroy"), row=2, col=1)
    fig.add_trace(go.Scatter(x=x_ms, y=sr["vcmd"], mode="lines", name="V_cmd",
                            line=dict(color="#FFB800", width=2), fill="tozeroy"), row=3, col=1)
    
    fig.update_layout(height=600, paper_bgcolor="#080C10", plot_bgcolor="#080C10",
                     font=dict(family="Share Tech Mono", color="#4A7A8A", size=11),
                     showlegend=False, margin=dict(l=60, r=20, t=40, b=40))
    fig.update_xaxes(gridcolor="#131D24", title_text="Time (ms)", row=3, col=1)
    fig.update_yaxes(gridcolor="#131D24")
    
    st.plotly_chart(fig, use_container_width=True, config={"displayModeBar": False})
    
    # Export step response
    import pandas as pd
    df_sr = pd.DataFrame(sr)
    st.download_button("↓ Export STEP CSV", df_sr.to_csv(index=False), "step_response.csv", "text/csv")

# ── Live Streaming Plot ──────────────────────────────────
elif st.session_state.running and len(spd) > 1:
    x_ms = st.session_state.stream_buffer.get("time_ms", [i * SAMPLE_PERIOD_MS for i in range(len(spd))])
    
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, row_heights=[0.5, 0.25, 0.25],
                        vertical_spacing=0.06, subplot_titles=["SPEED (rad/s)", "CURRENT (A)", "V_CMD (V)"])
    
    fig.add_trace(go.Scatter(x=x_ms, y=buf["ref"], mode="lines", name="Reference",
                            line=dict(color="#00FFD1", width=1, dash="dot"), opacity=0.4), row=1, col=1)
    fig.add_trace(go.Scatter(x=x_ms, y=spd, mode="lines", name="Speed",
                            line=dict(color="#00FFD1", width=2), fill="tozeroy"), row=1, col=1)
    fig.add_trace(go.Scatter(x=x_ms, y=cur, mode="lines", name="Current",
                            line=dict(color="#FF4B6E", width=2), fill="tozeroy"), row=2, col=1)
    fig.add_trace(go.Scatter(x=x_ms, y=vcmd, mode="lines", name="V_cmd",
                            line=dict(color="#FFB800", width=2), fill="tozeroy"), row=3, col=1)
    
    fig.update_layout(height=560, paper_bgcolor="#080C10", plot_bgcolor="#080C10",
                     font=dict(family="Share Tech Mono", color="#4A7A8A", size=11),
                     showlegend=False, margin=dict(l=60, r=20, t=40, b=40))
    fig.update_xaxes(gridcolor="#131D24", title_text="Time (ms)", row=3, col=1)
    fig.update_yaxes(gridcolor="#131D24")
    
    st.plotly_chart(fig, use_container_width=True, config={"displayModeBar": False})

# ── Capture Status Message ───────────────────────────────
elif st.session_state.capturing:
    n = len(st.session_state.capture_buffer["speed"])
    sp = st.session_state.capture_setpoint
    st.markdown(f"""
    <div class="capture-status">
        ◉ RECORDING · {n} samples · {n * st.session_state.stream_period_ms} ms elapsed<br>
        <span style='color:#4A7A8A; font-size:0.7rem'>
        Waiting for speed to settle within ±{SETTLING_BAND_PCT*100:.0f}% of {sp:.1f} rad/s
        </span>
    </div>
    """, unsafe_allow_html=True)

# ── Auto-Rerun Loop ──────────────────────────────────────
if st.session_state.running or st.session_state.capturing:
    time.sleep(st.session_state.stream_period_ms / 1000)
    st.rerun()
