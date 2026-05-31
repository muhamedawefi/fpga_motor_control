import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import streamlit as st
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import time
from backends.zmq_backend import ZMQBackend

SAMPLE_PERIOD_MS = 80  # ms per sample — matches time.sleep(0.08) rerun rate

st.set_page_config(
    page_title="FPGA Motor Control",
    page_icon="⚡",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.markdown("""
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');

html, body, [class*="css"] {
    font-family: 'Rajdhani', sans-serif;
    background-color: #080C10;
}
.stApp { background-color: #080C10; }

.title-block {
    border-left: 4px solid #00FFD1;
    padding: 8px 20px;
    margin-bottom: 24px;
    background: linear-gradient(90deg, rgba(0,255,209,0.05) 0%, transparent 100%);
}
.title-block h1 {
    font-family: 'Rajdhani', sans-serif;
    font-weight: 700;
    font-size: 2.2rem;
    color: #E8F4F8;
    margin: 0;
    letter-spacing: 2px;
}
.title-block p {
    color: #4A7A8A;
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.78rem;
    margin: 4px 0 0 0;
    letter-spacing: 1px;
}

.badge {
    display: inline-block; padding: 2px 10px; border-radius: 2px;
    font-family: 'Share Tech Mono', monospace; font-size: 0.7rem; letter-spacing: 1px;
}
.badge-live    { background: rgba(0,255,100,0.15); color: #00FF64; border: 1px solid #00FF6440; }
.badge-idle    { background: rgba(255,100,0,0.15);  color: #FF6400; border: 1px solid #FF640040; }
.badge-capture { background: rgba(255,184,0,0.15);  color: #FFB800; border: 1px solid #FFB80040; }

.metric-card {
    background: #0D1318; border: 1px solid #1A2830;
    border-top: 2px solid; padding: 16px 20px;
    border-radius: 4px; font-family: 'Share Tech Mono', monospace;
    margin-bottom: 8px;
}
.metric-card.speed   { border-top-color: #00FFD1; }
.metric-card.current { border-top-color: #FF4B6E; }
.metric-card.vcmd    { border-top-color: #FFB800; }
.metric-label { font-size: 0.68rem; color: #4A7A8A; letter-spacing: 2px; text-transform: uppercase; }
.metric-value { font-size: 2rem; font-weight: 700; color: #E8F4F8; margin: 4px 0; font-family: 'Rajdhani'; }
.metric-unit  { font-size: 0.7rem; color: #4A7A8A; }
.metric-delta-pos { color: #00FF64; font-size: 0.75rem; }
.metric-delta-neg { color: #FF4B6E; font-size: 0.75rem; }

.perf-card {
    background: #0D1318; border: 1px solid #1A2830;
    padding: 14px 16px; border-radius: 4px;
    font-family: 'Share Tech Mono', monospace;
    text-align: center; margin-bottom: 8px;
}
.perf-label { font-size: 0.65rem; color: #4A7A8A; letter-spacing: 2px; text-transform: uppercase; }
.perf-value { font-size: 1.5rem; font-weight: 700; color: #00FFD1; font-family: 'Rajdhani'; }
.perf-unit  { font-size: 0.65rem; color: #4A7A8A; }

.capture-status {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.8rem; color: #FFB800;
    text-align: center; padding: 12px;
    background: rgba(255,184,0,0.05);
    border: 1px solid rgba(255,184,0,0.2);
    border-radius: 4px; margin: 12px 0;
}

section[data-testid="stSidebar"] {
    background-color: #0A0F14;
    border-right: 1px solid #1A2830;
}
section[data-testid="stSidebar"] * { color: #C8D8E0 !important; }

.stButton > button {
    background: transparent; border: 1px solid #1A3040;
    color: #00FFD1 !important; font-family: 'Share Tech Mono', monospace;
    font-size: 0.75rem; letter-spacing: 1px; border-radius: 2px;
    transition: all 0.2s; width: 100%; margin-bottom: 4px;
}
.stButton > button:hover { background: rgba(0,255,209,0.08); border-color: #00FFD1; }
hr { border-color: #1A2830 !important; }
</style>
""", unsafe_allow_html=True)

# ── Session state ─────────────────────────────────────────
defaults = {
    "backend":          None,
    "history":          {"speed": [], "current": [], "vcmd": []},
    "last_setpoint":    0.0,
    "streaming":        False,
    "capturing":        False,
    "capture_buffer":   {"speed": [], "current": [], "vcmd": []},
    "capture_setpoint": 0.0,
    "step_response":    None,
}
for k, v in defaults.items():
    if k not in st.session_state:
        st.session_state[k] = v

if st.session_state.backend is None:
    st.session_state.backend = ZMQBackend()

backend = st.session_state.backend

# ── Performance analysis ──────────────────────────────────
def compute_performance(buf, setpoint):
    spd = buf["speed"]
    n   = len(spd)
    if n < 5 or setpoint == 0:
        return None

    sp        = abs(setpoint)
    peak      = max(spd) if setpoint > 0 else min(spd)
    overshoot = max(0.0, (abs(peak) - sp) / sp * 100)

    t10 = next((i for i, v in enumerate(spd) if abs(v) >= 0.1 * sp), None)
    t90 = next((i for i, v in enumerate(spd) if abs(v) >= 0.9 * sp), None)
    rise_samples = (t90 - t10) if (t10 is not None and t90 is not None) else None

    band   = 0.05 * sp
    settle = n
    for i in range(n - 1, -1, -1):
        if abs(abs(spd[i]) - sp) > band:
            settle = i + 1
            break

    return {
        "overshoot":    overshoot,
        "rise_samples": rise_samples,
        "rise_ms":      (rise_samples * SAMPLE_PERIOD_MS) if rise_samples else None,
        "settle_ms":    settle * SAMPLE_PERIOD_MS,
        "peak":         peak,
        "final":        spd[-1],
        "peak_idx":     spd.index(peak) if setpoint > 0 else spd.index(min(spd)),
    }

# ── Sidebar ───────────────────────────────────────────────
with st.sidebar:
    st.markdown("### CONTROL PANEL")
    st.markdown("---")

    setpoint = st.slider(
        "Speed Reference (rad/s)",
        min_value=-100.0, max_value=100.0,
        value=st.session_state.last_setpoint,
        step=0.5
    )

    if setpoint != st.session_state.last_setpoint:
        backend.set_speed(setpoint)
        st.session_state.last_setpoint = setpoint

    st.markdown("")

    if st.button("▶  START LIVE STREAM"):
        st.session_state.streaming     = True
        st.session_state.capturing     = False
        st.session_state.step_response = None
        backend.start_stream()

    if st.button("📈  CAPTURE STEP RESPONSE"):
        st.session_state.capturing      = True
        st.session_state.streaming      = False
        st.session_state.step_response  = None
        st.session_state.capture_buffer = {"speed": [], "current": [], "vcmd": []}
        st.session_state.capture_setpoint = setpoint
        backend.set_speed(setpoint)

    if st.button("■  STOP"):
        st.session_state.streaming = False
        st.session_state.capturing = False
        backend.set_speed(0)
        st.session_state.last_setpoint = 0.0

    if st.button("↺  CLEAR ALL"):
        st.session_state.history        = {"speed": [], "current": [], "vcmd": []}
        st.session_state.step_response  = None
        st.session_state.capture_buffer = {"speed": [], "current": [], "vcmd": []}

    st.markdown("---")

    if st.session_state.history["speed"]:
        import pandas as pd
        df_exp = pd.DataFrame(st.session_state.history)
        st.download_button(
            "↓  EXPORT LIVE CSV",
            df_exp.to_csv(index=False),
            "telemetry.csv", mime="text/csv"
        )

    st.markdown("---")
    st.markdown("""
    <div style='font-family: Share Tech Mono; font-size: 0.65rem; color: #2A4A5A; line-height: 1.8'>
    PLATFORM · FPGA Artix-7<br>
    CONTROL  · Cascade PI<br>
    COMM     · UART 115200<br>
    BRIDGE   · ZMQ TCP<br>
    SIM      · GHDL + CocoTB<br>
    </div>
    """, unsafe_allow_html=True)

# ── Header ────────────────────────────────────────────────
if st.session_state.capturing:
    n_captured = len(st.session_state.capture_buffer["speed"])
    badge = f'<span class="badge badge-capture">◉ CAPTURING  {n_captured} samples</span>'
elif st.session_state.streaming:
    badge = '<span class="badge badge-live">● LIVE</span>'
else:
    badge = '<span class="badge badge-idle">○ IDLE</span>'

st.markdown(f"""
<div class="title-block">
    <h1>FPGA MOTOR CONTROL {badge}</h1>
    <p>REAL-TIME DC MOTOR SUPERVISION · VHDL CASCADE PI · ARTIX-7 · COCOTB · ZMQ</p>
</div>
""", unsafe_allow_html=True)

# ── Data ingestion ────────────────────────────────────────
data = None
if st.session_state.streaming or st.session_state.capturing:
    data = backend.get_data()

if data:
    spd_val  = data["speed_raw"]   / 128
    cur_val  = data["current_raw"] / 4096
    vcmd_val = data["vcmd_raw"]    / 1024

    if st.session_state.streaming:
        h = st.session_state.history
        h["speed"].append(spd_val)
        h["current"].append(cur_val)
        h["vcmd"].append(vcmd_val)
        for k in h:
            if len(h[k]) > 400:
                h[k] = h[k][-400:]

    if st.session_state.capturing:
        buf = st.session_state.capture_buffer
        buf["speed"].append(spd_val)
        buf["current"].append(cur_val)
        buf["vcmd"].append(vcmd_val)

        sp = st.session_state.capture_setpoint
        n  = len(buf["speed"])
        if n > 50 and sp != 0:
            recent  = buf["speed"][-20:]
            settled = all(abs(v - sp) / abs(sp) < 0.05 for v in recent)
            if settled or n > 600:
                st.session_state.capturing     = False
                st.session_state.step_response = {k: list(v) for k, v in buf.items()}

# ── Live metric cards ─────────────────────────────────────
h    = st.session_state.history
spd  = h["speed"]   if h["speed"]   else [0.0]
cur  = h["current"] if h["current"] else [0.0]
vcmd = h["vcmd"]    if h["vcmd"]    else [0.0]

def delta_html(vals):
    if len(vals) < 2:
        return ""
    d   = vals[-1] - vals[-2]
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

BG   = "#080C10"
GRID = "#131D24"

# ── Step response plot (static, appears after capture) ────
if st.session_state.step_response:
    sr   = st.session_state.step_response
    sp   = st.session_state.capture_setpoint
    perf = compute_performance(sr, sp)
    x_ms = [i * SAMPLE_PERIOD_MS for i in range(len(sr["speed"]))]

    st.markdown("---")
    st.markdown("### STEP RESPONSE ANALYSIS")

    if perf:
        p1, p2, p3, p4 = st.columns(4)
        p1.markdown(f"""<div class="perf-card">
            <div class="perf-label">OVERSHOOT</div>
            <div class="perf-value">{perf['overshoot']:.1f}</div>
            <div class="perf-unit">%</div>
        </div>""", unsafe_allow_html=True)
        p2.markdown(f"""<div class="perf-card">
            <div class="perf-label">RISE TIME</div>
            <div class="perf-value">{perf['rise_ms'] if perf['rise_ms'] else '—'}</div>
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

    fig2 = make_subplots(
        rows=3, cols=1,
        shared_xaxes=True,
        row_heights=[0.5, 0.25, 0.25],
        vertical_spacing=0.06,
        subplot_titles=["SPEED  (rad/s)", "CURRENT  (A)", "V_CMD  (V)"]
    )

    # reference line
    fig2.add_trace(go.Scatter(
        x=x_ms, y=[sp] * len(x_ms),
        mode="lines", name="Reference",
        line=dict(color="#00FFD1", width=1, dash="dot"),
        opacity=0.5
    ), row=1, col=1)

    # ±5% settling band
    if perf:
        band = 0.05 * abs(sp)
        fig2.add_hrect(
            y0=sp - band, y1=sp + band,
            fillcolor="rgba(0,255,209,0.04)",
            line=dict(color="#00FFD1", width=0.5, dash="dot"),
            row=1, col=1
        )

    # speed trace
    fig2.add_trace(go.Scatter(
        x=x_ms, y=sr["speed"],
        mode="lines", name="Speed",
        line=dict(color="#00FFD1", width=2),
        fill="tozeroy", fillcolor="rgba(0,255,209,0.06)"
    ), row=1, col=1)

    # peak annotation
    if perf and perf["overshoot"] > 0.5:
        fig2.add_annotation(
            x=x_ms[perf["peak_idx"]], y=perf["peak"],
            text=f"  PEAK {perf['peak']:.2f} rad/s",
            showarrow=True, arrowhead=2, arrowsize=1,
            arrowcolor="#FFB800",
            font=dict(color="#FFB800", size=10, family="Share Tech Mono"),
            row=1, col=1
        )

    # settle time annotation
    if perf:
        settle_idx = min(perf["settle_ms"] // SAMPLE_PERIOD_MS, len(x_ms) - 1)
        fig2.add_vline(
            x=x_ms[settle_idx],
            line=dict(color="#4A7A8A", width=1, dash="dot"),
            row=1, col=1
        )
        fig2.add_annotation(
            x=x_ms[settle_idx], y=0,
            text=f"  SETTLED {perf['settle_ms']}ms",
            showarrow=False,
            font=dict(color="#4A7A8A", size=9, family="Share Tech Mono"),
            xanchor="left",
            row=1, col=1
        )

    # current
    fig2.add_trace(go.Scatter(
        x=x_ms, y=sr["current"],
        mode="lines", name="Current",
        line=dict(color="#FF4B6E", width=2),
        fill="tozeroy", fillcolor="rgba(255,75,110,0.06)"
    ), row=2, col=1)

    # vcmd
    fig2.add_trace(go.Scatter(
        x=x_ms, y=sr["vcmd"],
        mode="lines", name="V_cmd",
        line=dict(color="#FFB800", width=2),
        fill="tozeroy", fillcolor="rgba(255,184,0,0.06)"
    ), row=3, col=1)

    fig2.update_layout(
        height=600,
        paper_bgcolor=BG,
        plot_bgcolor=BG,
        font=dict(family="Share Tech Mono", color="#4A7A8A", size=11),
        showlegend=False,
        margin=dict(l=60, r=20, t=40, b=40),
        uirevision="step_response"
    )
    fig2.update_xaxes(
        gridcolor=GRID, zeroline=False,
        showticklabels=True,
        ticksuffix=" ms",
        title_text="Time (ms)",
        title_font=dict(color="#4A7A8A", size=10)
    )
    fig2.update_xaxes(showticklabels=True, row=3, col=1)
    fig2.update_yaxes(gridcolor=GRID, zeroline=False)

    st.plotly_chart(fig2, use_container_width=True, config={"displayModeBar": False})

    import pandas as pd
    df_sr = pd.DataFrame({"time_ms": x_ms, **sr})
    st.download_button(
        "↓  EXPORT STEP RESPONSE CSV",
        df_sr.to_csv(index=False),
        "step_response.csv", mime="text/csv"
    )

# ── Live chart (streaming mode only) ─────────────────────
elif st.session_state.streaming and len(spd) > 1:
    x_ms = [i * SAMPLE_PERIOD_MS for i in range(len(spd))]

    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True,
        row_heights=[0.5, 0.25, 0.25],
        vertical_spacing=0.06,
        subplot_titles=["SPEED  (rad/s)", "CURRENT  (A)", "V_CMD  (V)"]
    )

    fig.add_trace(go.Scatter(
        x=x_ms, y=[setpoint] * len(x_ms),
        mode="lines",
        line=dict(color="#00FFD1", width=1, dash="dot"),
        opacity=0.4
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=x_ms, y=spd, mode="lines",
        line=dict(color="#00FFD1", width=2),
        fill="tozeroy", fillcolor="rgba(0,255,209,0.04)"
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=x_ms, y=cur, mode="lines",
        line=dict(color="#FF4B6E", width=2),
        fill="tozeroy", fillcolor="rgba(255,75,110,0.04)"
    ), row=2, col=1)
    fig.add_trace(go.Scatter(
        x=x_ms, y=vcmd, mode="lines",
        line=dict(color="#FFB800", width=2),
        fill="tozeroy", fillcolor="rgba(255,184,0,0.04)"
    ), row=3, col=1)

    fig.update_layout(
        height=560,
        paper_bgcolor=BG, plot_bgcolor=BG,
        font=dict(family="Share Tech Mono", color="#4A7A8A", size=11),
        showlegend=False,
        margin=dict(l=60, r=20, t=40, b=40),
        uirevision="constant"
    )
    fig.update_xaxes(
        gridcolor=GRID, zeroline=False,
        showticklabels=True,
        ticksuffix=" ms",
        title_text="Time (ms)",
        title_font=dict(color="#4A7A8A", size=10)
    )
    fig.update_xaxes(showticklabels=True, row=3, col=1)
    fig.update_yaxes(gridcolor=GRID, zeroline=False)

    st.plotly_chart(fig, use_container_width=True, config={"displayModeBar": False})

elif st.session_state.capturing:
    n = len(st.session_state.capture_buffer["speed"])
    st.markdown(f"""
    <div class="capture-status">
        ◉ RECORDING · {n} samples · {n * SAMPLE_PERIOD_MS} ms elapsed<br>
        <span style='color:#4A7A8A; font-size:0.7rem'>
        waiting for speed to settle within 5% of {st.session_state.capture_setpoint:.1f} rad/s
        </span>
    </div>
    """, unsafe_allow_html=True)

# ── Rerun loop ────────────────────────────────────────────
if st.session_state.streaming or st.session_state.capturing:
    time.sleep(0.08)
    st.rerun()
