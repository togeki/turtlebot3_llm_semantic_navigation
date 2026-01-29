import math
import pandas as pd
import streamlit as st
import psycopg

DB_URL = "postgresql://tb3:tb3pass@localhost:5432/tb3db"

st.set_page_config(page_title="TB3 Dashboard", layout="wide")
st.title("TB3: ROS → PostgreSQL → Visualization")

# -------------------------
# DB loaders
# -------------------------
@st.cache_data(ttl=2)
def load_runs():
    with psycopg.connect(DB_URL) as conn:
        return pd.read_sql(
            """
            SELECT run_id, started_at, ended_at, target_place, result_status, target_pose,
                   duration_s, path_length_m, avg_speed_mps, stop_ratio
            FROM runs
            ORDER BY started_at DESC
            LIMIT 200
            """,
            conn
        )

@st.cache_data(ttl=2)
def load_events(run_id: str):
    with psycopg.connect(DB_URL) as conn:
        return pd.read_sql(
            """
            SELECT ts, type, payload
            FROM events
            WHERE run_id = %s
            ORDER BY ts ASC
            """,
            conn,
            params=(run_id,)
        )

@st.cache_data(ttl=2)
def load_topic(run_id: str, topic: str, limit: int = 20000):
    with psycopg.connect(DB_URL) as conn:
        df = pd.read_sql(
            """
            SELECT ts, payload
            FROM timeseries_messages
            WHERE run_id = %s AND topic = %s
            ORDER BY ts ASC
            LIMIT %s
            """,
            conn,
            params=(run_id, topic, limit)
        )
    return df

# -------------------------
# Helpers
# -------------------------
def safe_float(x):
    try:
        if pd.isna(x):
            return None
        return float(x)
    except Exception:
        return None

def path_length_xy(df_xy: pd.DataFrame) -> float:
    """Fallback: compute path length from xy points."""
    if df_xy.empty:
        return 0.0
    dist = 0.0
    x_prev = y_prev = None
    for x, y in zip(df_xy["x"], df_xy["y"]):
        if x_prev is not None:
            dist += math.hypot(x - x_prev, y - y_prev)
        x_prev, y_prev = x, y
    return float(dist)

# -------------------------
# Main
# -------------------------
runs = load_runs()

st.subheader("Runs")
if runs.empty:
    st.warning("No runs yet. Start Nav2 + collector, then publish /llm_place (or use llm_controller).")
    st.stop()

# Format a nicer view
runs_view = runs.copy()
for col in ["duration_s", "path_length_m", "avg_speed_mps", "stop_ratio"]:
    if col in runs_view.columns:
        runs_view[col] = pd.to_numeric(runs_view[col], errors="coerce")

# display-friendly formatting
runs_view["duration_s"] = runs_view["duration_s"].round(1)
runs_view["path_length_m"] = runs_view["path_length_m"].round(3)
runs_view["avg_speed_mps"] = runs_view["avg_speed_mps"].round(4)
runs_view["stop_ratio(%)"] = (runs_view["stop_ratio"] * 100).round(1)

show_cols = [
    "run_id", "started_at", "ended_at", "target_place", "result_status",
    "duration_s", "path_length_m", "avg_speed_mps", "stop_ratio(%)"
]
show_cols = [c for c in show_cols if c in runs_view.columns]
st.dataframe(runs_view[show_cols], use_container_width=True, height=320)

# -------------------------
# Leaderboards
# -------------------------
st.subheader("Top Runs (SUCCEEDED)")

succ = runs_view[runs_view["result_status"].fillna("") == "SUCCEEDED"].copy()
if succ.empty:
    st.info("No SUCCEEDED runs yet.")
else:
    # use original numeric columns for sorting
    succ_sort = runs.copy()
    for col in ["duration_s", "path_length_m", "avg_speed_mps", "stop_ratio"]:
        if col in succ_sort.columns:
            succ_sort[col] = pd.to_numeric(succ_sort[col], errors="coerce")
    succ_sort = succ_sort[succ_sort["result_status"].fillna("") == "SUCCEEDED"].copy()

    a, b, c = st.columns(3)

    with a:
        st.caption("Shortest duration (s)")
        df = succ_sort.sort_values("duration_s", na_position="last").head(10)
        df_show = df[["run_id", "target_place", "duration_s", "path_length_m", "stop_ratio"]].copy()
        df_show["stop_ratio"] = (df_show["stop_ratio"] * 100).round(1)
        st.dataframe(df_show, use_container_width=True, height=280)

    with b:
        st.caption("Shortest path (m)")
        df = succ_sort.sort_values("path_length_m", na_position="last").head(10)
        df_show = df[["run_id", "target_place", "path_length_m", "duration_s", "stop_ratio"]].copy()
        df_show["stop_ratio"] = (df_show["stop_ratio"] * 100).round(1)
        st.dataframe(df_show, use_container_width=True, height=280)

    with c:
        st.caption("Lowest stop_ratio (%)")
        df = succ_sort.sort_values("stop_ratio", na_position="last").head(10)
        df_show = df[["run_id", "target_place", "stop_ratio", "duration_s", "path_length_m"]].copy()
        df_show["stop_ratio"] = (df_show["stop_ratio"] * 100).round(1)
        st.dataframe(df_show, use_container_width=True, height=280)

st.divider()

# -------------------------
# Run details
# -------------------------
run_id = st.selectbox("Select run_id", runs["run_id"].astype(str).tolist())

row = runs[runs["run_id"].astype(str) == run_id].iloc[0]

# KPI cards from DB
path_m = safe_float(row.get("path_length_m"))
dur_s = safe_float(row.get("duration_s"))
avg_speed = safe_float(row.get("avg_speed_mps"))
stop_ratio = safe_float(row.get("stop_ratio"))

k1, k2, k3, k4 = st.columns(4)
k1.metric("path_length (m)", "-" if path_m is None else f"{path_m:.3f}")
k2.metric("duration (s)", "-" if dur_s is None else f"{dur_s:.1f}")
k3.metric("avg_speed (m/s)", "-" if avg_speed is None else f"{avg_speed:.4f}")
k4.metric("stop_ratio", "-" if stop_ratio is None else f"{stop_ratio:.1%}")

meta_cols = st.columns(4)
with meta_cols[0]:
    st.caption("target_place")
    st.write(row.get("target_place"))
with meta_cols[1]:
    st.caption("result_status")
    st.write(row.get("result_status"))
with meta_cols[2]:
    st.caption("started_at")
    st.write(row.get("started_at"))
with meta_cols[3]:
    st.caption("ended_at")
    st.write(row.get("ended_at"))

c1, c2 = st.columns([1, 1])

with c1:
    st.subheader("Events")
    ev = load_events(run_id)
    st.dataframe(ev, use_container_width=True, height=420)

with c2:
    st.subheader("Trajectory (x-y) from /odom")
    od = load_topic(run_id, "/odom")
    if od.empty:
        st.info("No /odom data for this run.")
    else:
        # payload is stored as JSON string; psycopg/pandas may already decode to dict depending on schema
        def _get_xy(payload):
            try:
                if isinstance(payload, dict):
                    return payload.get("x"), payload.get("y")
                # if payload is string, try parse
                import json
                p = json.loads(payload)
                return p.get("x"), p.get("y")
            except Exception:
                return None, None

        xs, ys = [], []
        for p in od["payload"]:
            x, y = _get_xy(p)
            xs.append(x)
            ys.append(y)

        xy = pd.DataFrame({"x": xs, "y": ys}).dropna()
        if xy.empty:
            st.info("No valid x-y points in /odom payload.")
        else:
            st.scatter_chart(xy, x="x", y="y")
            # fallback path length (for sanity check)
            st.caption(f"points: {len(xy)} | path_length(fallback) ≈ {path_length_xy(xy):.3f} m")

st.divider()

st.subheader("/cmd_vel (vx_cmd, wz_cmd)")
cv = load_topic(run_id, "/cmd_vel", limit=5000)
if cv.empty:
    st.info("No /cmd_vel data for this run.")
else:
    def _get_v(payload):
        try:
            if isinstance(payload, dict):
                return payload.get("vx_cmd"), payload.get("wz_cmd")
            import json
            p = json.loads(payload)
            return p.get("vx_cmd"), p.get("wz_cmd")
        except Exception:
            return None, None

    vx_list, wz_list = [], []
    for p in cv["payload"]:
        vx, wz = _get_v(p)
        vx_list.append(vx)
        wz_list.append(wz)

    out = pd.DataFrame({"ts": cv["ts"], "vx_cmd": vx_list, "wz_cmd": wz_list}).dropna().set_index("ts")
    st.line_chart(out)

