#!/usr/bin/env python3
import os
import psycopg
import json
import math

DB_URL = os.environ.get("DB_URL", "postgresql://tb3:tb3pass@localhost:5432/tb3db")

def path_length_from_odom(conn, run_id: str) -> float:
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT payload
            FROM timeseries_messages
            WHERE run_id=%s AND topic=%s
            ORDER BY ts ASC
            """,
            (run_id, "/odom")
        )
        rows = cur.fetchall()

    dist = 0.0
    x_prev = y_prev = None
    for (payload_str,) in rows:
        try:
            p = json.loads(payload_str) if isinstance(payload_str, str) else payload_str
        except Exception:
            p = {}
        if not isinstance(p, dict):
            continue
        x = p.get("x"); y = p.get("y")
        if x is None or y is None:
            continue
        try:
            x = float(x); y = float(y)
        except Exception:
            continue
        if x_prev is not None:
            dist += math.hypot(x - x_prev, y - y_prev)
        x_prev, y_prev = x, y
    return float(dist)

def stop_ratio_from_cmd(conn, run_id: str, vx_thresh: float = 0.01) -> float:
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT payload
            FROM timeseries_messages
            WHERE run_id=%s AND topic=%s
            ORDER BY ts ASC
            """,
            (run_id, "/cmd_vel")
        )
        rows = cur.fetchall()

    total = 0
    stopped = 0
    for (payload_str,) in rows:
        try:
            p = json.loads(payload_str) if isinstance(payload_str, str) else payload_str
        except Exception:
            p = {}
        if not isinstance(p, dict):
            continue
        vx = p.get("vx_cmd")
        if vx is None:
            continue
        try:
            vx = float(vx)
        except Exception:
            continue
        total += 1
        if abs(vx) < vx_thresh:
            stopped += 1
    return float(stopped / total) if total else 0.0

def main():
    with psycopg.connect(DB_URL, autocommit=True) as conn:
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT run_id, started_at, ended_at
                FROM runs
                WHERE ended_at IS NOT NULL
                  AND (duration_s IS NULL OR path_length_m IS NULL OR stop_ratio IS NULL)
                ORDER BY started_at ASC
                """
            )
            targets = cur.fetchall()

        print(f"Need backfill: {len(targets)} runs")
        for run_id, started_at, ended_at in targets:
            duration_s = (ended_at - started_at).total_seconds()
            path_m = path_length_from_odom(conn, run_id)
            stop_ratio = stop_ratio_from_cmd(conn, run_id, 0.01)
            avg_speed = (path_m / duration_s) if duration_s > 0 else None

            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE runs
                    SET duration_s=%s,
                        path_length_m=%s,
                        avg_speed_mps=%s,
                        stop_ratio=%s
                    WHERE run_id=%s
                    """,
                    (duration_s, path_m, avg_speed, stop_ratio, run_id)
                )
            print(f"OK {run_id} duration={duration_s:.2f}s path={path_m:.3f}m stop={stop_ratio:.2%}")

if __name__ == "__main__":
    main()
