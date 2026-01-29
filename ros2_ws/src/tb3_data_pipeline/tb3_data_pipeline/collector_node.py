import math
import json
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import psycopg


def now_utc():
    return datetime.now(timezone.utc)
    
def path_length_from_odom(conn, run_id: str) -> float:
    """Compute path length (m) from /odom x,y in DB."""
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


def stop_ratio_from_cmd_vel(conn, run_id: str, vx_thresh: float = 0.01) -> float:
    """Compute stop ratio from /cmd_vel vx_cmd."""
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

    return float(stopped / total) if total > 0 else 0.0


def compute_and_update_kpis(conn, run_id: str):
    """Compute KPIs for run_id and UPDATE runs table."""
    with conn.cursor() as cur:
        cur.execute("SELECT started_at, ended_at FROM runs WHERE run_id=%s", (run_id,))
        row = cur.fetchone()

    if not row:
        return None

    started_at, ended_at = row
    if started_at is None or ended_at is None:
        return None

    duration_s = float((ended_at - started_at).total_seconds())
    path_m = path_length_from_odom(conn, run_id)
    stop_ratio = stop_ratio_from_cmd_vel(conn, run_id, vx_thresh=0.01)
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

    return {"duration_s": duration_s, "path_length_m": path_m, "avg_speed_mps": avg_speed, "stop_ratio": stop_ratio}



class Tb3Collector(Node):
    def __init__(self):
        super().__init__('tb3_collector')

        self.declare_parameter('db_url', 'postgresql://tb3:tb3pass@localhost:5432/tb3db')
        self.db_url = self.get_parameter('db_url').get_parameter_value().string_value

        # run 状态
        self.run_id = None
        self.run_started_at = None
        self.target_place = None
        self.target_pose = None  # dict

        self.conn = psycopg.connect(self.db_url, autocommit=True)
        
        # throttling
        self.odom_last_ts = None
        self.cmd_last_ts = None
        self.odom_hz = 10.0
        self.cmd_hz = 10.0


        # subscribers
        self.create_subscription(String, '/llm_place', self.cb_llm_place, 10)
        self.create_subscription(String, '/llm_nav_json', self.cb_llm_nav_json, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 50)
        self.create_subscription(String, '/tb3_nav_result', self.cb_nav_result, 10)
        
        
        self.declare_parameter('run_timeout_s', 600.0)
        self.run_timeout_s = float(self.get_parameter('run_timeout_s').value)

        self.timeout_timer = self.create_timer(1.0, self.cb_timeout_check)


        self.get_logger().info('tb3_collector started. Waiting for topics...')

    def ensure_run(self):
        if self.run_id is not None:
            return

        self.run_id = uuid.uuid4()
        self.run_started_at = now_utc()

        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO runs(run_id, started_at, ros_distro)
                VALUES (%s, %s, %s)
                """,
                (str(self.run_id), self.run_started_at, 'humble')
            )

        self.get_logger().info(f'RUN START: {self.run_id}')

    def insert_event(self, event_type: str, payload: dict):
        if self.run_id is None:
            self.ensure_run()
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO events(run_id, ts, type, payload)
                VALUES (%s, %s, %s, %s)
                """,
                (str(self.run_id), now_utc(), event_type, json.dumps(payload))
            )

    def insert_ts(self, topic: str, frame_id: str, payload: dict, ts=None):
        if self.run_id is None:
            return
        if ts is None:
            ts = now_utc()
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO timeseries_messages(run_id, ts, topic, frame_id, payload)
                VALUES (%s, %s, %s, %s, %s)
                """,
                (str(self.run_id), ts, topic, frame_id, json.dumps(payload))
            )


    def cb_llm_place(self, msg: String):
        self.ensure_run()
        self.target_place = msg.data
        self.insert_event('LLM_PLACE', {'place': msg.data})

        # 更新 runs 表
        with self.conn.cursor() as cur:
            cur.execute(
                "UPDATE runs SET target_place=%s WHERE run_id=%s",
                (self.target_place, str(self.run_id))
            )

    def cb_llm_nav_json(self, msg: String):
        self.ensure_run()
        # 你的 /llm_nav_json 很可能是 JSON 字符串：{"x":..,"y":..,"yaw":..} 或类似
        try:
            pose = json.loads(msg.data)
        except Exception:
            pose = {'raw': msg.data}

        self.target_pose = pose
        self.insert_event('NAV_GOAL_SENT', {'goal': pose})

        with self.conn.cursor() as cur:
            cur.execute(
                "UPDATE runs SET target_pose=%s WHERE run_id=%s",
                (json.dumps(self.target_pose), str(self.run_id))
            )

    def cb_odom(self, msg: Odometry):
        if self.run_id is None:
            return
        
        now = now_utc()
        if self.odom_last_ts is not None:
            if (now - self.odom_last_ts).total_seconds() < (1.0 / self.odom_hz):
                return
        self.odom_last_ts = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist

        payload = {
            'x': p.x,
            'y': p.y,
            'z': p.z,
            'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w,
            'vx': t.linear.x,
            'vy': t.linear.y,
            'wz': t.angular.z
        }
        self.insert_ts('/odom', msg.header.frame_id, payload, ts=now)


    def cb_cmd_vel(self, msg: Twist):
        if self.run_id is None:
            return

        now = now_utc()
        if self.cmd_last_ts is not None:
            if (now - self.cmd_last_ts).total_seconds() < (1.0 / self.cmd_hz):
                return
        self.cmd_last_ts = now

        payload = {
            'vx_cmd': msg.linear.x,
            'vy_cmd': msg.linear.y,
            'wz_cmd': msg.angular.z
        }
        self.insert_ts('/cmd_vel', '', payload, ts=now)

    def cb_nav_result(self, msg: String):
        if self.run_id is None:
            return

        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}

        code = int(data.get("status_code", 0))
        code_map = {
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED",
            0: "UNKNOWN",
            1: "ACCEPTED",
            2: "EXECUTING",
            3: "CANCELING",
        }
        status_text = code_map.get(code, f"CODE_{code}")

        self.insert_event("NAV_RESULT", {"status_code": code, "status": status_text})

        ended = now_utc()
        with self.conn.cursor() as cur:
            cur.execute(
                """
                UPDATE runs
                SET ended_at=%s, result_status=%s, result_detail=%s
                WHERE run_id=%s
                """,
                (ended, status_text, json.dumps(data), str(self.run_id))
            )
        
        # KPI compute + write back to runs
        try:
            kpis = compute_and_update_kpis(self.conn, str(self.run_id))
            if kpis:
                self.get_logger().info(f"KPI UPDATED: {kpis}")
        except Exception as e:
            self.get_logger().warn(f"Failed to compute/update KPIs: {e}")


        self.get_logger().info(f"RUN END: {self.run_id} status={status_text}")

        # reset for next run
        self.run_id = None
        self.run_started_at = None
        self.target_place = None
        self.target_pose = None
        
        self.odom_last_ts = None
        self.cmd_last_ts = None

    def cb_timeout_check(self):
        if self.run_id is None or self.run_started_at is None:
            return

        now = now_utc()
        elapsed = (now - self.run_started_at).total_seconds()
        if elapsed < self.run_timeout_s:
            return

        # Timeout! close the run
        self.insert_event("RUN_TIMEOUT", {"elapsed_s": elapsed, "timeout_s": self.run_timeout_s})

        with self.conn.cursor() as cur:
            cur.execute(
                """
                UPDATE runs
                SET ended_at=%s,
                    result_status=%s,
                    result_detail=%s
                WHERE run_id=%s AND ended_at IS NULL
                """,
                (now, "TIMEOUT", json.dumps({"elapsed_s": elapsed}), str(self.run_id))
            )

        # KPI compute + write back (optional, but good)
        try:
            kpis = compute_and_update_kpis(self.conn, str(self.run_id))
            if kpis:
                self.get_logger().info(f"KPI UPDATED (timeout): {kpis}")
        except Exception as e:
            self.get_logger().warn(f"Failed to compute/update KPIs (timeout): {e}")

        self.get_logger().warn(f"RUN TIMEOUT: {self.run_id} elapsed={elapsed:.1f}s")

        # reset
        self.run_id = None
        self.run_started_at = None
        self.target_place = None
        self.target_pose = None
        self.odom_last_ts = None
        self.cmd_last_ts = None



def main():
    rclpy.init()
    node = Tb3Collector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down collector...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
