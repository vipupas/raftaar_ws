#!/usr/bin/env python3
"""
obstacle_mode_switcher_dual.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Dual front-facing LiDAR obstacle-aware MAVROS mode switcher with MANUAL
mode lock.  Both LiDARs face forward at different heights to catch
obstacles the other might miss (ground-level vs. upper-body).

Each LiDAR has its own threshold, detection sector, and topic.

LiDAR layout
------------
  /scan_top    – front-facing, mounted ~1.15m above ground
  /scan_low    – front-facing, mounted ~0.58m above ground

Logic
------
  obstacle = top_triggered OR low_triggered   (any-OR policy)
  If current ArduPilot mode is MANUAL → block all automatic switching.
  obstacle present (debounced)  → mode_when_obstacle  (default: GUIDED)
  obstacle absent  (debounced)  → mode_when_clear     (default: AUTO)
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode


def nearest_in_sector(msg: LaserScan, center_deg: float, half_deg: float, min_valid_m: float) -> float:
    ranges = msg.ranges
    if not ranges:
        return float("inf")

    a_min = msg.angle_min
    a_inc = msg.angle_increment
    a_max = msg.angle_max
    n = len(ranges)

    center_rad = math.radians(center_deg)
    half_rad   = math.radians(half_deg)

    lo_rad = center_rad - half_rad
    hi_rad = center_rad + half_rad

    def angle_to_index(a_rad: float) -> int:
        idx = int(round((a_rad - a_min) / a_inc))
        return max(0, min(n - 1, idx))

    segments = []
    if lo_rad < a_min:
        wrap_lo = lo_rad + 2 * math.pi
        segments.append((angle_to_index(wrap_lo), n - 1))
        segments.append((0, angle_to_index(hi_rad)))
    elif hi_rad > a_max:
        wrap_hi = hi_rad - 2 * math.pi
        segments.append((angle_to_index(lo_rad), n - 1))
        segments.append((0, angle_to_index(wrap_hi)))
    else:
        segments.append((angle_to_index(lo_rad), angle_to_index(hi_rad)))

    nearest = float("inf")
    for s0, s1 in segments:
        if s0 > s1:
            s0, s1 = s1, s0
        for i in range(s0, s1 + 1):
            r = ranges[i]
            if r <= min_valid_m or math.isinf(r) or math.isnan(r):
                continue
            if r < nearest:
                nearest = r
    return nearest


class DualObstacleModeSwitcher(Node):
    """Two front-facing LiDAR obstacle mode switcher with MANUAL-lock.

    Each LiDAR has its own threshold, center angle, and half angle."""

    SENSOR_IDS = ("top", "low")

    def __init__(self):
        super().__init__("obstacle_mode_switcher_dual")

        # ---- Per-sensor parameters ----
        # Top LiDAR
        self.declare_parameter("scan_topic_top",       "/scan_top")
        self.declare_parameter("threshold_m_top",       2.0)
        self.declare_parameter("top_center_deg",       180.0)
        self.declare_parameter("top_half_deg",          45.0)

        # Low LiDAR
        self.declare_parameter("scan_topic_low",        "/scan_low")
        self.declare_parameter("threshold_m_low",        2.0)
        self.declare_parameter("low_center_deg",         0.0)
        self.declare_parameter("low_half_deg",          45.0)

        # Shared
        self.declare_parameter("min_valid_m",          0.05)
        self.declare_parameter("debounce_sec",          0.5)
        self.declare_parameter("cooldown_sec",          2.0)

        self.declare_parameter("mode_when_obstacle",   "GUIDED")
        self.declare_parameter("mode_when_clear",      "AUTO")

        self._sensor_topic = {
            "top": self.get_parameter("scan_topic_top").value,
            "low": self.get_parameter("scan_topic_low").value,
        }

        self._sensor = {
            sid: {"nearest_m": float("inf"), "detected": False, "last_change": time.time()}
            for sid in self.SENSOR_IDS
        }

        self._obstacle_stable = False
        self._active_sensors  = []
        self._last_mode_cmd_time  = 0.0
        self._last_commanded_mode = None
        self._current_ardupilot_mode = ""
        self._state_received = False          # block switching until first /mavros/state

        # MAVROS
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.get_logger().info("Waiting for /mavros/set_mode service ...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Still waiting for /mavros/set_mode ...")

        self.obstacle_pub = self.create_publisher(Bool, "/obstacle_detected", 10)

        for sid in self.SENSOR_IDS:
            self.create_subscription(
                LaserScan, self._sensor_topic[sid],
                lambda msg, s=sid: self._cb_scan(msg, s), 10)

        self.create_subscription(State, "/mavros/state", self._cb_state, 10)
        self.create_timer(0.1, self._control_loop)

        tt = self.get_parameter("threshold_m_top").value
        tc = self.get_parameter("top_center_deg").value
        th = self.get_parameter("top_half_deg").value
        lt = self.get_parameter("threshold_m_low").value
        lc = self.get_parameter("low_center_deg").value
        lh = self.get_parameter("low_half_deg").value
        self.get_logger().info(
            f"\nDual front-LiDAR obstacle-mode switcher started\n"
            f"  top (1.15m) -> {self._sensor_topic['top']}\n"
            f"    thresh={tt}m  sector={tc}° ±{th}°\n"
            f"  low (0.58m) -> {self._sensor_topic['low']}\n"
            f"    thresh={lt}m  sector={lc}° ±{lh}°\n"
            f"  MANUAL lock = ENABLED"
        )

    def _cb_state(self, msg: State):
        self._current_ardupilot_mode = msg.mode
        self._state_received = True

    def _cb_scan(self, msg: LaserScan, sensor_id: str):
        min_valid_m = float(self.get_parameter("min_valid_m").value)
        # Per-sensor params — re-read each callback so ros2 param set works at runtime
        threshold  = float(self.get_parameter(f"threshold_m_{sensor_id}").value)
        center_deg = float(self.get_parameter(f"{sensor_id}_center_deg").value)
        half_deg   = float(self.get_parameter(f"{sensor_id}_half_deg").value)

        nearest  = nearest_in_sector(msg, center_deg, half_deg, min_valid_m)
        detected = math.isfinite(nearest) and nearest <= threshold

        state = self._sensor[sensor_id]
        state["nearest_m"] = nearest
        if detected != state["detected"]:
            state["detected"]    = detected
            state["last_change"] = time.time()

    def _control_loop(self):
        debounce_sec = float(self.get_parameter("debounce_sec").value)
        cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        now = time.time()

        prev_active = set(self._active_sensors)
        active = [
            sid for sid in self.SENSOR_IDS
            if self._sensor[sid]["detected"]
            and (now - self._sensor[sid]["last_change"]) >= debounce_sec
        ]
        self._active_sensors  = active
        self._obstacle_stable = len(active) > 0

        if set(active) != prev_active:
            self._log_sensor_status()

        self.obstacle_pub.publish(Bool(data=self._obstacle_stable))

        # ---- Wait for first state before allowing ANY switching ----
        if not self._state_received:
            return

        # ---- Only switch if current mode is AUTO or GUIDED ----
        allowed_modes = {"AUTO", "GUIDED"}
        if self._current_ardupilot_mode not in allowed_modes:
            return

        mode_obstacle = str(self.get_parameter("mode_when_obstacle").value)
        mode_clear    = str(self.get_parameter("mode_when_clear").value)
        desired_mode  = mode_obstacle if self._obstacle_stable else mode_clear

        if (now - self._last_mode_cmd_time) < cooldown_sec:
            return
        if self._last_commanded_mode == desired_mode:
            return

        self._send_set_mode(desired_mode)

    def _log_sensor_status(self):
        mode_str = self._current_ardupilot_mode or "?"
        header = "[OBSTACLE DETECTED]" if self._obstacle_stable else "[ALL CLEAR]"
        manual_note = "  ⛔ MANUAL LOCK" if mode_str == "MANUAL" else ""
        header += f"  (mode: {mode_str}{manual_note})"

        lines = [header]
        heights = {"top": "1.15m", "low": "0.58m"}

        for sid in self.SENSOR_IDS:
            topic   = self._sensor_topic[sid]
            nearest = self._sensor[sid]["nearest_m"]
            stable  = sid in self._active_sensors
            thresh  = float(self.get_parameter(f"threshold_m_{sid}").value)
            h       = heights[sid]

            dist_str = f"{nearest:6.2f} m" if math.isfinite(nearest) else "   inf m"

            if stable:
                tag  = "TRIGGERED"
                note = f"  <-- TRIGGERED (≤{thresh}m)"
            elif self._sensor[sid]["detected"]:
                tag  = "debouncing"
                note = "  (debouncing...)"
            else:
                tag  = "clear     "
                note = ""

            lines.append(f"  {tag}  {topic:<18s} ({h}) – {dist_str}{note}")

        self.get_logger().info("\n".join(lines))

    def _send_set_mode(self, mode: str):
        req = SetMode.Request()
        req.base_mode   = 0
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        self._last_mode_cmd_time  = time.time()
        self._last_commanded_mode = mode

        if self._active_sensors:
            triggered = ", ".join(self._sensor_topic[s] for s in self._active_sensors)
            trigger_note = f"triggered by: {triggered}"
        else:
            trigger_note = "all sensors clear"

        self.get_logger().info(f"Requesting mode change -> {mode}  ({trigger_note})")

        def _done(fut):
            try:
                res = fut.result()
                if res is None:
                    self.get_logger().error(f"SetMode returned None (mode={mode})")
                    return
                if res.mode_sent:
                    self.get_logger().info(f"Mode accepted: {mode}")
                else:
                    self.get_logger().warn(f"Mode rejected / not sent: {mode}")
            except Exception as exc:
                self.get_logger().error(f"SetMode call failed (mode={mode}): {exc}")

        future.add_done_callback(_done)


def main():
    rclpy.init()
    node = DualObstacleModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
