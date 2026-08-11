#!/usr/bin/env python3
"""
obstacle_mode_switcher_dual.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Watches two LiDARs and hands control to ROS when either one sees
something in the way.

Both sensors look ahead of the rover, at different heights, so that each
catches what the other would miss - a kerb or a tow bar low down, a
branch or a wing mirror higher up.

  /scan_top   mounted ~1.15 m up, sensor body rotated 180 deg
  /scan_low   mounted ~0.58 m up, sensor body facing forwards

Because the two are bolted on at different angles, "straight ahead" is a
different bearing in each one's data.  Both bearings are derived from the
mount angles in rover_config.py, which is the same place the transforms
come from - so the detector and the costmap can never disagree about
which way a sensor faces.

Logic
-----
  obstacle = top_triggered OR low_triggered   (either one is enough)
  obstacle present (held for debounce_sec) -> mode_when_obstacle (GUIDED)
  obstacle absent  (held for debounce_sec) -> mode_when_clear    (AUTO)

Safety: if the rover is in any mode other than AUTO or GUIDED - MANUAL,
HOLD, RTL - this node issues no commands at all.  A human who has taken
over stays in control.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

from raftar_lidar_pkg.scan_utils import nearest_in_sector
from raftar_lidar_pkg.rover_config import (
    TOP_SCAN_TOPIC, LOW_SCAN_TOPIC, OBSTACLE_TOPIC,
    TOP_LIDAR_YAW_DEG, LOW_LIDAR_YAW_DEG, forward_bearing_deg,
    OBSTACLE_THRESHOLD_TOP_M, OBSTACLE_THRESHOLD_LOW_M, DETECTION_HALF_ANGLE_DEG,
)


class DualObstacleModeSwitcher(Node):
    """Two front-facing LiDAR obstacle mode switcher with MANUAL-lock.

    Each LiDAR has its own threshold, center angle, and half angle."""

    SENSOR_IDS = ("top", "low")

    def __init__(self):
        super().__init__("obstacle_mode_switcher_dual")

        # ---- Per-sensor parameters ----
        # Defaults come from rover_config so that the sector each LiDAR
        # watches always matches the transform that tells the costmap
        # which way that same LiDAR is pointing.
        # Top LiDAR
        self.declare_parameter("scan_topic_top",       TOP_SCAN_TOPIC)
        self.declare_parameter("threshold_m_top",      OBSTACLE_THRESHOLD_TOP_M)
        self.declare_parameter("top_center_deg",       forward_bearing_deg(TOP_LIDAR_YAW_DEG))
        self.declare_parameter("top_half_deg",         DETECTION_HALF_ANGLE_DEG)

        # Low LiDAR
        self.declare_parameter("scan_topic_low",       LOW_SCAN_TOPIC)
        self.declare_parameter("threshold_m_low",      OBSTACLE_THRESHOLD_LOW_M)
        self.declare_parameter("low_center_deg",       forward_bearing_deg(LOW_LIDAR_YAW_DEG))
        self.declare_parameter("low_half_deg",         DETECTION_HALF_ANGLE_DEG)

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

        self.obstacle_pub = self.create_publisher(Bool, OBSTACLE_TOPIC, 10)

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
