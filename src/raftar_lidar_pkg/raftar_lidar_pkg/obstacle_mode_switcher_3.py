#!/usr/bin/env python3
"""
obstacle_mode_switcher.py
~~~~~~~~~~~~~~~~~~~~~~~~~
Monitors three LiDAR scans and switches MAVROS flight mode based on
obstacle detection.  Logs which sensor(s) triggered the obstacle state.

LiDAR layout
------------
  /scan        – top-mounted LiDAR    (full 360°, checks front sector)
  /scan_left   – left-side LiDAR      (checks its own forward-facing sector)
  /scan_right  – right-side LiDAR     (checks its own forward-facing sector)

Logic
-----
  obstacle = top_obstacle OR left_obstacle OR right_obstacle   (any-OR policy)
  obstacle present (debounced)  → mode_when_obstacle  (default: GUIDED)
  obstacle absent  (debounced)  → mode_when_clear     (default: MANUAL)

Console output example
----------------------
  [OBSTACLE DETECTED]
    ✓ /scan        –  0.73 m   <-- TRIGGERED
    ✗ /scan_left   –  2.41 m   (clear)
    ✗ /scan_right  –  inf  m   (clear)
  → Requesting mode change: GUIDED  (triggered by: /scan)
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import SetMode

from raftar_lidar_pkg.scan_utils import nearest_in_sector


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ObstacleModeSwitcher(Node):
    """Three-LiDAR obstacle-aware MAVROS mode switcher."""

    SENSOR_IDS = ("top", "left", "right")

    def __init__(self):
        super().__init__("obstacle_mode_switcher")

        # ------------------------------------------------------------------ #
        # Parameters                                                           #
        # ------------------------------------------------------------------ #

        self.declare_parameter("scan_topic_top",   "/scan")
        self.declare_parameter("scan_topic_left",  "/scan_left")
        self.declare_parameter("scan_topic_right", "/scan_right")

        self.declare_parameter("threshold_m",  0.5)
        self.declare_parameter("min_valid_m",  0.05)

        # Sector angles per sensor (center + half-width, degrees, in sensor frame)
        self.declare_parameter("top_center_deg",   0.0)
        self.declare_parameter("top_half_deg",     20.0)
        self.declare_parameter("left_center_deg",  0.0)
        self.declare_parameter("left_half_deg",    30.0)
        self.declare_parameter("right_center_deg", 0.0)
        self.declare_parameter("right_half_deg",   30.0)

        self.declare_parameter("debounce_sec",  0.5)
        self.declare_parameter("cooldown_sec",  2.0)

        self.declare_parameter("mode_when_obstacle", "GUIDED")
        self.declare_parameter("mode_when_clear",    "AUTO")

        # ------------------------------------------------------------------ #
        # Sensor topic map (resolved once at startup)                          #
        # ------------------------------------------------------------------ #
        self._sensor_topic = {
            "top":   self.get_parameter("scan_topic_top").value,
            "left":  self.get_parameter("scan_topic_left").value,
            "right": self.get_parameter("scan_topic_right").value,
        }

        # Per-sensor runtime state
        #   nearest_m  : last measured nearest distance in sector
        #   detected   : raw obstacle flag
        #   last_change: epoch time of last raw flag flip (for debounce)
        self._sensor = {
            sid: {
                "nearest_m":   float("inf"),
                "detected":    False,
                "last_change": time.time(),
            }
            for sid in self.SENSOR_IDS
        }

        # Debounced combined state
        self._obstacle_stable = False
        self._active_sensors  = []   # sensor IDs currently stable+triggered

        # Mode-command tracking
        self._last_mode_cmd_time  = 0.0
        self._last_commanded_mode = None

        # ------------------------------------------------------------------ #
        # MAVROS service                                                       #
        # ------------------------------------------------------------------ #
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.get_logger().info("Waiting for /mavros/set_mode service ...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Still waiting for /mavros/set_mode — is MAVROS running and connected?"
            )

        # ------------------------------------------------------------------ #
        # Subscriptions                                                        #
        # ------------------------------------------------------------------ #
        for sid in self.SENSOR_IDS:
            topic = self._sensor_topic[sid]
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, s=sid: self._cb_scan(msg, s),
                10,
            )

        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            "\nObstacle-mode switcher started\n"
            f"  top   -> {self._sensor_topic['top']}\n"
            f"  left  -> {self._sensor_topic['left']}\n"
            f"  right -> {self._sensor_topic['right']}\n"
            f"  threshold = {self.get_parameter('threshold_m').value} m"
        )

    # ------------------------------------------------------------------ #
    # Scan callbacks                                                       #
    # ------------------------------------------------------------------ #

    def _cb_scan(self, msg: LaserScan, sensor_id: str):
        threshold   = float(self.get_parameter("threshold_m").value)
        min_valid_m = float(self.get_parameter("min_valid_m").value)
        center_deg  = float(self.get_parameter(f"{sensor_id}_center_deg").value)
        half_deg    = float(self.get_parameter(f"{sensor_id}_half_deg").value)

        nearest = nearest_in_sector(
            msg,
            center_deg=center_deg,
            half_deg=half_deg,
            min_valid_m=min_valid_m,
        )

        detected = math.isfinite(nearest) and nearest <= threshold

        state = self._sensor[sensor_id]
        state["nearest_m"] = nearest       # always update distance reading

        if detected != state["detected"]:  # only reset debounce timer on flip
            state["detected"]    = detected
            state["last_change"] = time.time()

    # ------------------------------------------------------------------ #
    # Control loop (10 Hz)                                                 #
    # ------------------------------------------------------------------ #

    def _control_loop(self):
        debounce_sec = float(self.get_parameter("debounce_sec").value)
        cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        threshold    = float(self.get_parameter("threshold_m").value)
        now = time.time()

        # ------------------------------------------------------------------
        # Debounce each sensor, collect those that are stably triggering
        # ------------------------------------------------------------------
        prev_active = set(self._active_sensors)
        active = [
            sid for sid in self.SENSOR_IDS
            if self._sensor[sid]["detected"]
            and (now - self._sensor[sid]["last_change"]) >= debounce_sec
        ]

        self._active_sensors  = active
        self._obstacle_stable = len(active) > 0

        # Log a status block whenever the triggering set changes
        if set(active) != prev_active:
            self._log_sensor_status(threshold)

        # ------------------------------------------------------------------
        # Mode switching
        # ------------------------------------------------------------------
        mode_obstacle = str(self.get_parameter("mode_when_obstacle").value)
        mode_clear    = str(self.get_parameter("mode_when_clear").value)
        desired_mode  = mode_obstacle if self._obstacle_stable else mode_clear

        if (now - self._last_mode_cmd_time) < cooldown_sec:
            return
        if self._last_commanded_mode == desired_mode:
            return

        self._send_set_mode(desired_mode)

    # ------------------------------------------------------------------ #
    # Status logger                                                        #
    # ------------------------------------------------------------------ #

    def _log_sensor_status(self, threshold: float):
        """
        Prints a table like:

          [OBSTACLE DETECTED]
            TRIGGERED  /scan        –  0.73 m  <-- TRIGGERED
            clear      /scan_left   –  2.41 m
            clear      /scan_right  –   inf m
        """
        header = "[OBSTACLE DETECTED]" if self._obstacle_stable else "[ALL CLEAR]"
        lines  = [header]

        for sid in self.SENSOR_IDS:
            topic   = self._sensor_topic[sid]
            nearest = self._sensor[sid]["nearest_m"]
            stable  = sid in self._active_sensors

            dist_str = f"{nearest:6.2f} m" if math.isfinite(nearest) else "   inf m"

            if stable:
                tag  = "TRIGGERED"
                note = "  <-- TRIGGERED"
            elif self._sensor[sid]["detected"]:
                tag  = "debouncing"
                note = "  (debouncing...)"
            else:
                tag  = "clear     "
                note = ""

            lines.append(f"  {tag}  {topic:<18s} – {dist_str}{note}")

        self.get_logger().info("\n".join(lines))

    # ------------------------------------------------------------------ #
    # MAVROS set_mode                                                      #
    # ------------------------------------------------------------------ #

    def _send_set_mode(self, mode: str):
        req = SetMode.Request()
        req.base_mode   = 0
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)

        self._last_mode_cmd_time  = time.time()
        self._last_commanded_mode = mode

        if self._active_sensors:
            triggered_topics = ", ".join(
                self._sensor_topic[s] for s in self._active_sensors
            )
            trigger_note = f"triggered by: {triggered_topics}"
        else:
            trigger_note = "all sensors clear"

        self.get_logger().info(
            f"Requesting mode change -> {mode}  ({trigger_note})"
        )

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


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = ObstacleModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
