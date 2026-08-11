#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from mavros_msgs.srv import SetMode


class ObstacleModeSwitcher(Node):
    """
    If obstacle within threshold -> set mode GUIDED
    If no obstacle -> set mode AUTO

    Includes:
      - front sector check (front_center_deg ± front_angle_deg)
      - debounce time to avoid flicker
      - cooldown to avoid spamming set_mode
      - requires MAVROS connected
    """

    def __init__(self):
        super().__init__('obstacle_mode_switcher')

        # ---- Parameters ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('threshold_m', 2.0)
        self.declare_parameter('front_angle_deg', 90.0)   # half-width of the sector to check
        self.declare_parameter('front_center_deg', 180.0) # center of the front sector in your LiDAR frame
                                                           # 0   = angle_min reference direction
                                                           # 180 = opposite side (common when LiDAR is rear-mounted)
        self.declare_parameter('min_valid_m', 0.05)

        # Debounce: obstacle must stay present/absent for this long before switching
        self.declare_parameter('debounce_sec', 0.5)

        # Cooldown: minimum time between mode change service calls
        self.declare_parameter('cooldown_sec', 2.0)

        # Modes
        self.declare_parameter('mode_when_obstacle', 'GUIDED')
        self.declare_parameter('mode_when_clear', 'AUTO')

        scan_topic = self.get_parameter('scan_topic').value

        # ---- MAVROS service client ----
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.get_logger().info("Waiting for /mavros/set_mode service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Still waiting for /mavros/set_mode ... is mavros running & connected?")

        # ---- Obstacle state publisher (coordinator node subscribes to this) ----
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        # ---- State ----
        self.current_obstacle_raw = False     # latest raw obstacle detection
        self.obstacle_stable = False          # debounced result (used for switching)
        self.last_raw_change_time = time.time()

        self.last_mode_cmd_time = 0.0
        self.last_commanded_mode = None

        # ---- Subscribers ----
        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.cb_scan, 10)

        # Timer to handle debounce + switching
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Obstacle-mode switcher running. "
            f"scan={scan_topic}, "
            f"threshold={self.get_parameter('threshold_m').value}m, "
            f"front sector center={self.get_parameter('front_center_deg').value}° "
            f"±{self.get_parameter('front_angle_deg').value}°"
        )

    def cb_scan(self, msg: LaserScan):
        threshold       = float(self.get_parameter('threshold_m').value)
        front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        front_center_deg = float(self.get_parameter('front_center_deg').value)
        min_valid_m     = float(self.get_parameter('min_valid_m').value)

        ranges = msg.ranges
        if not ranges:
            return

        a_min = msg.angle_min
        a_inc = msg.angle_increment

        def angle_to_index(a_rad: float) -> int:
            return int(round((a_rad - a_min) / a_inc))

        # Pivot the sector around front_center_deg with wrap-around support.
        # The scan spans [angle_min, angle_max]. When the desired sector crosses
        # the ±180° boundary it wraps; we handle this by building a list of
        # (start_idx, end_idx) segments and scanning all of them.
        center_rad = math.radians(front_center_deg)
        half_rad   = math.radians(front_angle_deg)

        n          = len(ranges)
        a_max      = msg.angle_max

        lo_rad = center_rad - half_rad   # left  edge of sector
        hi_rad = center_rad + half_rad   # right edge of sector

        def clamp_index(idx: int) -> int:
            return max(0, min(n - 1, idx))

        # Check whether the sector overflows the scan boundaries
        segments = []

        if lo_rad < msg.angle_min:
            # Sector wraps past the low boundary → split into two parts:
            #   [wrapped_lo .. a_max]  and  [a_min .. hi]
            wrap_lo = lo_rad + 2 * math.pi      # equivalent angle on the high side
            segments.append((clamp_index(angle_to_index(wrap_lo)), n - 1))
            segments.append((0, clamp_index(angle_to_index(hi_rad))))

        elif hi_rad > a_max:
            # Sector wraps past the high boundary → split into two parts:
            #   [lo .. a_max]  and  [a_min .. wrapped_hi]
            wrap_hi = hi_rad - 2 * math.pi      # equivalent angle on the low side
            segments.append((clamp_index(angle_to_index(lo_rad)), n - 1))
            segments.append((0, clamp_index(angle_to_index(wrap_hi))))

        else:
            # No wrap — single contiguous segment
            segments.append((clamp_index(angle_to_index(lo_rad)),
                             clamp_index(angle_to_index(hi_rad))))

        nearest = float('inf')
        for seg_start, seg_end in segments:
            if seg_start > seg_end:
                seg_start, seg_end = seg_end, seg_start
            for i in range(seg_start, seg_end + 1):
                r = ranges[i]
                if r <= min_valid_m or math.isinf(r) or math.isnan(r):
                    continue
                if r < nearest:
                    nearest = r

        obstacle = (math.isfinite(nearest) and nearest <= threshold)

        # Track raw change time for debounce
        if obstacle != self.current_obstacle_raw:
            self.current_obstacle_raw = obstacle
            self.last_raw_change_time = time.time()

    def control_loop(self):
        debounce_sec = float(self.get_parameter('debounce_sec').value)
        cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        now = time.time()

        # Debounce logic: only accept new obstacle state if it stayed the same for debounce_sec
        if (now - self.last_raw_change_time) >= debounce_sec:
            self.obstacle_stable = self.current_obstacle_raw

        # Publish obstacle state for coordinator node
        self.obstacle_pub.publish(Bool(data=self.obstacle_stable))

        # Decide desired mode
        mode_obstacle = str(self.get_parameter('mode_when_obstacle').value)
        mode_clear    = str(self.get_parameter('mode_when_clear').value)

        desired_mode = mode_obstacle if self.obstacle_stable else mode_clear

        # Cooldown: don't spam set_mode
        if (now - self.last_mode_cmd_time) < cooldown_sec:
            return

        # Don't re-send the same mode repeatedly
        if self.last_commanded_mode == desired_mode:
            return

        # Call set_mode
        self.send_set_mode(desired_mode)

    def send_set_mode(self, mode: str):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)

        # Update time immediately to avoid spamming if service is slow
        self.last_mode_cmd_time = time.time()
        self.last_commanded_mode = mode

        def done_cb(fut):
            try:
                res = fut.result()
                if res is None:
                    self.get_logger().error(f"SetMode returned None for mode={mode}")
                    return
                if res.mode_sent:
                    self.get_logger().info(f"Mode change requested: {mode} (mode_sent=True)")
                else:
                    self.get_logger().warn(f"Mode change rejected/not sent: {mode} (mode_sent=False)")
            except Exception as e:
                self.get_logger().error(f"SetMode service call failed for mode={mode}: {e}")

        future.add_done_callback(done_cb)


def main():
    rclpy.init()
    node = ObstacleModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
