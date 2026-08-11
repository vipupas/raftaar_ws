#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import WaypointList, WaypointReached, HomePosition, State
from mavros_msgs.srv import SetMode, WaypointPull
from std_msgs.msg import Bool
from geographic_msgs.msg import GeoPointStamped
import tf2_ros

import pymap3d


class CoordinatorNode(Node):
    """
    Coordinator node (LOCAL PLANNING ONLY - no map, no AMCL, no bt_navigator):
    - Monitors obstacle detection from obstacle_mode_switcher
    - Reads current waypoint from ArduPilot mission
    - When obstacle detected  -> switch to GUIDED -> send a FollowPath goal
      (straight line from current pose to the current mission waypoint)
      to controller_server's local DWB planner
    - When obstacle cleared   -> cancel FollowPath goal -> switch back to AUTO
    - Watchdog: if the local planner cannot make progress within a timeout
      while an obstacle is still present, cancel the goal and force a
      zero-velocity stop instead of letting it oscillate/creep indefinitely.
    """

    def __init__(self):
        super().__init__('coordinator_node')

        # ---- Parameters ----
        self.declare_parameter('cooldown_sec', 2.0)
        self.declare_parameter('mode_guided', 'GUIDED')
        self.declare_parameter('mode_auto', 'AUTO')
        self.declare_parameter('stuck_timeout_sec', 8.0)   # watchdog: max time to let Nav2 try before forcing a stop
        self.declare_parameter('progress_check_interval_sec', 1.0)
        self.declare_parameter('min_progress_m', 0.1)       # must close at least this much distance to count as "progress"
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ---- State ----
        self.obstacle_detected   = False       # latest obstacle state from obstacle node
        self.current_wp_index    = 0           # current waypoint index rover is heading to
        self.waypoints           = []          # full mission waypoint list
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.rover_pose          = None        # current rover pose in odom frame
        self.nav2_goal_active    = False       # whether Nav2 goal is currently active
        self.nav2_goal_handle    = None        # handle to cancel Nav2 goal
        self.last_mode_cmd_time  = 0.0
        self.current_mode        = ''          # current flight mode from MAVROS

        # Watchdog state
        self.goal_target_east    = None
        self.goal_target_north   = None
        self.goal_start_time     = None
        self.best_distance_seen  = None
        self.last_progress_time  = None
        self.stopped_due_to_stuck = False       # latched: don't retry until obstacle fully clears

        # ---- TF broadcaster: map -> base_link (MUST start early so Nav2 costmap doesn't stall) ----
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.05, self._broadcast_tf)

        # ---- MAVROS set_mode service ----
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.get_logger().info("Waiting for /mavros/set_mode ...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Still waiting for /mavros/set_mode ...")

        # ---- Nav2 action client (FollowPath, served directly by controller_server -
        #      no bt_navigator / no global planner / no map required) ----
        self.nav2_client = ActionClient(self, FollowPath, '/follow_path')
        self.get_logger().info("Waiting for Nav2 controller_server /follow_path action server ...")
        while not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Still waiting for /follow_path ...")

        # ---- Direct cmd_vel publisher for emergency/watchdog stop ----
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.stop_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ---- MAVROS pull mission service ----
        self.pull_mission_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        self.pull_timer = self.create_timer(3.0, self._try_pull_mission)

        # ---- Subscribers ----

        # Obstacle state from obstacle_mode_switcher node
        self.create_subscription(Bool, '/obstacle_detected', self.cb_obstacle, 10)

        # Current waypoint index being targeted by ArduPilot
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.cb_waypoint_reached, 10)

        # Full mission waypoint list
        self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.cb_waypoints, 10)

        # Home position for GPS -> ENU conversion
        gp_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(HomePosition, '/mavros/home_position/home', self.cb_home_position, 10)
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.cb_gp_origin, gp_qos)

        # Rover odometry for current position
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, '/mavros/local_position/odom', self.cb_odom, odom_qos)

        # Current mode from MAVROS
        self.create_subscription(State, '/mavros/state', self.cb_state, 10)

        # ---- Watchdog timer ----
        interval = float(self.get_parameter('progress_check_interval_sec').value)
        self.watchdog_timer = self.create_timer(interval, self.watchdog_check)

        self.get_logger().info("Coordinator node started (local planning / FollowPath mode).")

    def _try_pull_mission(self):
        """Repeatedly pull mission from MAVROS until waypoints arrive."""
        if self.waypoints:
            self.pull_timer.cancel()
            return

        if not self.pull_mission_client.wait_for_service(timeout_sec=0.5):
            return

        req = WaypointPull.Request()
        future = self.pull_mission_client.call_async(req)

        def done_cb(fut):
            try:
                res = fut.result()
                if res and res.success:
                    self.get_logger().info(f"Mission pull successful, {res.wp_received} waypoints received.")
                else:
                    self.get_logger().warn("Mission pull returned no success.")
            except Exception as e:
                self.get_logger().error(f"Mission pull error: {e}")

        future.add_done_callback(done_cb)

    # ---------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------

    def cb_obstacle(self, msg: Bool):
        """Receive obstacle detection state from obstacle_mode_switcher.
        The obstacle node handles mode switching (GUIDED/AUTO).
        This coordinator only manages Nav2 FollowPath goals."""
        previous = self.obstacle_detected
        self.obstacle_detected = msg.data

        if self.obstacle_detected and not previous:
            if self.nav2_goal_active:
                return
            self.get_logger().info("Obstacle detected! Sending FollowPath goal.")
            self.stopped_due_to_stuck = False
            self.send_nav2_goal()

        elif not self.obstacle_detected and previous:
            if not self.nav2_goal_active and not self.nav2_goal_handle:
                self.get_logger().info("Obstacle cleared. No active Nav2 goal to cancel.")
                return
            self.get_logger().info("Obstacle cleared! Cancelling Nav2 goal.")
            self.cancel_nav2_goal()
            self.stopped_due_to_stuck = False

    def cb_waypoint_reached(self, msg: WaypointReached):
        """Track which waypoint was last reached -> next one is current target."""
        self.current_wp_index = msg.wp_seq + 1
        self.get_logger().info(f"Waypoint {msg.wp_seq} reached. Now heading to waypoint {self.current_wp_index}.")

    def cb_waypoints(self, msg: WaypointList):
        """Store the full mission waypoint list."""
        self.waypoints = msg.waypoints
        self.get_logger().info(f"Mission loaded: {len(self.waypoints)} waypoints.")

    def cb_home_position(self, msg: HomePosition):
        """Store home GPS position for GPS -> ENU conversion."""
        if self.home_lat is None:
            self.home_lat = msg.geo.latitude
            self.home_lon = msg.geo.longitude
            self.home_alt = msg.geo.altitude
            self.get_logger().info(f"Home: lat={self.home_lat:.6f}, lon={self.home_lon:.6f}, alt={self.home_alt:.2f}")

    def cb_gp_origin(self, msg):
        """Fallback: home GPS from latched gp_origin."""
        if self.home_lat is None:
            self.home_lat = msg.position.latitude
            self.home_lon = msg.position.longitude
            self.home_alt = msg.position.altitude
            self.get_logger().info(f"Home (gp_origin): lat={self.home_lat:.6f}, lon={self.home_lon:.6f}, alt={self.home_alt:.2f}")

    def cb_odom(self, msg: Odometry):
        """Store current rover pose (TF broadcasting handled by timer)."""
        self.rover_pose = msg.pose.pose

    def cb_state(self, msg: State):
        """Track current ArduPilot mode."""
        self.current_mode = msg.mode

    # ---------------------------------------------------------------
    # Mode switching
    # ---------------------------------------------------------------

    def switch_mode(self, mode: str):
        """Send set_mode request to MAVROS."""
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)

        def done_cb(fut):
            try:
                res = fut.result()
                if res and res.mode_sent:
                    self.get_logger().info(f"Mode switched to: {mode}")
                else:
                    self.get_logger().warn(f"Mode switch failed: {mode}")
            except Exception as e:
                self.get_logger().error(f"set_mode error: {e}")

        future.add_done_callback(done_cb)

    # ---------------------------------------------------------------
    # TF broadcasting
    # ---------------------------------------------------------------

    def _broadcast_tf(self):
        """Publish map -> base_link transform at 20Hz. Falls back to identity
        before odometry arrives so Nav2 costmap doesn't stall on startup."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        if self.rover_pose is not None:
            t.transform.translation.x = self.rover_pose.position.x
            t.transform.translation.y = self.rover_pose.position.y
            t.transform.translation.z = self.rover_pose.position.z
            t.transform.rotation = self.rover_pose.orientation
        else:
            t.transform.rotation.w = 1.0  # identity quaternion

        self.tf_broadcaster.sendTransform(t)

    # ---------------------------------------------------------------
    # GPS -> ENU conversion
    # ---------------------------------------------------------------

    def gps_to_enu(self, lat: float, lon: float, alt: float):
        if self.home_lat is None:
            self.get_logger().error("Home position not received yet! Cannot convert GPS to ENU.")
            return None, None, None
        east, north, up = pymap3d.geodetic2enu(lat, lon, alt, self.home_lat, self.home_lon, self.home_alt)
        return east, north, up

    # ---------------------------------------------------------------
    # Nav2 goal (FollowPath - local planner only, no map/global planner)
    # ---------------------------------------------------------------

    def send_nav2_goal(self):
        """Build a simple straight-line path (current pose -> target waypoint)
        and send it to controller_server's FollowPath action. DWB will locally
        deviate from this line to avoid obstacles seen in the local costmap."""

        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded from mission. Cannot send Nav2 goal.")
            return

        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().warn("Current waypoint index out of range.")
            return

        if self.rover_pose is None:
            self.get_logger().warn("No odometry yet. Cannot send Nav2 goal.")
            return

        wp = self.waypoints[self.current_wp_index]

        # Convert GPS waypoint to local ENU
        east, north, up = self.gps_to_enu(wp.x_lat, wp.y_long, wp.z_alt)
        if east is None:
            return

        self.get_logger().info(
            f"Sending FollowPath goal: waypoint {self.current_wp_index} "
            f"-> ENU: x={east:.2f}, y={north:.2f}"
        )

        # Build a 2-point straight-line Path: current pose -> target waypoint
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        start = PoseStamped()
        start.header = path.header
        start.pose = self.rover_pose
        path.poses.append(start)

        target = PoseStamped()
        target.header = path.header
        target.pose.position.x = east
        target.pose.position.y = north
        target.pose.position.z = 0.0

        dx = east - self.rover_pose.position.x
        dy = north - self.rover_pose.position.y
        yaw = math.atan2(dy, dx)
        target.pose.orientation.z = math.sin(yaw / 2.0)
        target.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(target)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'

        # ---- Watchdog init for this goal ----
        self.goal_target_east = east
        self.goal_target_north = north
        self.goal_start_time = time.time()
        self.best_distance_seen = self._distance_to_target()
        self.last_progress_time = time.time()

        send_goal_future = self.nav2_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.nav2_goal_response_cb)

    def nav2_goal_response_cb(self, future):
        """Called when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal!")
            self.nav2_goal_active = True   # latch — don't retry, obstacle is still present
            self.nav2_goal_handle = None
            return

        self.get_logger().info("Nav2 accepted the goal. Avoiding obstacle...")
        self.nav2_goal_active = True
        self.nav2_goal_handle = goal_handle

        # Monitor result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_cb)

    def nav2_result_cb(self, future):
        """Called when Nav2 finishes the goal."""
        self.nav2_goal_active = False
        self.nav2_goal_handle = None
        self.get_logger().info("Nav2 goal completed.")

    def cancel_nav2_goal(self):
        """Cancel active Nav2 goal."""
        if self.nav2_goal_handle is not None and self.nav2_goal_active:
            self.get_logger().info("Cancelling Nav2 goal...")
            self.nav2_goal_handle.cancel_goal_async()
            self.nav2_goal_active = False
            self.nav2_goal_handle = None
        else:
            self.get_logger().info("No active Nav2 goal to cancel.")

    # ---------------------------------------------------------------
    # Watchdog: detect "stuck" (no progress) and force a safe stop
    # ---------------------------------------------------------------

    def _distance_to_target(self):
        if self.rover_pose is None or self.goal_target_east is None:
            return None
        dx = self.goal_target_east - self.rover_pose.position.x
        dy = self.goal_target_north - self.rover_pose.position.y
        return math.hypot(dx, dy)

    def watchdog_check(self):
        """Runs periodically. If the local planner is active but not making
        real progress toward the target for longer than stuck_timeout_sec,
        cancel the goal and force a zero-velocity stop. Requires the obstacle
        to fully clear (handled in cb_obstacle) before trying again."""

        if not self.nav2_goal_active or self.stopped_due_to_stuck:
            return
        if not self.obstacle_detected:
            return

        dist = self._distance_to_target()
        if dist is None:
            return

        min_progress = float(self.get_parameter('min_progress_m').value)
        timeout = float(self.get_parameter('stuck_timeout_sec').value)
        now = time.time()

        if self.best_distance_seen is None or (self.best_distance_seen - dist) >= min_progress:
            # Real progress made -> reset the clock
            self.best_distance_seen = dist
            self.last_progress_time = now
            return

        stuck_duration = now - self.last_progress_time
        if stuck_duration >= timeout:
            self.get_logger().error(
                f"No progress for {stuck_duration:.1f}s while obstacle present. "
                f"Forcing safe stop (obstacle likely too large to avoid locally)."
            )
            self.force_stop()

    def force_stop(self):
        """Cancel Nav2 goal and publish zero velocity directly. Stays latched
        (stopped_due_to_stuck=True) until the obstacle fully clears, so the
        rover does not repeatedly retry into a blocking obstacle. Remains in
        GUIDED mode (does NOT resume AUTO) since resuming the mission could
        drive it straight back into the same obstacle."""
        self.cancel_nav2_goal()
        self.stopped_due_to_stuck = True

        stop_msg = Twist()  # all zeros
        for _ in range(5):
            self.stop_pub.publish(stop_msg)

        self.get_logger().error(
            "STOPPED: obstacle could not be avoided by local planner. "
            "Remaining in GUIDED and holding position. Manual intervention needed."
        )


def main():
    rclpy.init()
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
