#!/usr/bin/env python3
"""
coordinator_node.py
~~~~~~~~~~~~~~~~~~~
Decides *where to drive* while the rover is dodging an obstacle.

It does local planning only - no map server, no AMCL, no global planner,
no bt_navigator.  When an obstacle appears it hands Nav2's controller a
straight line from "where I am" to "the mission waypoint I was heading
for", and the local planner bends that line around whatever the LiDARs
can see.

Division of responsibility
--------------------------
  obstacle_mode_switcher_dual  owns the flight mode (AUTO <-> GUIDED).
  coordinator_node (this file) owns the Nav2 goal.
They talk over /obstacle_detected.  This node never changes mode itself -
two nodes fighting over one setting is a bug waiting to happen.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import WaypointList, WaypointReached, HomePosition
from mavros_msgs.srv import WaypointPull
from std_msgs.msg import Bool
from geographic_msgs.msg import GeoPointStamped
import tf2_ros

from raftar_lidar_pkg.geo import geodetic_to_enu
from raftar_lidar_pkg.rover_config import (
    MAP_FRAME, ODOM_FRAME, BASE_FRAME, CMD_VEL_TOPIC, OBSTACLE_TOPIC,
)

# MAVLink mission commands that carry a latitude/longitude we can drive to.
# Anything else (RTL, DO_*, conditionals) has no usable position.
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_SPLINE_WAYPOINT = 82
NAVIGABLE_COMMANDS = {MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_SPLINE_WAYPOINT}


class CoordinatorNode(Node):

    def __init__(self):
        super().__init__('coordinator_node')

        # ---- Parameters ----
        self.declare_parameter('stuck_timeout_sec', 8.0)
        self.declare_parameter('progress_check_interval_sec', 1.0)
        self.declare_parameter('min_progress_m', 0.1)
        self.declare_parameter('cmd_vel_topic', CMD_VEL_TOPIC)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        # Set False if MAVROS is configured to publish odom->base_link itself.
        # Two publishers of the same transform corrupts the TF tree.
        self.declare_parameter('publish_odom_tf', True)

        # ---- State ----
        self.obstacle_detected = False
        self.waypoints = []
        self.mission_current_seq = None    # from WaypointList.current_seq
        self.last_reached_seq = None       # from WaypointReached
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.home_is_ekf_origin = False    # True once gp_origin has been seen
        self.rover_pose = None
        self.nav2_goal_active = False
        self.nav2_goal_handle = None

        # Watchdog state
        self.goal_target_east = None
        self.goal_target_north = None
        self.best_distance_seen = None
        self.last_progress_time = None
        self.stopped_due_to_stuck = False   # latched until the obstacle clears

        # ---- TF: odom -> base_link, published early so the Nav2 costmap
        #      has a transform to work with before odometry arrives ----
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.05, self._broadcast_tf)

        # ---- Nav2 action client ----
        # controller_server serves /follow_path directly; no behaviour tree.
        self.nav2_client = ActionClient(self, FollowPath, '/follow_path')
        self.get_logger().info('Waiting for Nav2 /follow_path action server ...')
        while not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Still waiting for /follow_path ...')

        # ---- Emergency stop publisher ----
        self.stop_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.stop_hold_timer = self.create_timer(0.1, self._hold_stop)

        # ---- Mission pull ----
        self.pull_mission_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        self.pull_timer = self.create_timer(3.0, self._try_pull_mission)

        # ---- Subscribers ----
        self.create_subscription(Bool, OBSTACLE_TOPIC, self.cb_obstacle, 10)
        self.create_subscription(
            WaypointReached, '/mavros/mission/reached', self.cb_waypoint_reached, 10)
        self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self.cb_waypoints, 10)

        # gp_origin is latched, so it needs TRANSIENT_LOCAL to be received at all.
        gp_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            GeoPointStamped, '/mavros/global_position/gp_origin', self.cb_gp_origin, gp_qos)
        self.create_subscription(
            HomePosition, '/mavros/home_position/home', self.cb_home_position, 10)

        # MAVROS publishes sensor data BEST_EFFORT; a RELIABLE subscriber
        # would never connect to it.
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.cb_odom, odom_qos)

        # ---- Watchdog ----
        self.create_timer(
            float(self.get_parameter('progress_check_interval_sec').value),
            self.watchdog_check)

        self.get_logger().info('Coordinator node started (local planning / FollowPath).')

    # ------------------------------------------------------------------ #
    # Mission
    # ------------------------------------------------------------------ #

    def _try_pull_mission(self):
        """Ask the flight controller for its mission, until it answers."""
        if self.waypoints:
            self.pull_timer.cancel()
            return
        if not self.pull_mission_client.wait_for_service(timeout_sec=0.5):
            return

        future = self.pull_mission_client.call_async(WaypointPull.Request())

        def done_cb(fut):
            try:
                res = fut.result()
                if res and res.success:
                    self.get_logger().info(f'Mission pull OK, {res.wp_received} waypoints.')
                else:
                    self.get_logger().warn('Mission pull returned no success.')
            except Exception as exc:
                self.get_logger().error(f'Mission pull error: {exc}')

        future.add_done_callback(done_cb)

    def cb_waypoints(self, msg: WaypointList):
        self.waypoints = list(msg.waypoints)
        self.mission_current_seq = msg.current_seq
        self.get_logger().info(
            f'Mission loaded: {len(self.waypoints)} waypoints, '
            f'currently targeting #{msg.current_seq}.')

    def cb_waypoint_reached(self, msg: WaypointReached):
        self.last_reached_seq = msg.wp_seq
        self.get_logger().info(f'Waypoint {msg.wp_seq} reached.')

    def _target_waypoint(self):
        """The mission item the rover is currently driving toward.

        Returns (index, waypoint) or (None, None).

        Three sources, best first.  ``is_current`` is the flag ArduPilot
        itself sets, so it is authoritative; ``current_seq`` comes with the
        list; "last reached + 1" is the fallback.  The old code used only
        the fallback, which starts at 0 - and mission item 0 is *home*, so
        an obstacle met before the first waypoint sent the rover home.
        """
        if not self.waypoints:
            return None, None

        idx = None
        for i, wp in enumerate(self.waypoints):
            if wp.is_current:
                idx = i
                break
        if idx is None and self.mission_current_seq is not None:
            idx = self.mission_current_seq
        if idx is None and self.last_reached_seq is not None:
            idx = self.last_reached_seq + 1
        if idx is None:
            return None, None

        # Skip item 0 (home) and anything without a real position.
        for i in range(max(idx, 1), len(self.waypoints)):
            wp = self.waypoints[i]
            if wp.command not in NAVIGABLE_COMMANDS:
                continue
            if wp.x_lat == 0.0 and wp.y_long == 0.0:
                continue
            return i, wp

        return None, None

    # ------------------------------------------------------------------ #
    # Position
    # ------------------------------------------------------------------ #

    def cb_gp_origin(self, msg: GeoPointStamped):
        """The EKF origin - the exact point /mavros/local_position/odom is
        measured from.  This is the correct reference and it overrides any
        home position we may have latched earlier."""
        lat, lon, alt = msg.position.latitude, msg.position.longitude, msg.position.altitude
        if not self.home_is_ekf_origin:
            self.home_lat, self.home_lon, self.home_alt = lat, lon, alt
            self.home_is_ekf_origin = True
            self.get_logger().info(
                f'Local origin from gp_origin: {lat:.7f}, {lon:.7f}, {alt:.2f}')

    def cb_home_position(self, msg: HomePosition):
        """Fallback origin.  Home is usually the EKF origin but is not
        guaranteed to be, so gp_origin wins whenever it shows up."""
        if self.home_lat is None:
            self.home_lat = msg.geo.latitude
            self.home_lon = msg.geo.longitude
            self.home_alt = msg.geo.altitude
            self.get_logger().warn(
                f'Local origin from home_position (gp_origin not seen yet): '
                f'{self.home_lat:.7f}, {self.home_lon:.7f}')

    def cb_odom(self, msg: Odometry):
        self.rover_pose = msg.pose.pose

    def _broadcast_tf(self):
        """Publish odom -> base_link at 20 Hz.

        The rest of the tree is static: map -> odom comes from the launch
        file.  The old code published map -> base_link *and* a static
        map -> odom, which left odom dangling and would have broken the
        tree outright if MAVROS were ever configured to publish its own
        odom -> base_link.
        """
        if not self.get_parameter('publish_odom_tf').value:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME

        if self.rover_pose is not None:
            t.transform.translation.x = self.rover_pose.position.x
            t.transform.translation.y = self.rover_pose.position.y
            t.transform.translation.z = self.rover_pose.position.z
            t.transform.rotation = self.rover_pose.orientation
        else:
            t.transform.rotation.w = 1.0     # identity until odometry arrives

        self.tf_broadcaster.sendTransform(t)

    def gps_to_enu(self, lat, lon, alt):
        if self.home_lat is None:
            self.get_logger().error('No local origin yet; cannot convert GPS.')
            return None, None, None
        return geodetic_to_enu(lat, lon, alt, self.home_lat, self.home_lon, self.home_alt)

    # ------------------------------------------------------------------ #
    # Obstacle reaction
    # ------------------------------------------------------------------ #

    def cb_obstacle(self, msg: Bool):
        previous = self.obstacle_detected
        self.obstacle_detected = msg.data

        if self.obstacle_detected and not previous:
            if self.nav2_goal_active:
                return
            self.get_logger().info('Obstacle detected. Sending FollowPath goal.')
            self.stopped_due_to_stuck = False
            self.send_nav2_goal()

        elif not self.obstacle_detected and previous:
            self.get_logger().info('Obstacle cleared.')
            self.cancel_nav2_goal()
            self.stopped_due_to_stuck = False

    def send_nav2_goal(self):
        """Send a straight line from here to the target waypoint.

        The line is only a suggestion.  The local planner scores its own
        candidate trajectories against the costmap and deviates from this
        line as much as it needs to, which is what produces the avoidance
        manoeuvre.
        """
        if self.rover_pose is None:
            self.get_logger().warn('No odometry yet; cannot send Nav2 goal.')
            return

        index, wp = self._target_waypoint()
        if wp is None:
            self.get_logger().warn('No drivable mission waypoint; cannot send Nav2 goal.')
            return

        east, north, _ = self.gps_to_enu(wp.x_lat, wp.y_long, wp.z_alt)
        if east is None:
            return

        self.get_logger().info(
            f'FollowPath goal: waypoint #{index} -> ENU x={east:.2f}, y={north:.2f}')

        path = Path()
        path.header.frame_id = MAP_FRAME
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

        yaw = math.atan2(north - self.rover_pose.position.y,
                         east - self.rover_pose.position.x)
        target.pose.orientation.z = math.sin(yaw / 2.0)
        target.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(target)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = str(self.get_parameter('controller_id').value)
        goal.goal_checker_id = str(self.get_parameter('goal_checker_id').value)

        # Arm the watchdog for this attempt.
        self.goal_target_east = east
        self.goal_target_north = north
        self.best_distance_seen = self._distance_to_target()
        self.last_progress_time = time.time()

        self.nav2_client.send_goal_async(goal).add_done_callback(self.nav2_goal_response_cb)

    def nav2_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Nav2 goal send failed: {exc}')
            self._clear_goal_state()
            return

        if not goal_handle.accepted:
            # Do NOT latch nav2_goal_active here.  The old code set it True
            # with a null handle, so cancelling later fell through without
            # resetting it - and every subsequent obstacle was ignored for
            # the rest of the run, silently.
            self.get_logger().warn('Nav2 rejected the goal.')
            self._clear_goal_state()
            return

        self.get_logger().info('Nav2 accepted the goal. Avoiding obstacle ...')
        self.nav2_goal_active = True
        self.nav2_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.nav2_result_cb)

    def nav2_result_cb(self, future):
        try:
            status = future.result().status
            self.get_logger().info(f'Nav2 goal finished (status {status}).')
        except Exception as exc:
            self.get_logger().warn(f'Nav2 result error: {exc}')
        self._clear_goal_state()

    def _clear_goal_state(self):
        self.nav2_goal_active = False
        self.nav2_goal_handle = None

    def cancel_nav2_goal(self):
        if self.nav2_goal_handle is not None:
            self.get_logger().info('Cancelling Nav2 goal ...')
            self.nav2_goal_handle.cancel_goal_async()
        self._clear_goal_state()

    # ------------------------------------------------------------------ #
    # Watchdog
    # ------------------------------------------------------------------ #

    def _distance_to_target(self):
        if self.rover_pose is None or self.goal_target_east is None:
            return None
        return math.hypot(self.goal_target_east - self.rover_pose.position.x,
                          self.goal_target_north - self.rover_pose.position.y)

    def watchdog_check(self):
        """Stop the rover if the local planner cannot make headway.

        Without this, a rover facing an obstacle too wide to go around will
        creep and oscillate against it indefinitely.
        """
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
            self.best_distance_seen = dist
            self.last_progress_time = now
            return

        stuck_for = now - self.last_progress_time
        if stuck_for >= timeout:
            self.get_logger().error(
                f'No progress for {stuck_for:.1f}s with an obstacle present. '
                f'Forcing a safe stop.')
            self.force_stop()

    def force_stop(self):
        """Cancel the goal and hold the rover still.

        Stays in GUIDED on purpose.  Handing control back to AUTO would let
        the flight controller drive straight back into the obstacle it just
        failed to get around.
        """
        self.cancel_nav2_goal()
        self.stopped_due_to_stuck = True
        self.get_logger().error(
            'STOPPED: obstacle could not be avoided locally. '
            'Holding position in GUIDED - manual intervention needed.')

    def _hold_stop(self):
        """Keep asserting zero velocity while latched stopped.

        A single burst of stop messages can be missed; a steady stream at
        10 Hz cannot.
        """
        if self.stopped_due_to_stuck:
            self.stop_pub.publish(Twist())


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
