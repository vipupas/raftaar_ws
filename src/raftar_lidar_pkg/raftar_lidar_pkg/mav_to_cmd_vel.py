#!/usr/bin/env python3
"""
mav_to_cmd_vel.py
~~~~~~~~~~~~~~~~~
The last link in the chain: turns Nav2's velocity commands into something
the flight controller understands.

    Nav2 controller -> /cmd_vel -> velocity_smoother -> /cmd_vel_smoothed
                                                             |
                                                             v
                                     [this node] -> /mavros/setpoint_raw/local
                                                             |
                                                             v
                                              ArduPilot, while in GUIDED

Two conventions meet here, and they disagree
--------------------------------------------
ROS (REP-103) uses ENU with z up, so a positive ``angular.z`` is a LEFT turn.
MAVLink's FRAME_BODY_NED uses NED with z down, so a positive ``yaw_rate``
is a RIGHT turn.  Passing the number straight through - as the original
code did - makes the rover turn *toward* the obstacle it is trying to
avoid.  ``invert_yaw_rate`` corrects for it.

Forward velocity needs no such treatment: +x is forward in both.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget

from raftar_lidar_pkg.rover_config import CMD_VEL_SMOOTHED_TOPIC, SETPOINT_TOPIC


class CmdVelToMavros(Node):

    def __init__(self):
        super().__init__('cmdvel_to_mavros')

        self.declare_parameter('input_topic', CMD_VEL_SMOOTHED_TOPIC)
        self.declare_parameter('output_topic', SETPOINT_TOPIC)
        # Bench-test with the wheels off the ground and confirm that a left
        # turn command turns the rover left before trusting this in a field.
        self.declare_parameter('invert_yaw_rate', True)
        # ArduPilot expects a steady stream of guided setpoints; if the
        # stream dries up it falls back to its own failsafe behaviour.
        self.declare_parameter('publish_rate_hz', 20.0)
        # If Nav2 goes quiet for longer than this, command a stop rather
        # than letting the last velocity stand.
        self.declare_parameter('cmd_timeout_sec', 0.5)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._last_cmd = Twist()
        self._last_rx_time = 0.0
        self._was_stale = True

        self.sub = self.create_subscription(Twist, input_topic, self.cb, 10)
        self.pub = self.create_publisher(PositionTarget, output_topic, 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Twist -> PositionTarget (BODY_NED) running.\n'
            f'  in   : {input_topic}\n'
            f'  out  : {output_topic}\n'
            f'  rate : {rate:.0f} Hz\n'
            f'  yaw_rate inverted: {self.get_parameter("invert_yaw_rate").value}')

    def cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_rx_time = self.get_clock().now().nanoseconds / 1e9

    def _publish(self):
        """Emit a setpoint every tick, whether or not Nav2 spoke."""
        timeout = float(self.get_parameter('cmd_timeout_sec').value)
        now = self.get_clock().now().nanoseconds / 1e9
        stale = (now - self._last_rx_time) > timeout

        if stale != self._was_stale:
            if stale:
                self.get_logger().warn('No cmd_vel received; commanding zero velocity.')
            else:
                self.get_logger().info('cmd_vel stream resumed.')
            self._was_stale = stale

        cmd = Twist() if stale else self._last_cmd

        out = PositionTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.coordinate_frame = PositionTarget.FRAME_BODY_NED

        # Tell the autopilot to use velocity and yaw rate only, and to
        # ignore the position and acceleration fields we are not filling in.
        out.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )

        out.velocity.x = cmd.linear.x
        out.velocity.y = 0.0        # skid-steer cannot move sideways
        out.velocity.z = 0.0

        yaw_rate = cmd.angular.z
        if bool(self.get_parameter('invert_yaw_rate').value):
            yaw_rate = -yaw_rate    # ROS left-positive -> MAVLink right-positive
        out.yaw_rate = yaw_rate

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelToMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
