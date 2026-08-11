#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget

class CmdVelToMavros(Node):
    def __init__(self):
        super().__init__('cmdvel_to_mavros')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.get_logger().info("Twist -> PositionTarget (BODY_NED) converter running.")

    def cb(self, msg: Twist):
        out = PositionTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.coordinate_frame = PositionTarget.FRAME_BODY_NED
        out.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        out.velocity.x = msg.linear.x
        out.velocity.y = 0.0
        out.velocity.z = 0.0
        out.yaw_rate = msg.angular.z
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
