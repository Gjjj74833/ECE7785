#!/usr/bin/env python3
# Lab 2 - rotate_robot ROS2 Node
# CS/ME/ECE/AE/BME 7785 - Introduction to Robotics Research
# Georgia Institute of Technology
#
# Subscribes to /object_coord (Point published by find_object node).
# Turns the TurtleBot3 left or right to keep the object centered.
# Robot ONLY rotates in place — no forward/backward movement.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist


class RotateRobotNode(Node):

    def __init__(self):
        super().__init__('rotate_robot')

        self.declare_parameter('image_width',   320)    # must match camera resolution
        self.declare_parameter('dead_band',     0.10)   # fraction of half-width to ignore
        self.declare_parameter('angular_speed', 0.4)    # rad/s
        self.declare_parameter('max_angular',   0.8)    # rad/s cap (proportional mode)
        self.declare_parameter('proportional',  True)   # True=P-control, False=bang-bang

        self._img_w   = float(self.get_parameter('image_width').value)
        self._db      = self.get_parameter('dead_band').value
        self._spd     = self.get_parameter('angular_speed').value
        self._max_spd = self.get_parameter('max_angular').value
        self._prop    = self.get_parameter('proportional').value

        self._cx      = self._img_w / 2.0          # center column in pixels
        self._dead_px = self._db * self._cx         # dead-band width in pixels

        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._coord_sub = self.create_subscription(
            Point, '/object_coord', self._coord_callback, 10)

        self.get_logger().info('rotate_robot node ready.')

    def _coord_callback(self, msg: Point):
        twist = Twist()  # all zeros by default = stop

        # coord.z == -1 means find_object saw no object → stop
        if msg.z < 0:
            self._vel_pub.publish(twist)
            return

        error = msg.x - self._cx   # positive = object is RIGHT of center

        # Within dead-band → already centered, don't spin
        if abs(error) <= self._dead_px:
            self._vel_pub.publish(twist)
            return

        if self._prop:
            # Proportional: bigger error = faster turn
            norm_error = error / self._cx              # range roughly -1 to +1
            omega      = -self._spd * norm_error       # negative = turn right
            omega      = max(-self._max_spd, min(self._max_spd, omega))
        else:
            # Bang-bang: fixed speed, just direction
            omega = -self._spd if error > 0 else self._spd

        twist.angular.z = omega
        self._vel_pub.publish(twist)

        self.get_logger().info(
            f"obj_x={msg.x:.0f}  error={error:+.1f}px  "
            f"turn={'RIGHT' if error > 0 else 'LEFT '}  ω={omega:.3f} rad/s")

    def destroy_node(self):
        self._vel_pub.publish(Twist())   # stop robot on shutdown
        self.get_logger().info('rotate_robot shutting down – robot stopped.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RotateRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
