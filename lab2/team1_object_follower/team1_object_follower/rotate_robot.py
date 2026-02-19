#!/usr/bin/env python3
# Lab 2 - rotate_robot ROS2 Node
# CS/ME/ECE/AE/BME 7785 - Introduction to Robotics Research
# Georgia Institute of Technology
#
# Subscribes to /object_coord (Point from find_object).
# Turns the TurtleBot3 left/right to center the object.
# Robot ONLY rotates – no forward/backward movement.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist


class RotateRobot(Node):

    def __init__(self):
        super().__init__('rotate_robot')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_width',   320)    # pixels, match camera res
        self.declare_parameter('dead_band',     0.10)   # fraction of half-width
        self.declare_parameter('angular_speed', 0.4)    # rad/s base speed
        self.declare_parameter('max_angular',   0.8)    # rad/s cap
        self.declare_parameter('proportional',  True)   # True=P-control, False=bang-bang

        self._img_w     = float(self.get_parameter('image_width').value)
        self._dead_band = self.get_parameter('dead_band').value
        self._ang_spd   = self.get_parameter('angular_speed').value
        self._max_ang   = self.get_parameter('max_angular').value
        self._prop      = self.get_parameter('proportional').value

        self._center_x  = self._img_w / 2.0
        self._dead_px   = self._dead_band * self._center_x  # dead-band in pixels

        # ── Publisher / Subscriber ────────────────────────────────────────────
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._coord_sub = self.create_subscription(
            Point, '/object_coord', self._coord_callback, 10)

        self.get_logger().info(
            f'rotate_robot ready. '
            f'img_w={self._img_w}, dead_band={self._dead_band}, '
            f'speed={self._ang_spd}, proportional={self._prop}')

    def _coord_callback(self, msg: Point):
        twist = Twist()   # defaults all to 0.0

        # z = -1 means object not detected → stop
        if msg.z < 0:
            self._vel_pub.publish(twist)
            return

        error = msg.x - self._center_x   # + = object is RIGHT of center

        # Dead-band: close enough to center → stop turning
        if abs(error) <= self._dead_px:
            self._vel_pub.publish(twist)
            return

        if self._prop:
            # Proportional: scale speed by how far off-center (normalized 0-1)
            norm  = error / self._center_x
            omega = -self._ang_spd * norm
            omega = max(-self._max_ang, min(self._max_ang, omega))
        else:
            # Bang-bang: fixed speed, just pick direction
            omega = -self._ang_spd if error > 0 else self._ang_spd

        twist.angular.z = omega
        self._vel_pub.publish(twist)

        direction = 'RIGHT' if error > 0 else 'LEFT '
        self.get_logger().info(
            f'obj_x={msg.x:.0f}  error={error:+.1f}px  '
            f'turn {direction}  ω={omega:.3f} rad/s')

    def destroy_node(self):
        # Always stop the robot on shutdown
        self._vel_pub.publish(Twist())
        self.get_logger().info('rotate_robot shutting down – robot stopped.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
