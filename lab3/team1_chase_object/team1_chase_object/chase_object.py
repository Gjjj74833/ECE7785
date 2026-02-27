#!/usr/bin/env python3
# Lab 3 - chase_object ROS2 Node
# CS/ME/ECE/AE/BME 7785 - Introduction to Robotics Research
# Georgia Institute of Technology
#
# Two independent PD controllers (answering Q2: why PD not full PID):
#
#   Angular controller:
#     error = object angle (radians from camera center)
#     output → twist.angular.z
#     Goal: drive angular error to 0 (face the object)
#
#   Linear controller:
#     error = (current distance - desired distance) in meters
#     output → twist.linear.x
#     Goal: maintain desired distance from object
#
# Why PD and not full PID?
#   A P-only controller is sufficient to reject friction disturbances on the
#   TurtleBot3 because the onboard motor controllers already include their own
#   PID loops. Adding I at the high level risks integral windup during turns.
#   D provides damping to prevent oscillation as the robot approaches.
#
# Subscribes: /object_range  (geometry_msgs/Point)
#   .x = angular error (rad)
#   .y = distance (m)
#   .z = -1 if invalid
#
# Publishes:  /cmd_vel  (geometry_msgs/Twist)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math


class PIDController:
    """Simple PID with anti-windup clamp on the integral term."""

    def __init__(self, kp, ki, kd, output_limit, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit if integral_limit else output_limit

        self._integral   = 0.0
        self._prev_error = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = None

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral  = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # Derivative (on error, not measurement — avoids derivative kick)
        if self._prev_error is None:
            d = 0.0
        else:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))


class ChaseObjectNode(Node):

    def __init__(self):
        super().__init__('chase_object')

        # ── Tunable parameters ────────────────────────────────────────────────
        # Desired standoff distance from object (meters)
        self.declare_parameter('desired_distance', 0.5)

        # Angular PID gains
        # P: main turning force. D: damps oscillation.
        self.declare_parameter('ang_kp', 1.5)
        self.declare_parameter('ang_ki', 0.0)   # keep at 0 to avoid windup
        self.declare_parameter('ang_kd', 0.05)

        # Linear PID gains
        self.declare_parameter('lin_kp', 0.5)
        self.declare_parameter('lin_ki', 0.0)
        self.declare_parameter('lin_kd', 0.1)

        # Speed limits
        self.declare_parameter('max_angular', 1.0)   # rad/s
        self.declare_parameter('max_linear',  0.15)  # m/s  (safe for indoors)

        # Dead-bands: within these tolerances we consider the error resolved
        self.declare_parameter('angle_dead_band',    0.05)  # radians (~3 deg)
        self.declare_parameter('distance_dead_band', 0.05)  # meters (5 cm)

        # Read parameters
        self._desired_dist    = self.get_parameter('desired_distance').value
        self._max_ang         = self.get_parameter('max_angular').value
        self._max_lin         = self.get_parameter('max_linear').value
        self._ang_dead        = self.get_parameter('angle_dead_band').value
        self._lin_dead        = self.get_parameter('distance_dead_band').value

        # Build controllers
        self._ang_pid = PIDController(
            kp=self.get_parameter('ang_kp').value,
            ki=self.get_parameter('ang_ki').value,
            kd=self.get_parameter('ang_kd').value,
            output_limit=self._max_ang,
        )
        self._lin_pid = PIDController(
            kp=self.get_parameter('lin_kp').value,
            ki=self.get_parameter('lin_ki').value,
            kd=self.get_parameter('lin_kd').value,
            output_limit=self._max_lin,
        )

        self._last_time = self.get_clock().now()

        # ── Publisher / Subscriber ────────────────────────────────────────────
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._range_sub = self.create_subscription(
            Point, '/object_range', self._range_callback, 10)

        self.get_logger().info(
            f'chase_object ready. desired_dist={self._desired_dist}m')

    def _range_callback(self, msg: Point):
        twist = Twist()   # default all zero

        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        # z = -1 means no valid detection → stop and reset integrators
        if msg.z < 0:
            self._ang_pid.reset()
            self._lin_pid.reset()
            self._vel_pub.publish(twist)
            return

        ang_error = msg.x                          # radians, + = object is RIGHT
        distance  = msg.y                          # meters
        lin_error = distance - self._desired_dist  # + = too far, - = too close

        # ── Angular control ───────────────────────────────────────────────────
        if abs(ang_error) > self._ang_dead:
            # Negate: positive ang_error (object right) → turn right → negative ω
            angular_z = -self._ang_pid.compute(ang_error, dt)
        else:
            angular_z = 0.0
            self._ang_pid.reset()

        # ── Linear control ────────────────────────────────────────────────────
        # Only drive forward/back when roughly facing the object
        # (avoids driving sideways while still rotating)
        if abs(ang_error) < math.radians(15) and abs(lin_error) > self._lin_dead:
            linear_x = self._lin_pid.compute(lin_error, dt)
        else:
            linear_x = 0.0
            if abs(ang_error) >= math.radians(15):
                self._lin_pid.reset()

        twist.angular.z = float(angular_z)
        twist.linear.x  = float(linear_x)
        self._vel_pub.publish(twist)

        self.get_logger().info(
            f'ang_err={math.degrees(ang_error):+.1f}deg  '
            f'dist={distance:.3f}m  lin_err={lin_error:+.3f}m  '
            f'ω={angular_z:+.3f}  vx={linear_x:+.3f}')

    def destroy_node(self):
        self._vel_pub.publish(Twist())   # stop robot on shutdown
        self.get_logger().info('chase_object shutting down – robot stopped.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
