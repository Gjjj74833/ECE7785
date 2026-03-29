#!/usr/bin/env python3
# Lab 4 - go_to_goal ROS2 Node
# Yihan Liu
#
# Reads waypoints from wayPoints.txt
# Uses corrected odometry (RotationScript logic) for global position
# State machine:
#   GO_TO_GOAL      → P controllers for heading and distance
#   AVOID_OBSTACLE  → turn away from obstacle, then resume
#   STOP_AT_GOAL    → stop 10 seconds at each waypoint
#
# Waypoints (from wayPoints.txt):
#   (1.5, 0.0)  → stop within 10 cm
#   (1.5, 1.4)  → stop within 15 cm
#   (0.0, 1.4)  → stop within 20 cm
#
# Subscribes:
#   /odom              nav_msgs/Odometry
#   /obstacle_vector   geometry_msgs/Point
#
# Publishes:
#   /cmd_vel           geometry_msgs/Twist

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import numpy as np
import math
import os
import time


# ── Waypoints and tolerances ──────────────────────────────────────────────────
WAYPOINTS = [
    (1.5, 0.0,  0.10),   # (x, y, tolerance_m)
    (1.5, 1.4,  0.15),
    (0.0, 1.4,  0.20),
]

STOP_DURATION = 10.0   # seconds to stop at each waypoint

# ── Speed limits ──────────────────────────────────────────────────────────────
MAX_LINEAR  = 0.15    # m/s
MAX_ANGULAR = 1.5     # rad/s

# ── Go-to-goal P gains ────────────────────────────────────────────────────────
KP_LINEAR   = 0.4
KP_ANGULAR  = 1.8

# ── Obstacle avoidance ────────────────────────────────────────────────────────
OBSTACLE_STOP_DIST  = 0.40   # m: start avoiding when obstacle is within this range
OBSTACLE_CLEAR_DIST = 0.60   # m: resume go-to-goal when obstacle is beyond this
AVOID_ANGULAR_SPEED = 0.8    # rad/s: fixed turn speed during avoidance

# ── Dead-bands ────────────────────────────────────────────────────────────────
ANGLE_DEAD_BAND = math.radians(3.0)   # stop rotating when within 3 deg
DIST_DEAD_BAND  = 0.02                # stop linear when within 2 cm


# ── State machine states ──────────────────────────────────────────────────────
STATE_GO_TO_GOAL     = 'GO_TO_GOAL'
STATE_AVOID_OBSTACLE = 'AVOID_OBSTACLE'
STATE_STOP_AT_GOAL   = 'STOP_AT_GOAL'
STATE_DONE           = 'DONE'


class GoToGoalNode(Node):

    def __init__(self):
        super().__init__('go_to_goal')

        # ── Odometry correction state (from RotationScript.py) ───────────────
        self.Init       = True
        self.Init_pos   = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang   = 0.0
        self.globalPos  = Point()
        self.globalAng  = 0.0

        # ── Navigation state ─────────────────────────────────────────────────
        self._waypoints      = WAYPOINTS
        self._wp_index       = 0
        self._state          = STATE_GO_TO_GOAL
        self._stop_start_time = None

        # ── Obstacle data ─────────────────────────────────────────────────────
        self._obstacle_dist  = float('inf')
        self._obstacle_angle = 0.0
        self._obstacle_valid = False

        # ── Publishers / Subscribers ─────────────────────────────────────────
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 1)

        self._obstacle_sub = self.create_subscription(
            Point, '/obstacle_vector', self._obstacle_callback, 10)

        # Control loop at 10 Hz
        self._timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('go_to_goal node ready. Waypoints:')
        for i, (x, y, tol) in enumerate(self._waypoints):
            self.get_logger().info(f'  WP{i+1}: ({x}, {y})  tol={tol*100:.0f}cm')

    # ── Odometry callback (uses RotationScript logic) ─────────────────────────

    def _odom_callback(self, Odom):
        position = Odom.pose.pose.position
        q        = Odom.pose.pose.orientation
        orientation = np.arctan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        if self.Init:
            self.Init     = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.array([
                [ np.cos(self.Init_ang), np.sin(self.Init_ang)],
                [-np.sin(self.Init_ang), np.cos(self.Init_ang)]
            ])
            self.Init_pos.x = Mrot[0, 0] * position.x + Mrot[0, 1] * position.y
            self.Init_pos.y = Mrot[1, 0] * position.x + Mrot[1, 1] * position.y

        Mrot = np.array([
            [ np.cos(self.Init_ang), np.sin(self.Init_ang)],
            [-np.sin(self.Init_ang), np.cos(self.Init_ang)]
        ])

        self.globalPos.x = Mrot[0, 0] * position.x + Mrot[0, 1] * position.y - self.Init_pos.x
        self.globalPos.y = Mrot[1, 0] * position.x + Mrot[1, 1] * position.y - self.Init_pos.y
        self.globalAng   = orientation - self.Init_ang

    # ── Obstacle callback ──────────────────────────────────────────────────────

    def _obstacle_callback(self, msg: Point):
        if msg.z < 0:
            self._obstacle_valid = False
            self._obstacle_dist  = float('inf')
        else:
            self._obstacle_valid = True
            self._obstacle_dist  = msg.x
            self._obstacle_angle = msg.y

    # ── Main control loop ──────────────────────────────────────────────────────

    def _control_loop(self):
        twist = Twist()

        if self._state == STATE_DONE:
            self._vel_pub.publish(twist)
            return

        # ── Check if we need to switch to obstacle avoidance ──────────────────
        if (self._state == STATE_GO_TO_GOAL and
                self._obstacle_valid and
                self._obstacle_dist < OBSTACLE_STOP_DIST):
            self._state = STATE_AVOID_OBSTACLE
            self.get_logger().warn(
                f'OBSTACLE at {self._obstacle_dist:.2f}m — switching to avoidance')

        # ── State: STOP_AT_GOAL ───────────────────────────────────────────────
        if self._state == STATE_STOP_AT_GOAL:
            now = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now - self._stop_start_time
            self.get_logger().info(
                f'Stopped at WP{self._wp_index} — {elapsed:.1f}/{STOP_DURATION:.0f}s')

            if elapsed >= STOP_DURATION:
                self._wp_index += 1
                if self._wp_index >= len(self._waypoints):
                    self._state = STATE_DONE
                    self.get_logger().info('All waypoints reached! DONE.')
                else:
                    self._state = STATE_GO_TO_GOAL
                    self.get_logger().info(
                        f'Moving to WP{self._wp_index + 1}: '
                        f'({self._waypoints[self._wp_index][0]}, '
                        f'{self._waypoints[self._wp_index][1]})')
            self._vel_pub.publish(twist)
            return

        # ── State: AVOID_OBSTACLE ─────────────────────────────────────────────
        if self._state == STATE_AVOID_OBSTACLE:
            if (not self._obstacle_valid or
                    self._obstacle_dist > OBSTACLE_CLEAR_DIST):
                self._state = STATE_GO_TO_GOAL
                self.get_logger().info('Obstacle cleared — resuming go-to-goal')
            else:
                # Turn away from the obstacle
                # obstacle_angle: + = obstacle on left, - = obstacle on right
                # Turn right if obstacle is on left (positive angle), left if on right
                if self._obstacle_angle >= 0:
                    twist.angular.z = -AVOID_ANGULAR_SPEED  # turn right
                else:
                    twist.angular.z = AVOID_ANGULAR_SPEED   # turn left
                # Slow forward creep while turning to get around obstacle
                twist.linear.x = 0.05
            self._vel_pub.publish(twist)
            return

        # ── State: GO_TO_GOAL ─────────────────────────────────────────────────
        if self._wp_index >= len(self._waypoints):
            self._state = STATE_DONE
            self._vel_pub.publish(twist)
            return

        goal_x, goal_y, tolerance = self._waypoints[self._wp_index]

        # Current global pose
        cur_x   = self.globalPos.x
        cur_y   = self.globalPos.y
        cur_ang = self.globalAng

        # Distance and heading to goal
        dx   = goal_x - cur_x
        dy   = goal_y - cur_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Desired heading in global frame
        desired_heading = math.atan2(dy, dx)

        # Heading error (wrapped to [-pi, pi])
        heading_error = desired_heading - cur_ang
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        self.get_logger().info(
            f'[{self._state}] WP{self._wp_index+1}({goal_x},{goal_y}) '
            f'pos=({cur_x:.2f},{cur_y:.2f}) '
            f'dist={dist:.3f}m  hdg_err={math.degrees(heading_error):+.1f}deg')

        # ── Check if we reached the waypoint ──────────────────────────────────
        if dist <= tolerance:
            self._state           = STATE_STOP_AT_GOAL
            self._stop_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'Reached WP{self._wp_index+1} within {dist*100:.1f}cm — stopping 10s')
            self._vel_pub.publish(twist)
            return

        # ── P controllers ─────────────────────────────────────────────────────
        # Angular: always correct heading first
        angular_z = KP_ANGULAR * heading_error
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_z))

        # Linear: only drive forward when roughly facing the goal
        if abs(heading_error) < math.radians(20):
            linear_x = KP_LINEAR * dist
            linear_x = max(0.0, min(MAX_LINEAR, linear_x))
        else:
            linear_x = 0.0

        # Apply dead-bands
        if abs(heading_error) < ANGLE_DEAD_BAND:
            angular_z = 0.0
        if dist < DIST_DEAD_BAND:
            linear_x = 0.0

        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        self._vel_pub.publish(twist)

    def destroy_node(self):
        self._vel_pub.publish(Twist())
        self.get_logger().info('go_to_goal shutting down — robot stopped.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
