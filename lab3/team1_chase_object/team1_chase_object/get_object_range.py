#!/usr/bin/env python3
# Lab 3 - get_object_range ROS2 Node
# CS/ME/ECE/AE/BME 7785 - Introduction to Robotics Research
# Georgia Institute of Technology
#
# Fuses camera angle (from detect_object) with LIDAR scan (from /scan)
# to produce the object's:
#   - angular position relative to the robot (radians)
#   - distance from the robot (meters)
#
# Reference frame alignment:
#   Camera: 0 = straight ahead, + = right, - = left  (our convention)
#   LIDAR:  angle_min is the start angle, increments by angle_increment
#           Index 0 = directly ahead on the TurtleBot3 Burger
#           Angles increase counter-clockwise (ROS standard)
#           So LIDAR angle = 0 is straight ahead, positive = left, negative = right
#
#   To convert camera angle → LIDAR index:
#     lidar_angle = -camera_angle   (flip sign: camera+ is right, LIDAR+ is left)
#     index = round((lidar_angle - angle_min) / angle_increment)
#
# Published topic: /object_range  (geometry_msgs/Point)
#   Point.x = angular error (radians, positive = object is RIGHT of robot heading)
#   Point.y = distance to object (meters)
#   Point.z = -1.0 if object not found, 0.0 if valid

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import math

NOT_FOUND = -999.0   # sentinel from detect_object


class GetObjectRangeNode(Node):

    def __init__(self):
        super().__init__('get_object_range')

        # How many LIDAR beams on each side of the camera angle to average
        # Averaging reduces noise from a single noisy beam
        self.declare_parameter('lidar_window', 3)

        self._window = self.get_parameter('lidar_window').value

        # Store latest data from each sensor
        self._camera_angle: float = NOT_FOUND   # radians, from detect_object
        self._scan: LaserScan     = None

        # QoS for LIDAR (BEST_EFFORT like camera)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Subscribe to camera angle from detect_object
        self._angle_sub = self.create_subscription(
            Float32, '/object_angle',
            self._angle_callback, 10)

        # Subscribe to LIDAR scan
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self._scan_callback, sensor_qos)

        # Publish fused result to chase_object
        self._range_pub = self.create_publisher(Point, '/object_range', 10)

        # Process at 10 Hz using a timer rather than being purely callback-driven
        self._timer = self.create_timer(0.1, self._fuse_and_publish)

        self.get_logger().info('get_object_range node ready.')

    # ── Callbacks just store latest data ─────────────────────────────────────

    def _angle_callback(self, msg: Float32):
        self._camera_angle = msg.data

    def _scan_callback(self, msg: LaserScan):
        self._scan = msg

    # ── Main fusion logic ─────────────────────────────────────────────────────

    def _fuse_and_publish(self):
        result = Point()

        # If either sensor has no data yet, or object not found → publish not-found
        if self._scan is None or self._camera_angle == NOT_FOUND:
            result.x = 0.0
            result.y = 0.0
            result.z = -1.0   # sentinel: no valid data
            self._range_pub.publish(result)
            return

        scan          = self._scan
        camera_angle  = self._camera_angle   # radians, + = right of camera center

        # ── Convert camera angle to LIDAR index ──────────────────────────────
        # LIDAR angle convention: 0 = forward, increases counter-clockwise
        # Camera convention:      0 = forward, positive = right
        # So: lidar_angle = -camera_angle
        lidar_angle = -camera_angle

        # Clamp to valid LIDAR range
        lidar_angle = max(scan.angle_min,
                          min(scan.angle_max, lidar_angle))

        # Compute the center index
        center_idx = int(round(
            (lidar_angle - scan.angle_min) / scan.angle_increment
        ))
        center_idx = max(0, min(len(scan.ranges) - 1, center_idx))

        # ── Average a window of beams around the center index ─────────────────
        # This smooths out any single-beam noise or spurious inf/nan readings
        w = self._window
        idx_lo = max(0, center_idx - w)
        idx_hi = min(len(scan.ranges) - 1, center_idx + w)

        beams = []
        for i in range(idx_lo, idx_hi + 1):
            r = scan.ranges[i]
            # Filter out invalid readings (inf, nan, out of range)
            if (math.isfinite(r) and
                    scan.range_min <= r <= scan.range_max):
                beams.append(r)

        if not beams:
            # No valid LIDAR reading at this angle → publish not-found
            result.x = float(camera_angle)
            result.y = 0.0
            result.z = -1.0
            self._range_pub.publish(result)
            return

        distance = float(np.median(beams))   # median is more robust than mean

        # ── Publish result ────────────────────────────────────────────────────
        result.x = float(camera_angle)   # angular error (rad), + = object right
        result.y = distance              # distance in meters
        result.z = 0.0                   # 0 = valid reading

        self._range_pub.publish(result)

        self.get_logger().debug(
            f'angle={math.degrees(camera_angle):.1f}deg  '
            f'distance={distance:.3f}m  beams_used={len(beams)}')


def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
