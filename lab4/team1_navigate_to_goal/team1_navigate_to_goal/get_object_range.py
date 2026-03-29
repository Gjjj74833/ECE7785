#!/usr/bin/env python3

# Yihan Liu
#
# Subscribes to /scan (LaserScan)
# Filters LIDAR to detect the nearest obstacle (not walls, not floor noise)
# Publishes the vector from robot to nearest obstacle as geometry_msgs/Point
#   .x = distance to obstacle (meters)
#   .y = angle to obstacle (radians, ROS convention: + = left/CCW)
#   .z = -1 if no obstacle detected, 0 if valid
#
# Filtering strategy:
#   1. Only look at beams within MAX_OBSTACLE_DIST (ignore far walls)
#   2. Cluster consecutive close beams into segments (one object = one cluster)
#   3. Return the closest cluster centroid
#   4. Ignore the known blue box region (optional, conservative)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import math


# Only consider obstacles within this range (meters)
# Anything beyond is a wall
MAX_OBSTACLE_DIST = 1.0

# Minimum number of consecutive beams to count as a real obstacle
# (filters out single-beam noise)
MIN_CLUSTER_BEAMS = 3

# Angular gap (radians) that splits two separate obstacles
CLUSTER_GAP_RAD = math.radians(10.0)


class GetObjectRangeNode(Node):

    def __init__(self):
        super().__init__('get_object_range')

        self.declare_parameter('max_obstacle_dist', MAX_OBSTACLE_DIST)
        self.declare_parameter('min_cluster_beams', MIN_CLUSTER_BEAMS)

        self._max_dist     = self.get_parameter('max_obstacle_dist').value
        self._min_beams    = self.get_parameter('min_cluster_beams').value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, sensor_qos)

        self._obstacle_pub = self.create_publisher(Point, '/obstacle_vector', 10)

        self.get_logger().info('get_object_range node ready.')

    def _scan_callback(self, msg: LaserScan):
        ranges    = np.array(msg.ranges)
        n         = len(ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # Build angle array for each beam
        angles = angle_min + np.arange(n) * angle_inc

        # ── Step 1: mask valid beams within obstacle distance ─────────────────
        valid = (
            np.isfinite(ranges) &
            (ranges >= msg.range_min) &
            (ranges <= msg.range_max) &
            (ranges <= self._max_dist)
        )

        result = Point()

        if not np.any(valid):
            result.z = -1.0
            self._obstacle_pub.publish(result)
            return

        # ── Step 2: cluster consecutive valid beams ───────────────────────────
        valid_indices = np.where(valid)[0]

        clusters = []
        current  = [valid_indices[0]]

        for i in range(1, len(valid_indices)):
            prev_idx = valid_indices[i - 1]
            curr_idx = valid_indices[i]
            angle_gap = abs(angles[curr_idx] - angles[prev_idx])
            if angle_gap < CLUSTER_GAP_RAD:
                current.append(curr_idx)
            else:
                clusters.append(current)
                current = [curr_idx]
        clusters.append(current)

        # ── Step 3: filter clusters by minimum beam count ─────────────────────
        clusters = [c for c in clusters if len(c) >= self._min_beams]

        if not clusters:
            result.z = -1.0
            self._obstacle_pub.publish(result)
            return

        # ── Step 4: find the closest cluster ──────────────────────────────────
        best_dist  = float('inf')
        best_angle = 0.0

        for cluster in clusters:
            cluster_ranges = ranges[cluster]
            cluster_angles = angles[cluster]
            # Use minimum range in cluster as distance (closest point on object)
            min_idx   = np.argmin(cluster_ranges)
            dist      = float(cluster_ranges[min_idx])
            angle     = float(cluster_angles[min_idx])

            if dist < best_dist:
                best_dist  = dist
                best_angle = angle

        result.x = best_dist
        result.y = best_angle
        result.z = 0.0
        self._obstacle_pub.publish(result)

        self.get_logger().debug(
            f'Obstacle: dist={best_dist:.3f}m  angle={math.degrees(best_angle):.1f}deg')


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
