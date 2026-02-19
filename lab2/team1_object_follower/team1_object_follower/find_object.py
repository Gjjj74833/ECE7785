#!/usr/bin/env python3
# Lab 2 - find_object ROS2 Node
# CS/ME/ECE/AE/BME 7785 - Introduction to Robotics Research
# Georgia Institute of Technology
#
# Subscribes to /image_raw/compressed, detects object via HSV thresholding,
# publishes pixel coordinate to /object_coord (geometry_msgs/Point).
# Also publishes an annotated debug image to /find_object/compressed.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, List, Tuple


# ── HSV helpers (from Lab 1) ──────────────────────────────────────────────────

@dataclass
class HSVRange:
    lo: np.ndarray   # dtype uint8, shape (3,)
    hi: np.ndarray


def build_mask(hsv: np.ndarray, hsv_ranges: List[HSVRange]) -> np.ndarray:
    mask = None
    for r in hsv_ranges:
        m = cv2.inRange(hsv, r.lo, r.hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def detect_largest_blob(mask: np.ndarray,
                        min_area: float = 900.0) -> Optional[Tuple[int, int, float]]:
    """Returns (cx, cy, area) of the largest blob, or None."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnts_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = cnts_info[0] if len(cnts_info) == 2 else cnts_info[1]

    best, best_area = None, 0.0
    for c in contours:
        area = float(cv2.contourArea(c))
        if area >= min_area and area > best_area:
            best, best_area = c, area

    if best is None:
        return None

    M = cv2.moments(best)
    if M["m00"] > 1e-6:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        (cx, cy), _ = cv2.minEnclosingCircle(best)
        cx, cy = int(cx), int(cy)

    return cx, cy, best_area


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class FindObject(Node):

    # ┌─────────────────────────────────────────────────────────────────┐
    # │  CALIBRATE THIS for your object!                                │
    # │  Run the standalone find_object.py from Lab 1 on your laptop,  │
    # │  click on your object, note the HSV lo/hi values, paste below. │
    # │                                                                 │
    # │  Green (default):  lo=[35,60,60]   hi=[85,255,255]             │
    # │  Blue example:     lo=[100,80,50]  hi=[130,255,255]            │
    # │  Red example:      lo=[0,120,70]   hi=[10,255,255]  +          │
    # │                    lo=[170,120,70] hi=[179,255,255]            │
    # └─────────────────────────────────────────────────────────────────┘
    DEFAULT_HSV_RANGES = [
        HSVRange(lo=np.array([35,  60,  60], dtype=np.uint8),
                 hi=np.array([85, 255, 255], dtype=np.uint8)),
    ]

    def __init__(self):
        super().__init__('find_object')

        self.declare_parameter('min_area',   900.0)
        self.declare_parameter('show_debug', True)

        self._min_area   = self.get_parameter('min_area').value
        self._show_debug = self.get_parameter('show_debug').value
        self._hsv_ranges = self.DEFAULT_HSV_RANGES

        # QoS: depth=1 so we always act on the LATEST frame, never stale ones
        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed',
            self._image_callback, img_qos)

        # Point.x = col, Point.y = row, Point.z = area  (-1 = not found)
        self._coord_pub = self.create_publisher(Point, '/object_coord', 10)

        if self._show_debug:
            self._debug_pub = self.create_publisher(
                CompressedImage, '/find_object/compressed', 10)

        self.get_logger().info('find_object node ready.')

    def _image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        img_h, img_w = frame.shape[:2]

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask    = build_mask(hsv, self._hsv_ranges)
        result  = detect_largest_blob(mask, self._min_area)

        coord_msg = Point()
        if result is not None:
            cx, cy, area = result
            coord_msg.x = float(cx)
            coord_msg.y = float(cy)
            coord_msg.z = float(area)
        else:
            coord_msg.x = -1.0
            coord_msg.y = -1.0
            coord_msg.z = -1.0

        self._coord_pub.publish(coord_msg)

        if self._show_debug:
            vis = frame.copy()
            cv2.line(vis, (img_w // 2, 0), (img_w // 2, img_h), (0, 255, 255), 1)
            if result is not None:
                cx, cy, area = result
                cv2.circle(vis, (cx, cy), 8, (0, 0, 255), -1)
                cv2.drawMarker(vis, (cx, cy), (255, 0, 0),
                               cv2.MARKER_CROSS, 24, 2)
                cv2.putText(vis, f'({cx},{cy}) area={area:.0f}',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (20, 20, 20), 3)
                cv2.putText(vis, f'({cx},{cy}) area={area:.0f}',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (240, 240, 240), 1)
            else:
                cv2.putText(vis, 'NOT FOUND', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            ok, enc = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                dbg          = CompressedImage()
                dbg.header   = msg.header
                dbg.format   = 'jpeg'
                dbg.data     = enc.tobytes()
                self._debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = FindObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
