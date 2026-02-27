#!/usr/bin/env python3
# Lab 3 - detect_object ROS2 Node
# CS/ME/ECE/AE/BME 7785 - Introduction to Robotics Research
# Georgia Institute of Technology
#
# Identical detection logic to Lab 2 find_object.py.
# Key difference for Lab 3:
#   - Publishes the object's ANGULAR position (radians) instead of raw pixels
#     so get_object_range can correlate it with the LIDAR scan angles.
#
# Camera model (Raspberry Pi Camera v2, 320x240 mode):
#   Horizontal FOV ≈ 62.2°  →  radians per pixel = (62.2° in rad) / 320 px
#
# Published topic: /object_angle  (std_msgs/Float32)
#   value = angle from camera center in radians
#           positive = object is to the RIGHT of center
#           -999.0   = object not found (sentinel)
#
# Also still publishes /detect_object/compressed for debug viewing.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple, List
import math


# ── HSV detection helpers (same as Lab 1 & 2) ────────────────────────────────

@dataclass
class HSVRange:
    lo: np.ndarray
    hi: np.ndarray


@dataclass
class Detection:
    center: Tuple[int, int]
    bbox:   Tuple[int, int, int, int]
    area:   float
    radius: float


class Calibrator:
    def __init__(self, h_tol=12, s_tol=80, v_tol=80, patch=7):
        self.h_tol = int(h_tol)
        self.s_tol = int(s_tol)
        self.v_tol = int(v_tol)
        self.patch = int(patch)
        self._last_hsv_frame = None
        self.ranges: List[HSVRange] = [
            HSVRange(lo=np.array([100, 80, 50]), hi=np.array([130, 255, 255]))
        ]

    def update_hsv_frame(self, hsv_frame):
        self._last_hsv_frame = hsv_frame

    def mouse_cb(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self._last_hsv_frame is None:
            return
        H, W = self._last_hsv_frame.shape[:2]
        r = self.patch // 2
        x0, x1 = max(0, x - r), min(W, x + r + 1)
        y0, y1 = max(0, y - r), min(H, y + r + 1)
        patch    = self._last_hsv_frame[y0:y1, x0:x1, :]
        mean_hsv = patch.reshape(-1, 3).mean(axis=0)
        h, s, v  = mean_hsv
        h_lo = int(round(h)) - self.h_tol
        h_hi = int(round(h)) + self.h_tol
        s_lo = max(0,   int(round(s)) - self.s_tol)
        s_hi = min(255, int(round(s)) + self.s_tol)
        v_lo = max(0,   int(round(v)) - self.v_tol)
        v_hi = min(255, int(round(v)) + self.v_tol)
        ranges = []
        if h_lo < 0:
            ranges.append(HSVRange(
                lo=np.array([0, s_lo, v_lo], dtype=np.uint8),
                hi=np.array([min(179, h_hi), s_hi, v_hi], dtype=np.uint8)))
            ranges.append(HSVRange(
                lo=np.array([179 + h_lo, s_lo, v_lo], dtype=np.uint8),
                hi=np.array([179, s_hi, v_hi], dtype=np.uint8)))
        elif h_hi > 179:
            ranges.append(HSVRange(
                lo=np.array([max(0, h_lo), s_lo, v_lo], dtype=np.uint8),
                hi=np.array([179, s_hi, v_hi], dtype=np.uint8)))
            ranges.append(HSVRange(
                lo=np.array([0, s_lo, v_lo], dtype=np.uint8),
                hi=np.array([h_hi - 179, s_hi, v_hi], dtype=np.uint8)))
        else:
            ranges.append(HSVRange(
                lo=np.array([h_lo, s_lo, v_lo], dtype=np.uint8),
                hi=np.array([h_hi, s_hi, v_hi], dtype=np.uint8)))
        self.ranges = ranges

    def debug_text(self) -> str:
        parts = []
        for i, r in enumerate(self.ranges):
            parts.append(f"R{i}: lo={tuple(int(z) for z in r.lo)} hi={tuple(int(z) for z in r.hi)}")
        return " | ".join(parts)


def build_mask(hsv, hsv_ranges):
    mask = None
    for r in hsv_ranges:
        m = cv2.inRange(hsv, r.lo, r.hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def detect_largest_blob(mask, min_area):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    cnts_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours  = cnts_info[0] if len(cnts_info) == 2 else cnts_info[1]
    if not contours:
        return None
    best, best_area = None, 0.0
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < min_area and area > best_area:
            best_area, best = area, c
        elif area >= min_area and area > best_area:
            best_area, best = area, c
    if best is None or best_area < min_area:
        return None
    x, y, w, h      = cv2.boundingRect(best)
    (cx, cy), radius = cv2.minEnclosingCircle(best)
    M = cv2.moments(best)
    if M["m00"] > 1e-6:
        cx_i = int(M["m10"] / M["m00"])
        cy_i = int(M["m01"] / M["m00"])
    else:
        cx_i, cy_i = int(cx), int(cy)
    return Detection(center=(cx_i, cy_i), bbox=(x, y, w, h),
                     area=best_area, radius=float(radius))


def ema_smooth(prev, cur, alpha):
    if prev is None:
        return float(cur[0]), float(cur[1])
    return (alpha * float(cur[0]) + (1 - alpha) * prev[0],
            alpha * float(cur[1]) + (1 - alpha) * prev[1])


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

# Raspberry Pi Camera v2 horizontal FOV = 62.2 degrees at full res.
# At 320x240 (used on TurtleBot3), the effective FOV is the same horizontally.
HFOV_RAD = math.radians(62.2)   # total horizontal field of view in radians

NOT_FOUND_SENTINEL = -999.0      # published on /object_angle when no object


class DetectObjectNode(Node):

    def __init__(self):
        super().__init__('detect_object')

        self.declare_parameter('min_area',    900.0)
        self.declare_parameter('alpha',       0.55)
        self.declare_parameter('show_debug',  True)
        self.declare_parameter('image_width', 320)

        self._min_area   = self.get_parameter('min_area').value
        self._alpha      = self.get_parameter('alpha').value
        self._show_debug = self.get_parameter('show_debug').value
        self._img_w      = self.get_parameter('image_width').value

        # radians per pixel (from center)
        self._rad_per_px = HFOV_RAD / self._img_w

        self._calibrator  = Calibrator()
        self._smoothed_xy = None

        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed',
            self._image_callback, img_qos)

        # Publish angle in radians to get_object_range
        self._angle_pub = self.create_publisher(Float32, '/object_angle', 10)

        if self._show_debug:
            self._debug_pub = self.create_publisher(
                CompressedImage, '/detect_object/compressed', 10)
            try:
                cv2.namedWindow('detect_object', cv2.WINDOW_NORMAL)
                cv2.setMouseCallback('detect_object', self._calibrator.mouse_cb)
                self._local_display = True
            except Exception:
                self._local_display = False
        else:
            self._local_display = False

        self.get_logger().info(
            f'detect_object ready. HFOV={math.degrees(HFOV_RAD):.1f}deg, '
            f'rad/px={self._rad_per_px:.5f}')

    def _image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        img_h, img_w = frame.shape[:2]
        cx_center    = img_w / 2.0

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        self._calibrator.update_hsv_frame(hsv)

        mask = build_mask(hsv, self._calibrator.ranges)
        det  = detect_largest_blob(mask, self._min_area)

        angle_msg = Float32()

        if det is not None:
            self._smoothed_xy = ema_smooth(self._smoothed_xy, det.center, self._alpha)
            sx = self._smoothed_xy[0]

            # pixel offset from center → angle
            # positive offset (right of center) → positive angle
            pixel_offset  = sx - cx_center
            angle_rad     = pixel_offset * self._rad_per_px
            angle_msg.data = float(angle_rad)
        else:
            self._smoothed_xy = None
            angle_msg.data    = NOT_FOUND_SENTINEL

        self._angle_pub.publish(angle_msg)

        # Debug visualisation
        if self._show_debug:
            vis = frame.copy()
            cv2.line(vis, (img_w // 2, 0), (img_w // 2, img_h), (0, 255, 255), 1)

            if det is not None:
                sx_i = int(round(self._smoothed_xy[0]))
                sy_i = int(round(self._smoothed_xy[1]))
                x, y, w, h = det.bbox
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(vis, (sx_i, sy_i), 6, (0, 0, 255), -1)
                cv2.drawMarker(vis, (sx_i, sy_i), (255, 0, 0),
                               cv2.MARKER_CROSS, 20, 2)
                angle_deg = math.degrees(angle_msg.data)
                cv2.putText(vis, f"Angle: {angle_deg:.1f} deg  Area: {det.area:.0f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 3, cv2.LINE_AA)
                cv2.putText(vis, f"Angle: {angle_deg:.1f} deg  Area: {det.area:.0f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 240, 240), 1, cv2.LINE_AA)
            else:
                cv2.putText(vis, "Object: NOT FOUND",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 3, cv2.LINE_AA)
                cv2.putText(vis, "Object: NOT FOUND",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1, cv2.LINE_AA)

            cv2.putText(vis, self._calibrator.debug_text(),
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)

            ok, enc = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                dbg        = CompressedImage()
                dbg.header = msg.header
                dbg.format = 'jpeg'
                dbg.data   = enc.tobytes()
                self._debug_pub.publish(dbg)

            if self._local_display:
                cv2.imshow('detect_object', vis)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
