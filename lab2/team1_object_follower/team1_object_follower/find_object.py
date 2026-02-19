#!/usr/bin/env python3
# Lab 2 - find_object ROS2 Node
# CS/ME/ECE/AE/BME 7785 - Introduction to Robotics Research
# Georgia Institute of Technology
#
# This node is Lab 1's find_object.py converted into a ROS2 node.
# Instead of reading from a webcam, it subscribes to the TurtleBot camera.
# Instead of printing/displaying, it publishes the object's pixel coordinate.
#
# What stays the same from Lab 1:
#   - HSVRange dataclass
#   - Calibrator class (mouse click to set thresholds)
#   - build_mask()
#   - detect_largest_blob()
#   - ema_smooth()
#
# What changes:
#   - No webcam (cv2.VideoCapture) — images come from /image_raw/compressed
#   - Publishes Detection center as geometry_msgs/Point to /object_coord
#   - Also publishes annotated image to /find_object/compressed for laptop debug

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple, List


# ── Copied directly from Lab 1 find_object.py ────────────────────────────────

@dataclass
class HSVRange:
    lo: np.ndarray  # shape (3,), dtype uint8/int
    hi: np.ndarray  # shape (3,)


@dataclass
class Detection:
    center: Tuple[int, int]
    bbox:   Tuple[int, int, int, int]  # x, y, w, h
    area:   float
    radius: float


class Calibrator:
    """Click on the target object in the frame window to set HSV thresholds."""
    def __init__(self, h_tol=12, s_tol=80, v_tol=80, patch=7):
        self.h_tol = int(h_tol)
        self.s_tol = int(s_tol)
        self.v_tol = int(v_tol)
        self.patch = int(patch)
        self._last_hsv_frame = None
        self.ranges: List[HSVRange] = [
            HSVRange(lo=np.array([35, 60, 60]), hi=np.array([85, 255, 255]))
        ]  # default green-ish; click to override

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


def build_mask(hsv: np.ndarray, hsv_ranges: List[HSVRange]) -> np.ndarray:
    mask = None
    for r in hsv_ranges:
        m = cv2.inRange(hsv, r.lo, r.hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def detect_largest_blob(mask: np.ndarray, min_area: float) -> Optional[Detection]:
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
        if area < min_area:
            continue
        if area > best_area:
            best_area, best = area, c
    if best is None:
        return None
    x, y, w, h       = cv2.boundingRect(best)
    (cx, cy), radius  = cv2.minEnclosingCircle(best)
    M = cv2.moments(best)
    if M["m00"] > 1e-6:
        cx_i = int(M["m10"] / M["m00"])
        cy_i = int(M["m01"] / M["m00"])
    else:
        cx_i, cy_i = int(cx), int(cy)
    return Detection(center=(cx_i, cy_i), bbox=(x, y, w, h),
                     area=best_area, radius=float(radius))


def ema_smooth(prev: Optional[Tuple[float, float]],
               cur:  Tuple[int, int],
               alpha: float) -> Tuple[float, float]:
    if prev is None:
        return float(cur[0]), float(cur[1])
    return (alpha * float(cur[0]) + (1 - alpha) * prev[0],
            alpha * float(cur[1]) + (1 - alpha) * prev[1])


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class FindObjectNode(Node):

    def __init__(self):
        super().__init__('find_object')

        self.declare_parameter('min_area',   900.0)
        self.declare_parameter('alpha',      0.55)   # EMA smoothing, same as Lab 1
        self.declare_parameter('show_debug', True)   # publish annotated image

        self._min_area   = self.get_parameter('min_area').value
        self._alpha      = self.get_parameter('alpha').value
        self._show_debug = self.get_parameter('show_debug').value

        # Same calibrator as Lab 1 — click on the debug window to set HSV
        self._calibrator  = Calibrator()
        self._smoothed_xy = None

        # QoS: depth=1 so we never process stale frames
        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Subscribe to robot camera (same topic as view_image_raw.py)
        self._img_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            img_qos,
        )

        # Publish object pixel coordinate → rotate_robot reads this
        # Point.x = column, Point.y = row, Point.z = area
        # Point.z = -1 when object is NOT found (rotate_robot checks this)
        self._coord_pub = self.create_publisher(Point, '/object_coord', 10)

        # Publish annotated debug image → view on laptop with rqt or view_image_raw
        if self._show_debug:
            self._debug_pub = self.create_publisher(
                CompressedImage, '/find_object/compressed', 10)
            # Open a local display window so you can click to calibrate HSV
            # (only works if you run this node on a machine with a display,
            #  e.g. your laptop during initial testing)
            try:
                cv2.namedWindow('find_object', cv2.WINDOW_NORMAL)
                cv2.setMouseCallback('find_object', self._calibrator.mouse_cb)
                self._local_display = True
            except Exception:
                self._local_display = False  # headless robot, no display
        else:
            self._local_display = False

        self.get_logger().info('find_object node ready.')

    def _image_callback(self, msg: CompressedImage):
        # Decode compressed image — same as view_image_raw.py
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        img_h, img_w = frame.shape[:2]

        # ── Same processing pipeline as Lab 1 ────────────────────────────────
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        self._calibrator.update_hsv_frame(hsv)

        mask = build_mask(hsv, self._calibrator.ranges)
        det  = detect_largest_blob(mask, min_area=self._min_area)
        # ─────────────────────────────────────────────────────────────────────

        # Publish coordinate
        coord = Point()
        if det is not None:
            self._smoothed_xy = ema_smooth(self._smoothed_xy, det.center, self._alpha)
            sx = int(round(self._smoothed_xy[0]))
            sy = int(round(self._smoothed_xy[1]))
            coord.x = float(sx)
            coord.y = float(sy)
            coord.z = float(det.area)
        else:
            self._smoothed_xy = None
            coord.x = -1.0
            coord.y = -1.0
            coord.z = -1.0   # sentinel: no object

        self._coord_pub.publish(coord)

        # Build debug visualisation (same drawings as Lab 1)
        if self._show_debug:
            vis = frame.copy()
            # Center line so you can see left/right offset
            cv2.line(vis, (img_w // 2, 0), (img_w // 2, img_h), (0, 255, 255), 1)

            if det is not None:
                sx = int(round(self._smoothed_xy[0]))
                sy = int(round(self._smoothed_xy[1]))
                x, y, w, h = det.bbox
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(vis, (sx, sy), 6, (0, 0, 255), -1)
                cv2.drawMarker(vis, (sx, sy), (255, 0, 0),
                               markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                cv2.putText(vis, f"Center: ({sx}, {sy})  Area: {det.area:.0f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 3, cv2.LINE_AA)
                cv2.putText(vis, f"Center: ({sx}, {sy})  Area: {det.area:.0f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240, 240, 240), 1, cv2.LINE_AA)
            else:
                cv2.putText(vis, "Object: NOT FOUND",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 3, cv2.LINE_AA)
                cv2.putText(vis, "Object: NOT FOUND",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240, 240, 240), 1, cv2.LINE_AA)

            cv2.putText(vis, self._calibrator.debug_text(),
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
            cv2.putText(vis, "Click to calibrate HSV",
                        (10, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)

            # Publish to ROS topic so laptop can view via rqt
            ok, enc = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                dbg        = CompressedImage()
                dbg.header = msg.header
                dbg.format = 'jpeg'
                dbg.data   = enc.tobytes()
                self._debug_pub.publish(dbg)

            # Also show locally if we have a display (useful during development)
            if self._local_display:
                cv2.imshow('find_object', vis)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FindObjectNode()
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
