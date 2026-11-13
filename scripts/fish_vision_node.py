#!/usr/bin/python3
"""
Computer Vision Fish Activity & Depleted Feed Evaluation Node
=============================================================
Subscribes to:
  - /camera/image_raw (sensor_msgs/Image)
  - /esp32/sensors_filtered (std_msgs/String JSON, for HUD water status overlay)
  - /brain/state (std_msgs/String, for HUD FSM state overlay)

Performs real-time OpenCV analysis on the downward/forward camera stream:
  1. HSV Color & Contour Segmentation of active fish schools (orange/gold fish bodies).
  2. HSV Segmentation of feeding zone floor pads (depleted cyan pad vs. normal vs. anoxic murky zone).
  3. Frame-to-frame motion / surface agitation analysis (cv2.absdiff).
  4. Computes "Fish Feeding Activity / Depleted Food Score" (0.0% to 100.0%).

Publishes:
  - /vision/feeding_demand (std_msgs/Float32)
  - /vision/debug_image (sensor_msgs/Image) with bounding boxes, activity gauge, and water status HUD.
"""

import json
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class FishVisionNode(Node):
    """OpenCV node evaluating fish aggregation, motion agitation, and feed depletion."""

    def __init__(self) -> None:
        super().__init__("fish_vision_node")

        self.bridge = CvBridge()
        self.prev_gray: Optional[np.ndarray] = None
        self.smoothed_demand: float = 0.0
        self.ema_alpha: float = 0.35

        # Latest water telemetry & robot state for HUD overlay
        self.water_telemetry = {
            "weight": 5000.0,
            "ph": 7.50,
            "do": 6.50,
            "turbidity": 12.0,
            "temp": 25.0,
            "valve": 0,
        }
        self.fsm_state: str = "PATROL"

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.sensor_sub = self.create_subscription(
            String, "/esp32/sensors_filtered", self.sensor_callback, 10
        )
        self.state_sub = self.create_subscription(
            String, "/brain/state", self.state_callback, 10
        )

        # Publishers
        self.demand_pub = self.create_publisher(Float32, "/vision/feeding_demand", 10)
        self.debug_img_pub = self.create_publisher(Image, "/vision/debug_image", 10)

        self.get_logger().info(
            "Fish Vision Node started. Subscribed to /camera/image_raw, "
            "publishing /vision/feeding_demand and /vision/debug_image."
        )

    def sensor_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.water_telemetry.update(data)
        except (json.JSONDecodeError, TypeError):
            pass

    def state_callback(self, msg: String) -> None:
        self.fsm_state = msg.data

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        height, width = frame.shape[:2]
        total_pixels = float(height * width)
        debug_frame = frame.copy()

        # Convert to HSV and Grayscale
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # ---------------------------------------------------------------------
        # 1. Detect Active Fish Contours (Orange / Gold / Reddish-Amber HSV)
        # ---------------------------------------------------------------------
        lower_fish1 = np.array([4, 100, 80], dtype=np.uint8)
        upper_fish1 = np.array([28, 255, 255], dtype=np.uint8)
        fish_mask = cv2.inRange(hsv, lower_fish1, upper_fish1)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fish_mask = cv2.morphologyEx(fish_mask, cv2.MORPH_OPEN, kernel)
        fish_mask = cv2.dilate(fish_mask, kernel, iterations=1)

        contours, _ = cv2.findContours(
            fish_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        fish_count = 0
        fish_pixel_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= 45.0:
                fish_count += 1
                fish_pixel_area += area
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(
                    debug_frame, (x, y), (x + w, y + h), (0, 215, 255), 2
                )
                cv2.putText(
                    debug_frame,
                    f"Fish #{fish_count}",
                    (x, max(15, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        # ---------------------------------------------------------------------
        # 2. Detect Feeding Zone Pad & Depleted Feed Signature (Cyan/Teal/Purple)
        # ---------------------------------------------------------------------
        # Depleted feed tray: High-visibility cyan/blue floor disc
        lower_depleted_tray = np.array([85, 95, 90], dtype=np.uint8)
        upper_depleted_tray = np.array([108, 255, 255], dtype=np.uint8)
        depleted_mask = cv2.inRange(hsv, lower_depleted_tray, upper_depleted_tray)

        # Normal balanced tray: Greenish-teal floor disc
        lower_normal_tray = np.array([65, 80, 70], dtype=np.uint8)
        upper_normal_tray = np.array([84, 255, 255], dtype=np.uint8)
        normal_mask = cv2.inRange(hsv, lower_normal_tray, upper_normal_tray)

        # Poor water quality / anoxic purple-brown zone
        lower_anoxic = np.array([128, 50, 40], dtype=np.uint8)
        upper_anoxic = np.array([170, 255, 220], dtype=np.uint8)
        anoxic_mask = cv2.inRange(hsv, lower_anoxic, upper_anoxic)

        depleted_ratio = float(np.count_nonzero(depleted_mask)) / total_pixels
        normal_ratio = float(np.count_nonzero(normal_mask)) / total_pixels
        anoxic_ratio = float(np.count_nonzero(anoxic_mask)) / total_pixels

        # Draw bounding box around largest detected feeding zone pad
        combined_zone_mask = cv2.bitwise_or(
            depleted_mask, cv2.bitwise_or(normal_mask, anoxic_mask)
        )
        zone_contours, _ = cv2.findContours(
            combined_zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        zone_label = "OPEN WATER"
        zone_box_color = (200, 200, 200)
        if zone_contours:
            largest_zone = max(zone_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_zone) > 600.0:
                zx, zy, zw, zh = cv2.boundingRect(largest_zone)
                if anoxic_ratio > 0.015:
                    zone_label = "ANOXIC / POOR WATER ZONE"
                    zone_box_color = (0, 0, 255)
                elif depleted_ratio >= normal_ratio:
                    zone_label = "DEPLETED FEED ZONE (HIGH DEMAND)"
                    zone_box_color = (0, 255, 0)
                else:
                    zone_label = "BALANCED FEEDING ZONE"
                    zone_box_color = (255, 200, 0)

                cv2.rectangle(
                    debug_frame, (zx, zy), (zx + zw, zy + zh), zone_box_color, 2
                )
                cv2.putText(
                    debug_frame,
                    zone_label,
                    (zx, max(20, zy - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    zone_box_color,
                    2,
                    cv2.LINE_AA,
                )

        # ---------------------------------------------------------------------
        # 3. Frame-to-Frame Motion / Agitation Score
        # ---------------------------------------------------------------------
        motion_score = 0.0
        if self.prev_gray is not None and self.prev_gray.shape == gray_blur.shape:
            frame_diff = cv2.absdiff(gray_blur, self.prev_gray)
            _, motion_thresh = cv2.threshold(frame_diff, 14, 255, cv2.THRESH_BINARY)
            motion_ratio = float(np.count_nonzero(motion_thresh)) / total_pixels
            motion_score = min(20.0, motion_ratio * 250.0)
        self.prev_gray = gray_blur

        # ---------------------------------------------------------------------
        # 4. Compute Combined Fish Feeding Demand / Depleted Food Score (0-100%)
        # ---------------------------------------------------------------------
        # Fish count contribution (up to 55 points for >= 8 fish in view)
        fish_count_score = min(55.0, fish_count * 7.2)

        # Fish area coverage contribution (up to 20 points)
        fish_area_ratio = fish_pixel_area / total_pixels
        fish_area_score = min(20.0, fish_area_ratio * 950.0)

        # Depleted feed tray exposure contribution (up to 25 points)
        pad_depletion_score = min(
            25.0, (depleted_ratio * 180.0) + (normal_ratio * 95.0)
        )

        raw_demand = min(
            100.0,
            max(
                0.0,
                fish_count_score + fish_area_score + pad_depletion_score + motion_score,
            ),
        )
        self.smoothed_demand = (
            self.ema_alpha * raw_demand + (1.0 - self.ema_alpha) * self.smoothed_demand
        )

        # Publish /vision/feeding_demand
        demand_msg = Float32()
        demand_msg.data = float(round(self.smoothed_demand, 2))
        self.demand_pub.publish(demand_msg)

        # ---------------------------------------------------------------------
        # 5. Render Rich Annotated HUD on /vision/debug_image
        # ---------------------------------------------------------------------
        self._draw_hud_overlay(
            debug_frame,
            fish_count=fish_count,
            demand_score=self.smoothed_demand,
            zone_label=zone_label,
        )

        try:
            out_img_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            out_img_msg.header = msg.header
            self.debug_img_pub.publish(out_img_msg)
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to publish debug_image: {exc}")

    def _draw_hud_overlay(
        self,
        canvas: np.ndarray,
        fish_count: int,
        demand_score: float,
        zone_label: str,
    ) -> None:
        """Draws semi-transparent telemetry header, activity bar, and water safety status."""
        h, w = canvas.shape[:2]
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (w, 105), (18, 24, 32), -1)
        cv2.addWeighted(overlay, 0.75, canvas, 0.25, 0, canvas)

        # Title & FSM State
        cv2.putText(
            canvas,
            f"AQUACULTURE VISION | FSM: {self.fsm_state}",
            (12, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (80, 220, 255),
            2,
            cv2.LINE_AA,
        )

        # Fish Count & Feeding Demand Score
        demand_color = (
            (0, 255, 100)
            if demand_score >= 45.0
            else ((0, 215, 255) if demand_score >= 20.0 else (190, 190, 190))
        )
        cv2.putText(
            canvas,
            f"Fish Detected: {fish_count:02d} | Feeding Demand Score: {demand_score:5.1f}%",
            (12, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            demand_color,
            1,
            cv2.LINE_AA,
        )

        # Activity Progress Bar
        bar_x, bar_y, bar_w, bar_h = 12, 54, 260, 12
        cv2.rectangle(
            canvas, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (90, 90, 90), 1
        )
        fill_w = int((min(100.0, max(0.0, demand_score)) / 100.0) * bar_w)
        cv2.rectangle(
            canvas,
            (bar_x, bar_y),
            (bar_x + fill_w, bar_y + bar_h),
            demand_color,
            -1,
        )

        # Water Quality Status Line
        ph = self.water_telemetry.get("ph", 7.5)
        do_val = self.water_telemetry.get("do", 6.5)
        temp = self.water_telemetry.get("temp", 25.0)
        turb = self.water_telemetry.get("turbidity", 12.0)
        weight = self.water_telemetry.get("weight", 5000.0)
        valve = self.water_telemetry.get("valve", 0)

        water_safe = (do_val >= 4.0) and (6.5 <= ph <= 8.5) and (temp <= 32.0)
        status_text = "WATER: SAFE" if water_safe else "WATER: CRITICAL WARNING!"
        status_color = (0, 255, 0) if water_safe else (0, 0, 255)

        cv2.putText(
            canvas,
            f"pH:{ph:.2f} | DO:{do_val:.2f}mg/L | Temp:{temp:.1f}C | Turb:{turb:.0f}NTU | Hopper:{weight:.0f}g",
            (12, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"{status_text} | VALVE: {'OPEN' if valve else 'CLOSED'} | {zone_label}",
            (12, 99),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            status_color,
            1,
            cv2.LINE_AA,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FishVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
