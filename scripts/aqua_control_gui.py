#!/usr/bin/python3
"""
Live Aquaculture Autonomous Control Center & ESP32 Telemetry Dashboard GUI
==========================================================================
Interactive Tkinter + OpenCV/PIL dashboard (matching the AMR car project GUI,
adapted for the autonomous aquaculture water robot):
  - Displays live annotated underwater camera feed (/vision/debug_image)
  - Displays real-time Raw vs. Filtered ESP32 sensor telemetry (Weight, pH, DO, Turbidity, Temp)
  - Shows live Autonomous FSM state, target zone, and water safety alerts
  - Provides interactive buttons to dispatch the robot to any pond zone or toggle feeding
"""

import json
import os
import threading
import tkinter as tk
from tkinter import ttk
from typing import Dict, Optional

import cv2
import numpy as np
from PIL import Image, ImageTk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Bool, Float32, String


class AquaControlDashboardNode(Node):
    """ROS 2 node backing the Live Aquaculture Control Center GUI."""

    def __init__(self) -> None:
        super().__init__("aqua_control_gui")

        self.lock = threading.Lock()
        self.latest_rgb_frame: Optional[np.ndarray] = None
        self.fsm_state: str = "PATROL"
        self.mission_detail: str = "Autonomous Pond Patrol Active -> Navigating to Zone 1 (3.5, 3.5)"
        self.feeding_demand: float = 0.0

        self.raw_sensors: Dict[str, float] = {
            "weight": 5000.0,
            "ph": 7.50,
            "do": 6.50,
            "turbidity": 12.0,
            "temp": 25.0,
            "valve": 0,
            "x": 0.0,
            "y": 0.0,
        }
        self.filt_sensors: Dict[str, float] = dict(self.raw_sensors)

        # Publishers for interactive control
        self.dispatch_pub = self.create_publisher(String, "/aqua/dispatch_cmd", 10)
        self.valve_pub = self.create_publisher(Bool, "/feeder_valve_cmd", 10)
        self.refill_pub = self.create_publisher(Bool, "/feeder_refill_cmd", 10)

        # Subscribers
        self.create_subscription(RosImage, "/vision/debug_image", self._img_cb, 10)
        self.create_subscription(String, "/esp32/sensors_raw", self._raw_cb, 10)
        self.create_subscription(String, "/esp32/sensors_filtered", self._filt_cb, 10)
        self.create_subscription(String, "/brain/state", self._state_cb, 10)
        self.create_subscription(String, "/aqua/mission_status", self._mission_cb, 10)
        self.create_subscription(Float32, "/vision/feeding_demand", self._demand_cb, 10)

    def _img_cb(self, msg: RosImage) -> None:
        if msg.width == 0 or msg.height == 0:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size < msg.width * msg.height * 3:
            return
        bgr = arr.reshape((msg.height, msg.width, 3))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        with self.lock:
            self.latest_rgb_frame = rgb

    def _raw_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.raw_sensors.update(data)
        except Exception:
            pass

    def _filt_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.filt_sensors.update(data)
        except Exception:
            pass

    def _state_cb(self, msg: String) -> None:
        with self.lock:
            self.fsm_state = msg.data

    def _mission_cb(self, msg: String) -> None:
        with self.lock:
            self.mission_detail = msg.data

    def _demand_cb(self, msg: Float32) -> None:
        with self.lock:
            self.feeding_demand = float(msg.data)

    def send_dispatch(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.dispatch_pub.publish(msg)

    def send_valve(self, open_valve: bool) -> None:
        msg = Bool()
        msg.data = open_valve
        self.valve_pub.publish(msg)

    def send_refill(self) -> None:
        msg = Bool()
        msg.data = True
        self.refill_pub.publish(msg)


def build_and_run_gui(ros_node: AquaControlDashboardNode) -> None:
    root = tk.Tk()
    root.title("Autonomous Aquaculture Robot — Water Quality, Fish Vision & ESP32 Command Center")
    root.geometry("1120x680")
    root.configure(bg="#0f172a")

    # Header Banner
    header = tk.Frame(root, bg="#1e293b", height=60)
    header.pack(fill=tk.X, padx=10, pady=(10, 5))

    title_lbl = tk.Label(
        header,
        text="AQUACULTURE AUTONOMOUS WATER ROBOT — LIVE AI VISION & ESP32 SENSOR FUSION",
        font=("DejaVu Sans", 13, "bold"),
        fg="#38bdf8",
        bg="#1e293b",
    )
    title_lbl.pack(anchor="w", padx=12, pady=(6, 2))

    mission_lbl = tk.Label(
        header,
        text="Initializing autonomous pond patrol...",
        font=("DejaVu Sans", 10, "bold"),
        fg="#4ade80",
        bg="#1e293b",
    )
    mission_lbl.pack(anchor="w", padx=12, pady=(0, 6))

    # Main Split Layout: Left = Live Camera Feed, Right = ESP32 Telemetry + Zone Dispatch
    body = tk.Frame(root, bg="#0f172a")
    body.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    left_panel = tk.LabelFrame(
        body,
        text=" Live Downward/Forward Fish Monitoring Camera (/vision/debug_image) ",
        font=("DejaVu Sans", 10, "bold"),
        fg="#e2e8f0",
        bg="#1e293b",
        bd=2,
    )
    left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

    cam_label = tk.Label(left_panel, bg="#090d16")
    cam_label.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

    right_panel = tk.Frame(body, bg="#0f172a", width=440)
    right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
    right_panel.pack_propagate(False)

    # ESP32 Sensor Telemetry Frame (Raw vs Filtered)
    sensor_frame = tk.LabelFrame(
        right_panel,
        text=" ESP32 Sensors: Raw (Noisy) vs. Filtered (Median+Kalman+EMA) ",
        font=("DejaVu Sans", 10, "bold"),
        fg="#38bdf8",
        bg="#1e293b",
        bd=2,
    )
    sensor_frame.pack(fill=tk.X, pady=(0, 8))

    sensor_labels: Dict[str, tk.Label] = {}
    sensor_specs = [
        ("weight", "1. HX711 Hopper Weight (g):"),
        ("ph", "2. Water Alkalinity (pH):"),
        ("do", "3. Dissolved Oxygen (mg/L):"),
        ("turbidity", "4. Water Turbidity (NTU):"),
        ("temp", "5. Water Temperature (°C):"),
        ("pose", "Robot Water Pose (X, Y):"),
        ("safety", "Water Quality Safety Check:"),
    ]
    for key, title in sensor_specs:
        row = tk.Frame(sensor_frame, bg="#1e293b")
        row.pack(fill=tk.X, padx=10, pady=4)
        tk.Label(
            row,
            text=title,
            font=("DejaVu Sans", 9, "bold"),
            fg="#cbd5e1",
            bg="#1e293b",
        ).pack(side=tk.LEFT)
        val_lbl = tk.Label(
            row,
            text="--",
            font=("DejaVu Sans Mono", 9, "bold"),
            fg="#f8fafc",
            bg="#1e293b",
        )
        val_lbl.pack(side=tk.RIGHT)
        sensor_labels[key] = val_lbl

    # Autonomous Zone Dispatch & Actuator Controls
    ctrl_frame = tk.LabelFrame(
        right_panel,
        text=" Autonomous Pond Navigation & Feeder Valve Controls ",
        font=("DejaVu Sans", 10, "bold"),
        fg="#facc15",
        bg="#1e293b",
        bd=2,
    )
    ctrl_frame.pack(fill=tk.BOTH, expand=True)

    buttons = [
        ("AUTO PATROL ALL ZONES (FSM Loop)", "AUTO", "#0284c7"),
        ("Dispatch -> Zone 1 (3.5, 3.5) [High Fish Demand]", "Z1", "#0d9488"),
        ("Dispatch -> Zone 2 (-3.5, 3.5) [Balanced Normal]", "Z2", "#059669"),
        ("Dispatch -> Zone 3 (-3.5, -3.5) [Poor Water / Hypoxic]", "Z3", "#be123c"),
        ("Dispatch -> Zone 4 (3.5, -3.5) [Active Fish School]", "Z4", "#2563eb"),
        ("Return to Home Refill Dock (0.0, 0.0)", "HOME", "#4f46e5"),
    ]
    for text, cmd_code, bg_col in buttons:
        btn = tk.Button(
            ctrl_frame,
            text=text,
            font=("DejaVu Sans", 9, "bold"),
            bg=bg_col,
            fg="white",
            activebackground="#38bdf8",
            relief=tk.FLAT,
            pady=5,
            command=lambda c=cmd_code: ros_node.send_dispatch(c),
        )
        btn.pack(fill=tk.X, padx=10, pady=3)

    # Bottom row for manual valve & refill
    act_row = tk.Frame(ctrl_frame, bg="#1e293b")
    act_row.pack(fill=tk.X, padx=10, pady=(6, 8))

    tk.Button(
        act_row,
        text="OPEN VALVE",
        font=("DejaVu Sans", 9, "bold"),
        bg="#dc2626",
        fg="white",
        relief=tk.FLAT,
        command=lambda: ros_node.send_valve(True),
    ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 3))

    tk.Button(
        act_row,
        text="CLOSE VALVE",
        font=("DejaVu Sans", 9, "bold"),
        bg="#475569",
        fg="white",
        relief=tk.FLAT,
        command=lambda: ros_node.send_valve(False),
    ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)

    tk.Button(
        act_row,
        text="REFILL 5000g",
        font=("DejaVu Sans", 9, "bold"),
        bg="#ca8a04",
        fg="white",
        relief=tk.FLAT,
        command=ros_node.send_refill,
    ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(3, 0))

    def update_ui() -> None:
        with ros_node.lock:
            rgb = ros_node.latest_rgb_frame
            raw = dict(ros_node.raw_sensors)
            filt = dict(ros_node.filt_sensors)
            state = ros_node.fsm_state
            mission = ros_node.mission_detail
            demand = ros_node.feeding_demand

        mission_lbl.configure(
            text=f"FSM STATE: [{state}] | Demand: {demand:.1f}% | {mission}"
        )

        sensor_labels["weight"].configure(
            text=f"Filt: {filt['weight']:6.1f}g  (Raw: {raw['weight']:6.1f}g)"
        )
        sensor_labels["ph"].configure(
            text=f"Filt: {filt['ph']:5.2f}   (Raw: {raw['ph']:5.2f})"
        )
        sensor_labels["do"].configure(
            text=f"Filt: {filt['do']:5.2f}   (Raw: {raw['do']:5.2f})"
        )
        sensor_labels["turbidity"].configure(
            text=f"Filt: {filt['turbidity']:5.1f}   (Raw: {raw['turbidity']:5.1f})"
        )
        sensor_labels["temp"].configure(
            text=f"Filt: {filt['temp']:5.1f}°C (Raw: {raw['temp']:5.1f}°C)"
        )
        sensor_labels["pose"].configure(
            text=f"X = {filt.get('x', 0.0):+5.2f}m, Y = {filt.get('y', 0.0):+5.2f}m | Valve: {'OPEN' if filt.get('valve', 0) else 'CLOSED'}"
        )

        is_safe = (
            (filt["do"] >= 4.0)
            and (6.5 <= filt["ph"] <= 8.5)
            and (filt["temp"] <= 32.0)
        )
        sensor_labels["safety"].configure(
            text="SAFE FOR FEEDING" if is_safe else "CRITICAL WATER WARNING (SKIP FEED!)",
            fg="#4ade80" if is_safe else "#f87171",
        )

        if rgb is not None:
            resized_rgb = cv2.resize(rgb, (620, 465), interpolation=cv2.INTER_LINEAR)
            pil_img = Image.fromarray(resized_rgb)
            tk_img = ImageTk.PhotoImage(image=pil_img)
            cam_label.configure(image=tk_img)
            cam_label.image = tk_img

        root.after(80, update_ui)

    root.after(100, update_ui)
    root.mainloop()


def main(args=None) -> None:
    if not os.environ.get("DISPLAY"):
        print("No DISPLAY environment variable found; skipping Tkinter GUI window.")
        return

    rclpy.init(args=args)
    node = AquaControlDashboardNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        build_and_run_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
