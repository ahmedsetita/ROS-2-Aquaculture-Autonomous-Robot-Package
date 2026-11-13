#!/usr/bin/python3
"""
ESP32 Hardware Sensor Suite & Noise Filtering Simulation Node
=============================================================
Simulates the 5 aquaculture sensors connected to the ESP32 dynamically
as a function of the robot's (X, Y) coordinates in the fish pond:
  1. LOADCELLS + HX711 (Remaining feed weight in grams, starts at 5000g, -50g/s when valve OPEN)
  2. PH-SENSOR (Water pH, normal ~7.5, anomalous in poor water quality zones)
  3. Dissolved Oxygen Sensor (DO in mg/L, normal ~6.5 mg/L, drops in hypoxic/dense zones)
  4. TURBIDITY SENSOR (Water cloudiness in NTU)
  5. TEMPERATURE SENSOR (Water temperature in Celsius, ~25.0C + spatial gradient)

Injects realistic Gaussian white noise + random ADC voltage spikes into raw readings,
and runs a 3-stage digital filter pipeline:
  [Sliding-Window Median Filter] -> [1D Kalman Filter] -> [Exponential Moving Average (EMA)]

Publishes:
  - /esp32/sensors_raw (std_msgs/String JSON)
  - /esp32/sensors_filtered (std_msgs/String JSON)
  - /esp32/raw/<sensor> & /esp32/filtered/<sensor> (std_msgs/Float32 for rqt_plot)
  - /joint_states (sensor_msgs/JointState for feeder_valve_joint visual actuation)
"""

from collections import deque
import json
import math
import random
from typing import Dict

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String


class MedianKalmanEMAFilter:
    """
    Combined 3-stage digital signal filter for noisy embedded aquaculture sensors:
      1. Sliding-Window Median Filter: Rejects impulse noise / random ADC voltage spikes.
      2. 1D Linear Kalman Filter: Optimal state estimation under Gaussian white noise.
      3. Exponential Moving Average (EMA): Smooths high-frequency residual jitter.
    """

    def __init__(
        self,
        window_size: int = 7,
        process_variance_q: float = 1e-3,
        measurement_variance_r: float = 0.05,
        ema_alpha: float = 0.35,
        initial_value: float = 0.0,
    ) -> None:
        self.window_size = max(3, int(window_size) | 1)  # Ensure odd window size >= 3
        self.buffer: deque = deque(maxlen=self.window_size)
        self.q = float(process_variance_q)
        self.r = float(measurement_variance_r)
        self.alpha = float(ema_alpha)

        self.x_hat = float(initial_value)
        self.p_cov = 1.0
        self.ema_val = float(initial_value)
        self.initialized = False

    def reset(self, value: float) -> None:
        self.buffer.clear()
        for _ in range(self.window_size):
            self.buffer.append(float(value))
        self.x_hat = float(value)
        self.p_cov = 0.1
        self.ema_val = float(value)
        self.initialized = True

    def update(self, raw_measurement: float) -> float:
        if not self.initialized:
            self.reset(raw_measurement)
            return self.ema_val

        # Stage 1: Median Filter (Spike Rejection)
        self.buffer.append(float(raw_measurement))
        sorted_buf = sorted(self.buffer)
        median_val = sorted_buf[len(sorted_buf) // 2]

        # Stage 2: 1D Kalman Filter (Gaussian Noise Suppression)
        # Prediction step
        p_pred = self.p_cov + self.q
        # Update step
        kalman_gain = p_pred / (p_pred + self.r)
        self.x_hat = self.x_hat + kalman_gain * (median_val - self.x_hat)
        self.p_cov = (1.0 - kalman_gain) * p_pred

        # Stage 3: Exponential Moving Average (EMA Low-Pass Smoothing)
        self.ema_val = self.alpha * self.x_hat + (1.0 - self.alpha) * self.ema_val
        return self.ema_val


class ESP32SensorSimNode(Node):
    """Simulates the ESP32 aquaculture sensor suite, ADC noise, and digital filtering."""

    def __init__(self) -> None:
        super().__init__("esp32_sim_node")

        # Declare ROS 2 parameters
        self.declare_parameter("initial_feed_weight_g", 5000.0)
        self.declare_parameter("dispense_rate_g_per_sec", 50.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("spike_probability", 0.08)
        self.declare_parameter("auto_refill_at_dock", True)

        self.feed_weight_g: float = float(
            self.get_parameter("initial_feed_weight_g").value
        )
        self.max_feed_weight_g: float = 5000.0
        self.dispense_rate: float = float(
            self.get_parameter("dispense_rate_g_per_sec").value
        )
        self.publish_rate_hz: float = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.spike_prob: float = float(
            self.get_parameter("spike_probability").value
        )
        self.auto_refill_at_dock: bool = bool(
            self.get_parameter("auto_refill_at_dock").value
        )

        # Robot state from /odom and actuator command from /feeder_valve_cmd
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.valve_open: bool = False
        self.last_time = self.get_clock().now()

        # Instantiate 3-Stage Filters for all 5 sensors
        self.filters: Dict[str, MedianKalmanEMAFilter] = {
            "weight": MedianKalmanEMAFilter(
                window_size=7,
                process_variance_q=0.5,
                measurement_variance_r=15.0,
                ema_alpha=0.45,
                initial_value=self.feed_weight_g,
            ),
            "ph": MedianKalmanEMAFilter(
                window_size=7,
                process_variance_q=0.002,
                measurement_variance_r=0.08,
                ema_alpha=0.30,
                initial_value=7.50,
            ),
            "do": MedianKalmanEMAFilter(
                window_size=7,
                process_variance_q=0.005,
                measurement_variance_r=0.12,
                ema_alpha=0.30,
                initial_value=6.50,
            ),
            "turbidity": MedianKalmanEMAFilter(
                window_size=7,
                process_variance_q=0.2,
                measurement_variance_r=8.0,
                ema_alpha=0.30,
                initial_value=12.0,
            ),
            "temp": MedianKalmanEMAFilter(
                window_size=7,
                process_variance_q=0.002,
                measurement_variance_r=0.06,
                ema_alpha=0.25,
                initial_value=25.0,
            ),
        }

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.valve_sub = self.create_subscription(
            Bool, "/feeder_valve_cmd", self.valve_callback, 10
        )
        self.refill_sub = self.create_subscription(
            Bool, "/feeder_refill_cmd", self.refill_callback, 10
        )

        # Primary JSON Publishers (as requested in specification)
        self.raw_json_pub = self.create_publisher(String, "/esp32/sensors_raw", 10)
        self.filt_json_pub = self.create_publisher(
            String, "/esp32/sensors_filtered", 10
        )

        # Individual Float32 Publishers for direct rqt_plot noise comparison
        self.raw_float_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.filt_float_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for sensor_name in ["weight", "ph", "do", "turbidity", "temp"]:
            self.raw_float_pubs[sensor_name] = self.create_publisher(
                Float32, f"/esp32/raw/{sensor_name}", 10
            )
            self.filt_float_pubs[sensor_name] = self.create_publisher(
                Float32, f"/esp32/filtered/{sensor_name}", 10
            )

        # JointState publisher for the bottom red dispenser valve visual indicator
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Timer loop
        timer_period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(timer_period, self.sensor_update_loop)

        self.get_logger().info(
            f"ESP32 Sensor Simulation Node initialized at {self.publish_rate_hz:.1f} Hz "
            f"(Initial Hopper Feed Weight: {self.feed_weight_g:.1f}g)"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def valve_callback(self, msg: Bool) -> None:
        if msg.data != self.valve_open:
            state_str = "OPEN (Dispensing 50g/s)" if msg.data else "CLOSED"
            self.get_logger().info(f"Feeder Dispenser Valve toggled -> {state_str}")
        self.valve_open = bool(msg.data)

    def refill_callback(self, msg: Bool) -> None:
        if msg.data:
            self.feed_weight_g = self.max_feed_weight_g
            self.filters["weight"].reset(self.feed_weight_g)
            self.get_logger().info(
                f"Hopper refilled to {self.feed_weight_g:.1f}g via /feeder_refill_cmd!"
            )

    @staticmethod
    def _gaussian_zone_influence(
        x: float, y: float, zx: float, zy: float, sigma: float = 1.45
    ) -> float:
        """Computes smooth radial Gaussian weight [0.0, 1.0] around zone center (zx, zy)."""
        dist_sq = (x - zx) ** 2 + (y - zy) ** 2
        return math.exp(-dist_sq / (2.0 * sigma * sigma))

    def compute_ground_truth_environment(self, dt: float) -> Dict[str, float]:
        """
        Computes noise-free physical ground-truth values for all 5 sensors
        based on robot position (X, Y) and feeder valve state.
        """
        # 1. Load Cell + HX711 Weight Dynamics
        if self.valve_open and self.feed_weight_g > 0.0:
            self.feed_weight_g = max(
                0.0, self.feed_weight_g - self.dispense_rate * dt
            )

        # Automatic dock refill when stationed at Home (0, 0) and low on feed
        dist_to_home = math.hypot(self.robot_x, self.robot_y)
        if (
            self.auto_refill_at_dock
            and dist_to_home < 0.45
            and not self.valve_open
            and self.feed_weight_g < 300.0
        ):
            self.feed_weight_g = self.max_feed_weight_g
            self.filters["weight"].reset(self.feed_weight_g)
            self.get_logger().info(
                "Robot docked at (0, 0) refill station -> Hopper automatically refilled to 5000.0g!"
            )

        # Compute spatial influence weights for each of the 4 pond zones
        w_z1 = self._gaussian_zone_influence(self.robot_x, self.robot_y, 3.5, 3.5)
        w_z2 = self._gaussian_zone_influence(self.robot_x, self.robot_y, -3.5, 3.5)
        w_z3 = self._gaussian_zone_influence(self.robot_x, self.robot_y, -3.5, -3.5)
        w_z4 = self._gaussian_zone_influence(self.robot_x, self.robot_y, 3.5, -3.5)

        # 2. pH Sensor Ground Truth
        # Normal ~7.5; Zone 3 (-3.5, -3.5) is acidic (drops to ~5.75 < 6.5 threshold)
        true_ph = (
            7.50
            - 0.20 * w_z1
            + 0.05 * w_z2
            - 1.75 * w_z3
            + 0.15 * w_z4
        )

        # 3. Dissolved Oxygen (DO) Sensor Ground Truth (mg/L)
        # Normal ~6.5 mg/L; Zone 3 drops to ~3.0 mg/L (< 4.0 mg/L critical hypoxia threshold)
        true_do = (
            6.55
            - 1.35 * w_z1
            - 0.15 * w_z2
            - 3.55 * w_z3
            - 1.10 * w_z4
        )

        # 4. Turbidity Sensor Ground Truth (NTU)
        # Normal ~12 NTU; increases in active fish zones (Z1, Z4) and murky anoxic zone (Z3)
        true_turbidity = (
            12.0
            + 34.0 * w_z1
            + 8.0 * w_z2
            + 105.0 * w_z3
            + 56.0 * w_z4
        )

        # 5. Water Temperature Sensor Ground Truth (Celsius)
        # Base 25.0C with spatial solar gradient + localized thermal anomaly at Zone 3
        true_temp = (
            25.0
            + 0.12 * self.robot_x
            - 0.08 * self.robot_y
            + 1.8 * w_z3
        )

        return {
            "weight": self.feed_weight_g,
            "ph": true_ph,
            "do": true_do,
            "turbidity": true_turbidity,
            "temp": true_temp,
        }

    def inject_sensor_noise(self, true_vals: Dict[str, float]) -> Dict[str, float]:
        """
        Injects Gaussian white noise + random electromagnetic/ADC voltage spikes
        into the ground-truth sensor readings.
        """
        noise_specs = {
            # sensor: (gaussian_std, spike_magnitude)
            "weight": (6.5, 145.0),
            "ph": (0.14, 1.85),
            "do": (0.22, 2.40),
            "turbidity": (3.2, 48.0),
            "temp": (0.18, 3.20),
        }

        raw_vals: Dict[str, float] = {}
        for key, true_val in true_vals.items():
            std_dev, spike_mag = noise_specs[key]
            gaussian_noise = random.gauss(0.0, std_dev)

            # Random high-amplitude ADC spike (motor commutator EMI / power ripple)
            spike = 0.0
            if random.random() < self.spike_prob:
                spike = random.choice([-1.0, 1.0]) * spike_mag * random.uniform(0.6, 1.3)

            raw_reading = true_val + gaussian_noise + spike
            if key in ("weight", "do", "turbidity"):
                raw_reading = max(0.0, raw_reading)
            raw_vals[key] = raw_reading

        return raw_vals

    def sensor_update_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.publish_rate_hz
        self.last_time = now

        # 1. Compute physical ground truth
        true_vals = self.compute_ground_truth_environment(dt)

        # 2. Inject Gaussian noise + random voltage spikes
        raw_vals = self.inject_sensor_noise(true_vals)

        # 3. Run combined Median + Kalman + EMA filtering pipeline
        filtered_vals: Dict[str, float] = {}
        for key, raw_val in raw_vals.items():
            filtered_vals[key] = self.filters[key].update(raw_val)

        # 4. Publish JSON packets on /esp32/sensors_raw and /esp32/sensors_filtered
        raw_packet = {
            "weight": round(raw_vals["weight"], 2),
            "ph": round(raw_vals["ph"], 3),
            "do": round(raw_vals["do"], 3),
            "turbidity": round(raw_vals["turbidity"], 2),
            "temp": round(raw_vals["temp"], 2),
            "valve": 1 if self.valve_open else 0,
            "x": round(self.robot_x, 2),
            "y": round(self.robot_y, 2),
        }
        filt_packet = {
            "weight": round(filtered_vals["weight"], 2),
            "ph": round(filtered_vals["ph"], 3),
            "do": round(filtered_vals["do"], 3),
            "turbidity": round(filtered_vals["turbidity"], 2),
            "temp": round(filtered_vals["temp"], 2),
            "valve": 1 if self.valve_open else 0,
            "x": round(self.robot_x, 2),
            "y": round(self.robot_y, 2),
        }

        raw_msg = String()
        raw_msg.data = json.dumps(raw_packet)
        self.raw_json_pub.publish(raw_msg)

        filt_msg = String()
        filt_msg.data = json.dumps(filt_packet)
        self.filt_json_pub.publish(filt_msg)

        # 5. Publish numeric Float32 topics for rqt_plot
        for key in ["weight", "ph", "do", "turbidity", "temp"]:
            f_raw = Float32()
            f_raw.data = float(raw_vals[key])
            self.raw_float_pubs[key].publish(f_raw)

            f_filt = Float32()
            f_filt.data = float(filtered_vals[key])
            self.filt_float_pubs[key].publish(f_filt)

        # 6. Publish JointState for the bottom red dispenser valve spool
        js_msg = JointState()
        js_msg.header.stamp = now.to_msg()
        js_msg.name = ["feeder_valve_joint"]
        js_msg.position = [0.05 if self.valve_open else 0.0]
        js_msg.velocity = [0.0]
        js_msg.effort = [0.0]
        self.joint_state_pub.publish(js_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ESP32SensorSimNode()
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
