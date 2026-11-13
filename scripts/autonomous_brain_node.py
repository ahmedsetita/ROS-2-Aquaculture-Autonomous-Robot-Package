#!/usr/bin/python3
"""
Autonomous Brain & Finite State Machine (FSM) Navigation Controller
===================================================================
Main Raspberry Pi high-level controller executing a 4-state Finite State Machine:
  - STATE 1: PATROL (Closed-loop PID waypoint navigation using /cmd_vel, /odom, /imu/data)
  - STATE 2: INSPECT_AND_EVALUATE (Evaluates /esp32/sensors_filtered & /vision/feeding_demand;
             enforces strict water quality safety logic for DO, pH, and Temperature)
  - STATE 3: AUTONOMOUS_FEEDING (Holds position, opens bottom valve via /feeder_valve_cmd,
             monitors real-time HX711 weight until 250g target dose is dispensed)
  - STATE 4: RETURN_TO_HOME (Aborts patrol when HX711 weight < 300g and navigates to (0, 0) dock)
"""

from enum import Enum
import json
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String


class FSMState(Enum):
    PATROL = "PATROL"
    INSPECT_AND_EVALUATE = "INSPECT_AND_EVALUATE"
    AUTONOMOUS_FEEDING = "AUTONOMOUS_FEEDING"
    RETURN_TO_HOME = "RETURN_TO_HOME"


class PIDController:
    """Discrete-time PID controller with integral anti-windup and output saturation."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        min_out: float,
        max_out: float,
        integral_limit: float = 1.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral_limit = integral_limit

        self.integral: float = 0.0
        self.prev_error: float = 0.0
        self.first_run: bool = True

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 1e-5:
            return 0.0

        self.integral += error * dt
        self.integral = max(
            -self.integral_limit, min(self.integral_limit, self.integral)
        )

        if self.first_run:
            derivative = 0.0
            self.first_run = False
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(self.min_out, min(self.max_out, output))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Converts a quaternion (x, y, z, w) into Euler yaw angle in radians [-pi, pi]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wraps an angle in radians to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AutonomousBrainNode(Node):
    """Main autonomous decision-making and closed-loop PID navigation node."""

    def __init__(self) -> None:
        super().__init__("autonomous_brain_node")

        # Declare configurable parameters
        self.declare_parameter("target_feed_dose_g", 250.0)
        self.declare_parameter("low_feed_threshold_g", 300.0)
        self.declare_parameter("feeding_demand_threshold_pct", 35.0)
        self.declare_parameter("min_safe_do_mg_l", 4.0)
        self.declare_parameter("min_safe_ph", 6.5)
        self.declare_parameter("max_safe_ph", 8.5)
        self.declare_parameter("max_safe_temp_c", 32.0)
        self.declare_parameter("inspection_duration_sec", 2.5)
        self.declare_parameter("waypoint_tolerance_m", 0.25)

        self.target_dose_g: float = float(
            self.get_parameter("target_feed_dose_g").value
        )
        self.low_feed_threshold_g: float = float(
            self.get_parameter("low_feed_threshold_g").value
        )
        self.demand_threshold_pct: float = float(
            self.get_parameter("feeding_demand_threshold_pct").value
        )
        self.min_safe_do: float = float(self.get_parameter("min_safe_do_mg_l").value)
        self.min_safe_ph: float = float(self.get_parameter("min_safe_ph").value)
        self.max_safe_ph: float = float(self.get_parameter("max_safe_ph").value)
        self.max_safe_temp: float = float(self.get_parameter("max_safe_temp_c").value)
        self.inspect_duration_sec: float = float(
            self.get_parameter("inspection_duration_sec").value
        )
        self.wp_tolerance_m: float = float(
            self.get_parameter("waypoint_tolerance_m").value
        )

        # Pond Feeding Zone Waypoints: (name, zone_center_x, zone_center_y)
        # Note: The robot approaches along the vector toward the zone and stops 0.45m
        # before the zone center facing directly into the zone so the downward-angled
        # camera (43 deg pitch) frames the entire fish school while immersing the sensors.
        self.zones: List[Tuple[str, float, float]] = [
            ("Zone 1 [NE - High Fish Aggregation]", 3.5, 3.5),
            ("Zone 2 [NW - Normal Balanced Zone]", -3.5, 3.5),
            ("Zone 3 [SW - Anoxic / Poor Water Zone]", -3.5, -3.5),
            ("Zone 4 [SE - Active School & Sediment]", 3.5, -3.5),
        ]
        self.current_zone_idx: int = 0

        # FSM State
        self.state: FSMState = FSMState.PATROL
        self.state_enter_time = self.get_clock().now()
        self.feeding_start_weight_g: float = 5000.0
        self.valve_is_open: bool = False

        # Robot Pose & IMU Telemetry
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.odom_yaw: float = 0.0
        self.imu_yaw: float = 0.0
        self.fused_yaw: float = 0.0
        self.odom_received: bool = False
        self.imu_received: bool = False

        # Filtered ESP32 Sensor Readings & Vision Demand
        self.sensors: Dict[str, float] = {
            "weight": 5000.0,
            "ph": 7.50,
            "do": 6.50,
            "turbidity": 12.0,
            "temp": 25.0,
        }
        self.sensors_received: bool = False
        self.feeding_demand_pct: float = 0.0

        # Closed-Loop PID Controllers for Linear Speed and Heading Alignment
        self.linear_pid = PIDController(
            kp=0.85, ki=0.02, kd=0.12, min_out=0.0, max_out=0.85, integral_limit=0.5
        )
        self.angular_pid = PIDController(
            kp=1.90, ki=0.05, kd=0.20, min_out=-1.20, max_out=1.20, integral_limit=0.5
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, 10
        )
        self.sensors_sub = self.create_subscription(
            String, "/esp32/sensors_filtered", self.sensors_callback, 10
        )
        self.vision_sub = self.create_subscription(
            Float32, "/vision/feeding_demand", self.vision_callback, 10
        )
        self.dispatch_sub = self.create_subscription(
            String, "/aqua/dispatch_cmd", self.dispatch_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.valve_pub = self.create_publisher(Bool, "/feeder_valve_cmd", 10)
        self.refill_pub = self.create_publisher(Bool, "/feeder_refill_cmd", 10)
        self.state_pub = self.create_publisher(String, "/brain/state", 10)
        self.mission_pub = self.create_publisher(String, "/aqua/mission_status", 10)
        self.latest_mission_text: str = "Autonomous Pond Patrol -> Navigating to Zone 1 (3.5, 3.5)"

        # Control loop at 20 Hz
        self.last_loop_time = self.get_clock().now()
        self.control_timer = self.create_timer(0.05, self.fsm_control_loop)

        self.get_logger().info(
            "Autonomous Brain FSM Node initialized. Starting in STATE 1: PATROL."
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True
        self._update_fused_heading()

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        self.imu_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.imu_received = True
        self._update_fused_heading()

    def _update_fused_heading(self) -> None:
        if self.odom_received and self.imu_received:
            # Complementary fusion of Odometry and IMU yaw using vector average
            sin_sum = 0.7 * math.sin(self.odom_yaw) + 0.3 * math.sin(self.imu_yaw)
            cos_sum = 0.7 * math.cos(self.odom_yaw) + 0.3 * math.cos(self.imu_yaw)
            self.fused_yaw = math.atan2(sin_sum, cos_sum)
        elif self.odom_received:
            self.fused_yaw = self.odom_yaw
        elif self.imu_received:
            self.fused_yaw = self.imu_yaw

    def sensors_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            for key in ["weight", "ph", "do", "turbidity", "temp"]:
                if key in data:
                    self.sensors[key] = float(data[key])
            self.sensors_received = True
        except (json.JSONDecodeError, ValueError, TypeError):
            pass

    def vision_callback(self, msg: Float32) -> None:
        self.feeding_demand_pct = float(msg.data)

    def dispatch_callback(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        zone_map = {"Z1": 0, "Z2": 1, "Z3": 2, "Z4": 3}
        if cmd in zone_map:
            self.current_zone_idx = zone_map[cmd]
            self.set_feeder_valve(False)
            self.transition_to(
                FSMState.PATROL,
                f"Operator dispatched robot to {self.zones[self.current_zone_idx][0]}",
            )
        elif cmd == "HOME":
            self.set_feeder_valve(False)
            self.transition_to(
                FSMState.RETURN_TO_HOME,
                "Operator commanded Return to Home Dock (0.0, 0.0)",
            )
        elif cmd == "AUTO":
            self.set_feeder_valve(False)
            self.transition_to(
                FSMState.PATROL,
                f"Resuming Autonomous Patrol at {self.zones[self.current_zone_idx][0]}",
            )

    def set_feeder_valve(self, open_valve: bool) -> None:
        if self.valve_is_open != open_valve:
            self.valve_is_open = open_valve
            msg = Bool()
            msg.data = open_valve
            self.valve_pub.publish(msg)

    def stop_robot(self) -> None:
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def transition_to(self, new_state: FSMState, reason: str = "") -> None:
        if reason:
            self.latest_mission_text = reason
        if self.state != new_state:
            self.get_logger().info(
                f"[FSM TRANSITION] {self.state.value} -> {new_state.value} | {reason}"
            )
            self.state = new_state
            self.state_enter_time = self.get_clock().now()
            self.linear_pid.reset()
            self.angular_pid.reset()

    def compute_observation_waypoint(
        self, zone_x: float, zone_y: float
    ) -> Tuple[float, float, float]:
        """
        Computes a vantage point 0.45m radially inward from the zone center,
        facing directly at (zone_x, zone_y) so the downward-pitched camera
        frames the fish school center while immersing the water sensors in the zone.
        """
        angle_from_origin = math.atan2(zone_y, zone_x)
        standoff_dist = 0.45
        wp_x = zone_x - standoff_dist * math.cos(angle_from_origin)
        wp_y = zone_y - standoff_dist * math.sin(angle_from_origin)
        desired_yaw = angle_from_origin
        return wp_x, wp_y, desired_yaw

    def navigate_to_point(
        self,
        target_x: float,
        target_y: float,
        dt: float,
        final_yaw: float = 0.0,
        align_final_yaw: bool = False,
    ) -> bool:
        """
        Closed-loop PID navigation toward (target_x, target_y).
        Returns True when within waypoint tolerance (and aligned if requested).
        """
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        dist_error = math.hypot(dx, dy)

        twist = Twist()

        if dist_error > self.wp_tolerance_m:
            desired_heading = math.atan2(dy, dx)
            yaw_error = normalize_angle(desired_heading - self.fused_yaw)

            # Angular PID control
            omega_z = self.angular_pid.compute(yaw_error, dt)

            # Only drive forward fast when roughly facing the waypoint
            heading_scale = max(0.0, math.cos(yaw_error))
            raw_v = self.linear_pid.compute(dist_error, dt)
            linear_v = raw_v * heading_scale

            twist.linear.x = float(linear_v)
            twist.angular.z = float(omega_z)
            self.cmd_vel_pub.publish(twist)
            return False

        # Position reached; optionally align to final inspection yaw
        if align_final_yaw:
            yaw_error = normalize_angle(final_yaw - self.fused_yaw)
            if abs(yaw_error) > 0.12:
                twist.linear.x = 0.0
                twist.angular.z = float(self.angular_pid.compute(yaw_error, dt))
                self.cmd_vel_pub.publish(twist)
                return False

        self.stop_robot()
        return True

    def evaluate_water_quality(self) -> Tuple[bool, List[str]]:
        """
        Safety Logic: Checks whether Dissolved Oxygen, pH, and Temperature are safe
        for fish metabolism before dispensing feed.
        """
        violations: List[str] = []
        do_val = self.sensors["do"]
        ph_val = self.sensors["ph"]
        temp_val = self.sensors["temp"]

        if do_val < self.min_safe_do:
            violations.append(
                f"Low Dissolved Oxygen ({do_val:.2f} mg/L < {self.min_safe_do:.1f} mg/L)"
            )
        if ph_val < self.min_safe_ph or ph_val > self.max_safe_ph:
            violations.append(
                f"Anomalous pH ({ph_val:.2f} outside [{self.min_safe_ph:.1f}, {self.max_safe_ph:.1f}])"
            )
        if temp_val > self.max_safe_temp:
            violations.append(
                f"High Water Temperature ({temp_val:.2f}C > {self.max_safe_temp:.1f}C)"
            )

        return (len(violations) == 0), violations

    def fsm_control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05
        self.last_loop_time = now

        # Publish current FSM state string & mission status
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        mission_msg = String()
        mission_msg.data = self.latest_mission_text
        self.mission_pub.publish(mission_msg)

        if not self.odom_received:
            return

        # Global Preemption Check: Return to Home if feed hopper weight < 300g
        if (
            self.sensors_received
            and self.sensors["weight"] < self.low_feed_threshold_g
            and self.state != FSMState.RETURN_TO_HOME
        ):
            self.set_feeder_valve(False)
            self.stop_robot()
            self.transition_to(
                FSMState.RETURN_TO_HOME,
                f"Hopper weight ({self.sensors['weight']:.1f}g) < {self.low_feed_threshold_g:.1f}g threshold! Aborting patrol for refill.",
            )
            return

        # =====================================================================
        # STATE 1: PATROL (Autonomous Waypoint Navigation)
        # =====================================================================
        if self.state == FSMState.PATROL:
            self.set_feeder_valve(False)
            zone_name, zx, zy = self.zones[self.current_zone_idx]
            wp_x, wp_y, inspect_yaw = self.compute_observation_waypoint(zx, zy)

            reached = self.navigate_to_point(
                wp_x, wp_y, dt, final_yaw=inspect_yaw, align_final_yaw=True
            )
            if reached:
                self.transition_to(
                    FSMState.INSPECT_AND_EVALUATE,
                    f"Arrived at {zone_name} vantage point ({wp_x:.2f}, {wp_y:.2f}). Evaluating sensors & vision...",
                )

        # =====================================================================
        # STATE 2: INSPECT & EVALUATE
        # =====================================================================
        elif self.state == FSMState.INSPECT_AND_EVALUATE:
            self.stop_robot()
            self.set_feeder_valve(False)

            elapsed = (now - self.state_enter_time).nanoseconds / 1e9
            if elapsed < self.inspect_duration_sec:
                return

            zone_name, _, _ = self.zones[self.current_zone_idx]
            is_safe, violations = self.evaluate_water_quality()

            ph_val = self.sensors["ph"]
            do_val = self.sensors["do"]
            temp_val = self.sensors["temp"]
            turb_val = self.sensors["turbidity"]
            weight_val = self.sensors["weight"]

            if not is_safe:
                violation_str = "; ".join(violations)
                self.get_logger().error(
                    f"CRITICAL WATER QUALITY WARNING at {zone_name}: {violation_str}! "
                    f"[Telemetry: DO={do_val:.2f}mg/L, pH={ph_val:.2f}, Temp={temp_val:.1f}C, Turb={turb_val:.1f}NTU]. "
                    f"Skipping heavy feeding to prevent fish suffocation & ammonia toxicity!"
                )
                self.current_zone_idx = (self.current_zone_idx + 1) % len(self.zones)
                self.transition_to(
                    FSMState.PATROL,
                    f"Skipped unsafe {zone_name}. Advancing to {self.zones[self.current_zone_idx][0]}.",
                )
                return

            # Water is SAFE -> Check Vision Feeding Demand or Sensor Activity
            effective_demand = max(
                self.feeding_demand_pct,
                min(85.0, (turb_val - 15.0) * 1.45) if turb_val > 30.0 else 0.0,
            )

            self.get_logger().info(
                f"[INSPECTION REPORT] {zone_name} | Water SAFE (DO={do_val:.2f}mg/L, pH={ph_val:.2f}, "
                f"Temp={temp_val:.1f}C, Turb={turb_val:.1f}NTU) | Vision Demand={self.feeding_demand_pct:.1f}% "
                f"(Effective Demand={effective_demand:.1f}%) | Hopper={weight_val:.1f}g"
            )

            if effective_demand >= self.demand_threshold_pct:
                self.feeding_start_weight_g = weight_val
                self.transition_to(
                    FSMState.AUTONOMOUS_FEEDING,
                    f"Water quality SAFE and high fish demand ({effective_demand:.1f}% >= {self.demand_threshold_pct:.1f}%). "
                    f"Dispensing {self.target_dose_g:.0f}g feed dose.",
                )
            else:
                self.current_zone_idx = (self.current_zone_idx + 1) % len(self.zones)
                self.transition_to(
                    FSMState.PATROL,
                    f"Low feeding demand ({effective_demand:.1f}%) at {zone_name}. Moving to next waypoint.",
                )

        # =====================================================================
        # STATE 3: AUTONOMOUS FEEDING
        # =====================================================================
        elif self.state == FSMState.AUTONOMOUS_FEEDING:
            self.stop_robot()
            self.set_feeder_valve(True)

            current_weight = self.sensors["weight"]
            dispensed_g = self.feeding_start_weight_g - current_weight
            zone_name, _, _ = self.zones[self.current_zone_idx]

            # Safety re-check while dispensing
            is_safe, violations = self.evaluate_water_quality()
            if not is_safe:
                self.set_feeder_valve(False)
                self.get_logger().error(
                    f"Emergency valve shutoff at {zone_name} due to water quality change: {violations}"
                )
                self.current_zone_idx = (self.current_zone_idx + 1) % len(self.zones)
                self.transition_to(FSMState.PATROL, "Aborted feeding on water alert.")
                return

            if dispensed_g >= self.target_dose_g:
                self.set_feeder_valve(False)
                self.get_logger().info(
                    f"[FEEDING COMPLETE] Dispensed {dispensed_g:.1f}g at {zone_name}. "
                    f"Remaining Hopper Weight: {current_weight:.1f}g."
                )
                self.current_zone_idx = (self.current_zone_idx + 1) % len(self.zones)
                self.transition_to(
                    FSMState.PATROL,
                    f"Target dose dispensed. Proceeding to {self.zones[self.current_zone_idx][0]}.",
                )

        # =====================================================================
        # STATE 4: RETURN TO HOME (Refill at (0, 0) Dock)
        # =====================================================================
        elif self.state == FSMState.RETURN_TO_HOME:
            self.set_feeder_valve(False)
            reached_home = self.navigate_to_point(
                0.0, 0.0, dt, final_yaw=0.0, align_final_yaw=True
            )
            if reached_home:
                if self.sensors["weight"] < 4500.0:
                    refill_msg = Bool()
                    refill_msg.data = True
                    self.refill_pub.publish(refill_msg)
                else:
                    self.get_logger().info(
                        f"Hopper refilled at Home Dock (Weight: {self.sensors['weight']:.1f}g). Resuming autonomous patrol!"
                    )
                    self.transition_to(
                        FSMState.PATROL,
                        "Refill complete at (0, 0) dock. Resuming pond patrol.",
                    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_feeder_valve(False)
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
