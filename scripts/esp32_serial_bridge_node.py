#!/usr/bin/python3
"""
ESP32 Hardware Serial Bridge Node (Real Hardware Mode)
======================================================
Companion ROS 2 node that bridges the physical USB-connected ESP32 running
`firmware/esp32_aquaculture_firmware.ino` (at 115200 baud) into the exact same
ROS 2 topics used by the Gazebo simulation (`esp32_sim_node.py`):

  - Publishes /esp32/sensors_filtered (std_msgs/String JSON)
  - Publishes /esp32/sensors_raw (std_msgs/String JSON)
  - Publishes /esp32/raw/<sensor> & /esp32/filtered/<sensor> (std_msgs/Float32 for rqt_plot)
  - Publishes /joint_states (sensor_msgs/JointState for feeder_valve_joint)
  - Subscribes to /feeder_valve_cmd (std_msgs/Bool) -> sends `{"valve": 1}\n` or `{"valve": 0}\n`
  - Subscribes to /feeder_tare_cmd (std_msgs/Bool) -> sends `{"tare": 1}\n`
"""

import json
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None
    SerialException = Exception


class ESP32SerialBridgeNode(Node):
    """Bidirectional UART JSON bridge between Raspberry Pi ROS 2 and physical ESP32."""

    def __init__(self) -> None:
        super().__init__("esp32_serial_bridge_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.02)
        self.declare_parameter("reconnect_interval_sec", 2.0)

        self.port_name: str = str(self.get_parameter("serial_port").value)
        self.baud_rate: int = int(self.get_parameter("baud_rate").value)
        self.timeout_sec: float = float(
            self.get_parameter("serial_timeout_sec").value
        )
        self.reconnect_sec: float = float(
            self.get_parameter("reconnect_interval_sec").value
        )

        self.serial_conn: Optional["serial.Serial"] = None
        self.valve_open: bool = False
        self.last_reconnect_attempt = self.get_clock().now()

        # Subscribers
        self.valve_sub = self.create_subscription(
            Bool, "/feeder_valve_cmd", self.valve_cmd_callback, 10
        )
        self.tare_sub = self.create_subscription(
            Bool, "/feeder_tare_cmd", self.tare_cmd_callback, 10
        )

        # JSON Publishers
        self.raw_json_pub = self.create_publisher(String, "/esp32/sensors_raw", 10)
        self.filt_json_pub = self.create_publisher(
            String, "/esp32/sensors_filtered", 10
        )

        # Numeric Float32 Publishers for rqt_plot
        self.raw_float_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.filt_float_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for name in ["weight", "ph", "do", "turbidity", "temp"]:
            self.raw_float_pubs[name] = self.create_publisher(
                Float32, f"/esp32/raw/{name}", 10
            )
            self.filt_float_pubs[name] = self.create_publisher(
                Float32, f"/esp32/filtered/{name}", 10
            )

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        self._try_connect_serial()
        self.poll_timer = self.create_timer(0.02, self.serial_poll_loop)

    def _try_connect_serial(self) -> None:
        if serial is None:
            self.get_logger().error(
                "pyserial is not installed! Install via `sudo apt install python3-serial`."
            )
            return

        try:
            self.serial_conn = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=self.timeout_sec,
            )
            self.get_logger().info(
                f"Connected to physical ESP32 on {self.port_name} @ {self.baud_rate} baud."
            )
        except SerialException as exc:
            self.serial_conn = None
            self.get_logger().warn(
                f"Waiting for ESP32 on {self.port_name} ({exc}). Will retry every {self.reconnect_sec}s..."
            )

    def _send_serial_json(self, payload: dict) -> None:
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            line = json.dumps(payload) + "\n"
            self.serial_conn.write(line.encode("utf-8"))
            self.serial_conn.flush()
        except SerialException as exc:
            self.get_logger().error(f"ESP32 Serial write error: {exc}")
            self._close_serial()

    def _close_serial(self) -> None:
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

    def valve_cmd_callback(self, msg: Bool) -> None:
        self.valve_open = bool(msg.data)
        self._send_serial_json({"valve": 1 if self.valve_open else 0})

    def tare_cmd_callback(self, msg: Bool) -> None:
        if msg.data:
            self._send_serial_json({"tare": 1})

    def serial_poll_loop(self) -> None:
        now = self.get_clock().now()

        if self.serial_conn is None or not self.serial_conn.is_open:
            elapsed = (now - self.last_reconnect_attempt).nanoseconds / 1e9
            if elapsed >= self.reconnect_sec:
                self.last_reconnect_attempt = now
                self._try_connect_serial()
            return

        try:
            while self.serial_conn.in_waiting > 0:
                raw_line = self.serial_conn.readline().decode(
                    "utf-8", errors="ignore"
                ).strip()
                if not raw_line or not raw_line.startswith("{"):
                    continue
                self._handle_incoming_json(raw_line, now)
        except SerialException as exc:
            self.get_logger().error(f"ESP32 Serial read error: {exc}")
            self._close_serial()

    def _handle_incoming_json(self, raw_line: str, now) -> None:
        try:
            data = json.loads(raw_line)
        except json.JSONDecodeError:
            return

        # Extract filtered values
        filt_packet = {
            "weight": float(data.get("weight", 0.0)),
            "ph": float(data.get("ph", 7.0)),
            "do": float(data.get("do", 6.5)),
            "turbidity": float(data.get("turbidity", 10.0)),
            "temp": float(data.get("temp", 25.0)),
            "valve": int(data.get("valve", 1 if self.valve_open else 0)),
        }

        # Extract raw values (fallback to filtered if firmware omits raw_* keys)
        raw_packet = {
            "weight": float(data.get("raw_weight", filt_packet["weight"])),
            "ph": float(data.get("raw_ph", filt_packet["ph"])),
            "do": float(data.get("raw_do", filt_packet["do"])),
            "turbidity": float(data.get("raw_turbidity", filt_packet["turbidity"])),
            "temp": float(data.get("raw_temp", filt_packet["temp"])),
            "valve": filt_packet["valve"],
        }

        filt_msg = String()
        filt_msg.data = json.dumps(filt_packet)
        self.filt_json_pub.publish(filt_msg)

        raw_msg = String()
        raw_msg.data = json.dumps(raw_packet)
        self.raw_json_pub.publish(raw_msg)

        for key in ["weight", "ph", "do", "turbidity", "temp"]:
            f_raw = Float32()
            f_raw.data = float(raw_packet[key])
            self.raw_float_pubs[key].publish(f_raw)

            f_filt = Float32()
            f_filt.data = float(filt_packet[key])
            self.filt_float_pubs[key].publish(f_filt)

        js_msg = JointState()
        js_msg.header.stamp = now.to_msg()
        js_msg.name = ["feeder_valve_joint"]
        js_msg.position = [0.05 if filt_packet["valve"] else 0.0]
        js_msg.velocity = [0.0]
        js_msg.effort = [0.0]
        self.joint_state_pub.publish(js_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ESP32SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
