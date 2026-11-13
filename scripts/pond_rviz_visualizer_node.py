#!/usr/bin/python3
"""
Aquaculture Pond 3D RViz2 Map, Zones, Fish & Trajectory Visualizer Node
=======================================================================
Publishes rich 3D visualization topics so RViz2 shows the entire aquaculture
water environment, feeding zones, swimming fish schools, robot path, and live
floating telemetry HUD (just like the AMR car project, adapted for water):
  - /map (nav_msgs/OccupancyGrid) : 2D pond map with walls & dock
  - /pond/markers (visualization_msgs/MarkerArray) :
      * Semi-transparent 3D water basin
      * 4 colored 3D feeding zone pads + labels
      * Animated 3D fish schools swimming in each zone
      * Live floating telemetry text above the robot
      * Golden feed pellets pouring from the bottom red valve when feeding
  - /pond/patrol_path (nav_msgs/Path) : Planned autonomous patrol loop
  - /pond/robot_trail (nav_msgs/Path) : Actual historical trajectory of the robot
"""

import json
import math
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


class PondRvizVisualizerNode(Node):
    """Generates OccupancyGrid map, 3D pond markers, fish schools, and paths for RViz2."""

    def __init__(self) -> None:
        super().__init__("pond_rviz_visualizer_node")

        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.valve_open: bool = False
        self.fsm_state: str = "PATROL"
        self.feeding_demand: float = 0.0
        self.sensors: Dict[str, float] = {
            "weight": 5000.0,
            "ph": 7.50,
            "do": 6.50,
            "turbidity": 12.0,
            "temp": 25.0,
        }
        self.start_time = self.get_clock().now()

        # Zones: (id, label, x, y, r, g, b, fish_count)
        self.zones: List[Tuple[int, str, float, float, float, float, float, int]] = [
            (1, "ZONE 1: High Fish Aggregation (Depleted Feed)", 3.5, 3.5, 0.0, 0.85, 0.95, 10),
            (2, "ZONE 2: Normal Balanced Zone", -3.5, 3.5, 0.15, 0.80, 0.45, 5),
            (3, "ZONE 3: CRITICAL HYPOXIA & ACIDIC pH", -3.5, -3.5, 0.90, 0.15, 0.35, 2),
            (4, "ZONE 4: Active School & Sediment Agitation", 3.5, -3.5, 0.10, 0.70, 0.95, 8),
        ]

        # Subscribers
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Bool, "/feeder_valve_cmd", self.valve_cb, 10)
        self.create_subscription(String, "/brain/state", self.state_cb, 10)
        self.create_subscription(String, "/esp32/sensors_filtered", self.sensors_cb, 10)
        self.create_subscription(Float32, "/vision/feeding_demand", self.demand_cb, 10)

        # Transient local QoS for /map so RViz2 always receives it
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.markers_pub = self.create_publisher(MarkerArray, "/pond/markers", 10)
        self.patrol_path_pub = self.create_publisher(Path, "/pond/patrol_path", 10)
        self.trail_pub = self.create_publisher(Path, "/pond/robot_trail", 10)

        self.robot_trail = Path()
        self.robot_trail.header.frame_id = "map"

        self.occupancy_grid_msg = self._build_pond_occupancy_grid()
        self.map_pub.publish(self.occupancy_grid_msg)

        self.create_timer(0.1, self.publish_visualizations)
        self.create_timer(2.0, self.republish_map)

        self.get_logger().info("Pond RViz2 3D Visualizer Node initialized (/map, /pond/markers, /pond/patrol_path).")

    def odom_cb(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Append to robot trail if moved > 0.08m
        if not self.robot_trail.poses:
             add_pose = True
        else:
            last_p = self.robot_trail.poses[-1].pose.position
            add_pose = math.hypot(self.robot_x - last_p.x, self.robot_y - last_p.y) > 0.08

        if add_pose:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose = msg.pose.pose
            ps.pose.position.z = 0.05
            self.robot_trail.poses.append(ps)
            if len(self.robot_trail.poses) > 400:
                self.robot_trail.poses.pop(0)

    def valve_cb(self, msg: Bool) -> None:
        self.valve_open = bool(msg.data)

    def state_cb(self, msg: String) -> None:
        self.fsm_state = msg.data

    def demand_cb(self, msg: Float32) -> None:
        self.feeding_demand = float(msg.data)

    def sensors_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            for k in ["weight", "ph", "do", "turbidity", "temp"]:
                if k in data:
                    self.sensors[k] = float(data[k])
        except Exception:
            pass

    def _build_pond_occupancy_grid(self) -> OccupancyGrid:
        """Creates a 16m x 16m OccupancyGrid at 0.05m/px resolution representing the fish pond."""
        res = 0.05
        width = 320   # 16.0m
        height = 320  # 16.0m
        origin_x = -8.0
        origin_y = -8.0

        grid = np.zeros((height, width), dtype=np.int8)

        # Outer surroundings outside 14m x 14m pond (-1 unknown)
        for r in range(height):
            y = origin_y + (r + 0.5) * res
            for c in range(width):
                x = origin_x + (c + 0.5) * res
                if abs(x) > 7.2 or abs(y) > 7.2:
                    grid[r, c] = -1
                elif abs(x) >= 6.8 or abs(y) >= 6.8:
                    # Pond concrete bounding walls (100 occupied)
                    grid[r, c] = 100
                else:
                    # Navigable water surface (0 free)
                    grid[r, c] = 0

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = MapMetaData()
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        return msg

    def republish_map(self) -> None:
        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.occupancy_grid_msg)

    def publish_visualizations(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        t_sec = (now - self.start_time).nanoseconds / 1e9

        ma = MarkerArray()
        marker_id = 0

        # 1. Translucent Cyan Water Surface Plane in RViz2
        water_marker = Marker()
        water_marker.header.frame_id = "map"
        water_marker.header.stamp = stamp
        water_marker.ns = "pond_water"
        water_marker.id = marker_id
        marker_id += 1
        water_marker.type = Marker.CUBE
        water_marker.action = Marker.ADD
        water_marker.pose.position.x = 0.0
        water_marker.pose.position.y = 0.0
        water_marker.pose.position.z = 0.01
        water_marker.pose.orientation.w = 1.0
        water_marker.scale.x = 13.6
        water_marker.scale.y = 13.6
        water_marker.scale.z = 0.02
        water_marker.color.r = 0.05
        water_marker.color.g = 0.55
        water_marker.color.b = 0.82
        water_marker.color.a = 0.28
        ma.markers.append(water_marker)

        # 2. Home / Refill Dock Pad at (0, 0)
        dock_marker = Marker()
        dock_marker.header.frame_id = "map"
        dock_marker.header.stamp = stamp
        dock_marker.ns = "pond_zones"
        dock_marker.id = marker_id
        marker_id += 1
        dock_marker.type = Marker.CYLINDER
        dock_marker.action = Marker.ADD
        dock_marker.pose.position.x = 0.0
        dock_marker.pose.position.y = 0.0
        dock_marker.pose.position.z = 0.025
        dock_marker.pose.orientation.w = 1.0
        dock_marker.scale.x = 1.4
        dock_marker.scale.y = 1.4
        dock_marker.scale.z = 0.02
        dock_marker.color.r = 0.20
        dock_marker.color.g = 0.50
        dock_marker.color.b = 0.95
        dock_marker.color.a = 0.65
        ma.markers.append(dock_marker)

        dock_txt = Marker()
        dock_txt.header.frame_id = "map"
        dock_txt.header.stamp = stamp
        dock_txt.ns = "zone_labels"
        dock_txt.id = marker_id
        marker_id += 1
        dock_txt.type = Marker.TEXT_VIEW_FACING
        dock_txt.action = Marker.ADD
        dock_txt.pose.position.x = 0.0
        dock_txt.pose.position.y = 0.0
        dock_txt.pose.position.z = 0.45
        dock_txt.scale.z = 0.28
        dock_txt.color.r = 1.0
        dock_txt.color.g = 1.0
        dock_txt.color.b = 1.0
        dock_txt.color.a = 0.95
        dock_txt.text = "HOME DOCK & FEED REFILL (0, 0)"
        ma.markers.append(dock_txt)

        # 3. 4 Feeding Zones + Animated Swimming Fish Schools
        for zid, zlabel, zx, zy, cr, cg, cb, num_fish in self.zones:
            z_disc = Marker()
            z_disc.header.frame_id = "map"
            z_disc.header.stamp = stamp
            z_disc.ns = "pond_zones"
            z_disc.id = marker_id
            marker_id += 1
            z_disc.type = Marker.CYLINDER
            z_disc.action = Marker.ADD
            z_disc.pose.position.x = zx
            z_disc.pose.position.y = zy
            z_disc.pose.position.z = 0.02
            z_disc.pose.orientation.w = 1.0
            z_disc.scale.x = 2.7
            z_disc.scale.y = 2.7
            z_disc.scale.z = 0.02
            z_disc.color.r = cr
            z_disc.color.g = cg
            z_disc.color.b = cb
            z_disc.color.a = 0.55
            ma.markers.append(z_disc)

            z_txt = Marker()
            z_txt.header.frame_id = "map"
            z_txt.header.stamp = stamp
            z_txt.ns = "zone_labels"
            z_txt.id = marker_id
            marker_id += 1
            z_txt.type = Marker.TEXT_VIEW_FACING
            z_txt.action = Marker.ADD
            z_txt.pose.position.x = zx
            z_txt.pose.position.y = zy
            z_txt.pose.position.z = 0.70
            z_txt.scale.z = 0.26
            z_txt.color.r = 1.0
            z_txt.color.g = 1.0 if zid != 3 else 0.4
            z_txt.color.b = 1.0 if zid != 3 else 0.4
            z_txt.color.a = 1.0
            z_txt.text = zlabel
            ma.markers.append(z_txt)

            # Animated swimming fish markers in each zone
            for f_idx in range(num_fish):
                orbit_r = 0.25 + 0.10 * (f_idx % 4) + 0.15 * (f_idx // 4)
                speed = 0.9 + 0.15 * (f_idx % 3)
                angle = t_sec * speed + f_idx * (2.0 * math.pi / max(1, num_fish))
                fx = zx + orbit_r * math.cos(angle)
                fy = zy + orbit_r * math.sin(angle)
                tangent_yaw = angle + math.pi / 2.0

                fish_m = Marker()
                fish_m.header.frame_id = "map"
                fish_m.header.stamp = stamp
                fish_m.ns = "swimming_fish"
                fish_m.id = marker_id
                marker_id += 1
                fish_m.type = Marker.ARROW
                fish_m.action = Marker.ADD
                fish_m.pose.position.x = fx
                fish_m.pose.position.y = fy
                fish_m.pose.position.z = 0.04
                fish_m.pose.orientation.z = math.sin(tangent_yaw * 0.5)
                fish_m.pose.orientation.w = math.cos(tangent_yaw * 0.5)
                fish_m.scale.x = 0.32
                fish_m.scale.y = 0.11
                fish_m.scale.z = 0.08
                fish_m.color.r = 1.0
                fish_m.color.g = 0.45
                fish_m.color.b = 0.05
                fish_m.color.a = 0.95
                ma.markers.append(fish_m)

        # 4. Floating Live Robot Status HUD in RViz2 above aqua_robot
        hud_marker = Marker()
        hud_marker.header.frame_id = "map"
        hud_marker.header.stamp = stamp
        hud_marker.ns = "robot_hud"
        hud_marker.id = marker_id
        marker_id += 1
        hud_marker.type = Marker.TEXT_VIEW_FACING
        hud_marker.action = Marker.ADD
        hud_marker.pose.position.x = self.robot_x
        hud_marker.pose.position.y = self.robot_y
        hud_marker.pose.position.z = 1.25
        hud_marker.scale.z = 0.24
        hud_marker.color.r = 0.2 if not self.valve_open else 1.0
        hud_marker.color.g = 1.0
        hud_marker.color.b = 0.4 if not self.valve_open else 0.1
        hud_marker.color.a = 1.0
        hud_marker.text = (
            f"[{self.fsm_state}] Hopper: {self.sensors['weight']:.0f}g | "
            f"Valve: {'OPEN (FEEDING!)' if self.valve_open else 'CLOSED'}\n"
            f"pH: {self.sensors['ph']:.2f} | DO: {self.sensors['do']:.2f}mg/L | "
            f"Temp: {self.sensors['temp']:.1f}C | Demand: {self.feeding_demand:.0f}%"
        )
        ma.markers.append(hud_marker)

        # 5. Active Feed Pellet Stream when Bottom Red Valve is OPEN
        pellet_marker = Marker()
        pellet_marker.header.frame_id = "map"
        pellet_marker.header.stamp = stamp
        pellet_marker.ns = "feed_pellets"
        pellet_marker.id = marker_id
        marker_id += 1
        pellet_marker.type = Marker.SPHERE_LIST
        pellet_marker.action = Marker.ADD
        pellet_marker.scale.x = 0.07
        pellet_marker.scale.y = 0.07
        pellet_marker.scale.z = 0.07
        pellet_marker.color.r = 1.0
        pellet_marker.color.g = 0.82
        pellet_marker.color.b = 0.10
        pellet_marker.color.a = 0.95 if self.valve_open else 0.0
        if self.valve_open:
            for i in range(12):
                p = Point()
                spread = 0.04 * (i % 4)
                ang = t_sec * 6.0 + i * 1.1
                p.x = self.robot_x + spread * math.cos(ang)
                p.y = self.robot_y + spread * math.sin(ang)
                p.z = max(0.02, 0.16 - 0.012 * ((i + int(t_sec * 20)) % 12))
                pellet_marker.points.append(p)
        else:
            p = Point(x=self.robot_x, y=self.robot_y, z=-1.0)
            pellet_marker.points.append(p)
        ma.markers.append(pellet_marker)

        self.markers_pub.publish(ma)

        # 6. Publish Planned Patrol Loop Path
        patrol_path = Path()
        patrol_path.header.frame_id = "map"
        patrol_path.header.stamp = stamp
        loop_pts = [
            (0.0, 0.0),
            (3.18, 3.18),
            (-3.18, 3.18),
            (-3.18, -3.18),
            (3.18, -3.18),
            (0.0, 0.0),
        ]
        for px, py in loop_pts:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = stamp
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = 0.04
            ps.pose.orientation.w = 1.0
            patrol_path.poses.append(ps)
        self.patrol_path_pub.publish(patrol_path)

        # 7. Publish Robot Historical Trail
        self.robot_trail.header.stamp = stamp
        self.trail_pub.publish(self.robot_trail)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PondRvizVisualizerNode()
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
