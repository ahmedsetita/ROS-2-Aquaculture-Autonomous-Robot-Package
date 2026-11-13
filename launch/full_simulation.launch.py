#!/usr/bin/python3
"""
One-Click Full Simulation, RViz2 & Autonomous Water Control Launch File
=======================================================================
Launches:
  1. Gazebo Classic with `worlds/fish_farm.world`
  2. `robot_state_publisher` + `static_transform_publisher` (map -> odom)
  3. `spawn_entity.py` (via system `/usr/bin/python3`) to spawn `aqua_robot` in the water
  4. `rviz2` pre-configured with `rviz/aqua_robot.rviz` (3D Robot, Pond Map, Fish Schools, Paths, Camera HUD)
  5. `pond_rviz_visualizer_node.py` (2D OccupancyGrid `/map` + 3D `/pond/markers` & trajectories)
  6. `esp32_sim_node.py` (or `esp32_serial_bridge_node.py` if `use_real_hardware:=true`)
  7. `fish_vision_node.py` (OpenCV Fish Feeding Activity & Water Status HUD)
  8. `autonomous_brain_node.py` (4-State FSM & Closed-Loop PID Autonomous Pond Patrol)
  9. `aqua_control_gui.py` (Live Aquaculture Camera & ESP32 Telemetry Command Center GUI)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _get_clean_system_path() -> str:
    """Removes Conda/Miniconda from PATH so ROS 2 Humble executables use /usr/bin/python3."""
    raw_parts = os.environ.get("PATH", "").split(os.pathsep)
    filtered = [
        p for p in raw_parts if "conda" not in p.lower() and p.strip() != ""
    ]
    for sys_dir in ["/usr/bin", "/bin", "/usr/local/bin", "/opt/ros/humble/bin"]:
        if sys_dir not in filtered:
            filtered.insert(0, sys_dir)
    return os.pathsep.join(filtered)


def generate_launch_description() -> LaunchDescription:
    clean_path = _get_clean_system_path()
    os.environ["PATH"] = clean_path

    pkg_share = get_package_share_directory("aquaculture_autonomous_robot")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    xacro_file = os.path.join(pkg_share, "urdf", "aqua_robot.urdf.xacro")
    world_file = os.path.join(pkg_share, "worlds", "fish_farm.world")
    rviz_config_file = os.path.join(pkg_share, "rviz", "aqua_robot.rviz")

    # Process Xacro into URDF XML string and save a compiled file for guaranteed Gazebo spawning
    robot_description_doc = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_doc.toxml()
    compiled_urdf_path = "/tmp/aqua_robot_compiled.urdf"
    with open(compiled_urdf_path, "w", encoding="utf-8") as f_urdf:
        f_urdf.write(robot_description_xml)

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock if true",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo GUI client (gzclient) if true",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 3D Aquaculture Pond & Navigation view",
    )
    control_gui_arg = DeclareLaunchArgument(
        "control_gui",
        default_value="true",
        description="Launch Live Aquaculture Camera & ESP32 Telemetry Control Center GUI",
    )
    use_real_hw_arg = DeclareLaunchArgument(
        "use_real_hardware",
        default_value="false",
        description="If true, runs esp32_serial_bridge_node.py instead of esp32_sim_node.py",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="USB Serial device path for real ESP32 hardware mode",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    control_gui = LaunchConfiguration("control_gui")
    use_real_hardware = LaunchConfiguration("use_real_hardware")
    serial_port = LaunchConfiguration("serial_port")

    # 1. Gazebo Simulation Server + Client with fish_farm.world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "gui": gui,
            "verbose": "false",
        }.items(),
    )

    # 2. Static Transform map -> odom (for RViz2 Pond Map & 3D Markers)
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 3. Robot State Publisher (URDF -> TF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_xml,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # 4. Spawn `aqua_robot` into Gazebo using explicit /usr/bin/python3 + compiled URDF file
    spawn_aqua_robot_cmd = ExecuteProcess(
        cmd=[
            "/usr/bin/python3",
            "/opt/ros/humble/lib/gazebo_ros/spawn_entity.py",
            "-entity",
            "aqua_robot",
            "-file",
            compiled_urdf_path,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.05",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # 5. Pond 3D RViz Visualizer Node (/map, /pond/markers, /pond/patrol_path, /pond/robot_trail)
    pond_visualizer_node = Node(
        package="aquaculture_autonomous_robot",
        executable="pond_rviz_visualizer_node.py",
        name="pond_rviz_visualizer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 6. RViz2 with pre-configured aquaculture view
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    # 7A. ESP32 Sensor Suite & Noise Filtering Simulation Node (Default Simulation Mode)
    esp32_sim_node = Node(
        package="aquaculture_autonomous_robot",
        executable="esp32_sim_node.py",
        name="esp32_sim_node",
        output="screen",
        condition=UnlessCondition(use_real_hardware),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "initial_feed_weight_g": 5000.0,
                "dispense_rate_g_per_sec": 50.0,
                "publish_rate_hz": 20.0,
                "spike_probability": 0.08,
                "auto_refill_at_dock": True,
            }
        ],
    )

    # 7B. Real ESP32 USB Serial Bridge Node (Active when use_real_hardware:=true)
    esp32_serial_bridge_node = Node(
        package="aquaculture_autonomous_robot",
        executable="esp32_serial_bridge_node.py",
        name="esp32_serial_bridge_node",
        output="screen",
        condition=IfCondition(use_real_hardware),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "serial_port": serial_port,
                "baud_rate": 115200,
            }
        ],
    )

    # 8. Computer Vision Fish Activity & Depleted Feed Node
    fish_vision_node = Node(
        package="aquaculture_autonomous_robot",
        executable="fish_vision_node.py",
        name="fish_vision_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 9. Autonomous Brain & Closed-Loop PID FSM Controller Node
    autonomous_brain_node = Node(
        package="aquaculture_autonomous_robot",
        executable="autonomous_brain_node.py",
        name="autonomous_brain_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_feed_dose_g": 250.0,
                "low_feed_threshold_g": 300.0,
                "feeding_demand_threshold_pct": 35.0,
                "min_safe_do_mg_l": 4.0,
                "min_safe_ph": 6.5,
                "max_safe_ph": 8.5,
                "max_safe_temp_c": 32.0,
                "inspection_duration_sec": 2.5,
                "waypoint_tolerance_m": 0.25,
            }
        ],
    )

    # 10. Live Aquaculture Control Center & Telemetry Dashboard GUI
    aqua_control_gui_node = Node(
        package="aquaculture_autonomous_robot",
        executable="aqua_control_gui.py",
        name="aqua_control_gui",
        output="screen",
        condition=IfCondition(control_gui),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="PATH", value=clean_path),
            use_sim_time_arg,
            gui_arg,
            rviz_arg,
            control_gui_arg,
            use_real_hw_arg,
            serial_port_arg,
            gazebo_launch,
            map_to_odom_tf,
            robot_state_publisher_node,
            spawn_aqua_robot_cmd,
            pond_visualizer_node,
            rviz2_node,
            esp32_sim_node,
            esp32_serial_bridge_node,
            fish_vision_node,
            autonomous_brain_node,
            aqua_control_gui_node,
        ]
    )
