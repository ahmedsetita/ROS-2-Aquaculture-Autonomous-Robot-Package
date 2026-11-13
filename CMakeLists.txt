cmake_minimum_required(VERSION 3.8)
project(aquaculture_autonomous_robot)

# Ensure ROS 2 Humble system Python (/usr/bin/python3) is used even if Conda is active
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "System Python3 for ROS 2" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "System Python3 for ROS 2" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install Python module package
ament_python_install_package(${PROJECT_NAME})

# Install executable ROS 2 Python nodes
install(PROGRAMS
  scripts/esp32_sim_node.py
  scripts/fish_vision_node.py
  scripts/autonomous_brain_node.py
  scripts/esp32_serial_bridge_node.py
  scripts/pond_rviz_visualizer_node.py
  scripts/aqua_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF, Worlds, Launch, RViz, and Firmware directories
install(DIRECTORY
  urdf
  worlds
  launch
  rviz
  firmware
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
