cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_pf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2_ros)
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Main Node
add_executable(pure_pursuit_node src/pure_pursuit_pf_node.cpp)
ament_target_dependencies(pure_pursuit_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
)

# Waypoint Viz Node
add_executable(waypoint_viz_node src/waypoint_viz_node.cpp)
ament_target_dependencies(waypoint_viz_node 
  rclcpp std_msgs visualization_msgs
)

# Install Executables
install(TARGETS pure_pursuit_node waypoint_viz_node
  DESTINATION lib/${PROJECT_NAME})

# Install Launch and Config (THIS IS WHAT YOU WERE MISSING)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Python setup
#ament_python_install_package(${PROJECT_NAME})
#install(PROGRAMS scripts/pure_pursuit_node.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()