cmake_minimum_required(VERSION 3.8)
project(waypoint_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crs_ros2_interfaces REQUIRED)
# Add include directories
include_directories(src)
include_directories(JSON_Parser)

add_executable(waypoint_executive
  src/mainWaypoint.cpp
  src/WaypointExecutive.cpp
  src/Task.cpp
  JSON_Parser/MissionAnalyser.cpp
  ../crs_common/position/position.cpp
)

ament_target_dependencies(waypoint_executive
  rclcpp
  std_msgs
  geometry_msgs
  crs_ros2_interfaces
)

install(TARGETS
  waypoint_executive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
