cmake_minimum_required(VERSION 3.8)
project(waypoint_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(src)
include_directories(src/JSON_Parser)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)

add_executable(waypoint_executive
  src/mainWaypoint.cpp
  src/WaypointExecutive.cpp
  src/Task.cpp
  JSON_Parser/MissionAnalyser.cpp
  crs_common/position/position.cpp
)

ament_target_dependencies(waypoint_executive
  rclcpp
  std_msgs
  geometry_msgs
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

ament_add_gtest(waypoint_test
    test/test_waypoint.cpp
    src/mainWaypoint.cpp
  src/WaypointExecutive.cpp
  src/Task.cpp
  JSON_Parser/MissionAnalyser.cpp
  crs_common/position/position.cpp
  )
ament_target_dependencies(waypoint_test
    rclcpp
    std_msgs
  geometry_msgs
  )

ament_package()
