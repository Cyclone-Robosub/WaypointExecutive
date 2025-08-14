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
include_directories(lib/json-schema-validator)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME}
  src/mainWaypoint.cpp
  src/WaypointExecutive.cpp
  src/Task.cpp
  JSON_Parser/MissionAnalyser.cpp
  crs_common/position/position.cpp
)
# TODO
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json_schema_validator)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_waypoint.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test PRIVATE nlohmann_json_schema_validator)
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
    geometry_msgs
  )

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_add_gtest(waypoint_test
  test/test_waypoint.cpp
  src/WaypointExecutive.cpp
  src/Task.cpp
  JSON_Parser/MissionAnalyser.cpp
  crs_common/position/position.cpp
  )

# target_link_libraries(waypoint_executive PRIVATE nlohmann_json_schema_validator)
# target_link_libraries(waypoint_test PRIVATE nlohmann_json_schema_validator)
ament_package()
