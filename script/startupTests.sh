source /opt/ros/jazzy/setup.bash
cd ../
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
cd build/waypoint_executive

./waypoint_test 