#include "WaypointExecutive.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

// TODO:
// Parse input commands or YAML File.
// The Format could be Waypoint -> given seconds.
// In the way future, we could make a waypoint seconds generator function.
// Create a Executive Scheduler based on timestamps and then the waypoint.
// Think of a Data Structure for this.
// If, Static -> just read of the getline. (File Output Input Data Structure)
// My suggestion if dynamic, Linked List (if we want to reorganize order) vs
// vector(if we do not care about reorganize order).
// Run the Executive Scheduler to publish WayPoints onto the waypoint topic.
// The topic just to be clear will publish continuously the message that
// contains the waypoint at this current moment. Listen to Position Topic, do
// some simple reasoning to determine to move to next waypoint. Here we can
// include the Changing of State Features in which we have predetermined actions
// with waypoints attached to them. Executive Overload should be able to shut
// down if ExecutiveMainLoop does not have that privilege level.

// Pseudo code
/*
    (This can be refactored later)
    Get the file of waypoint commands.
    Go through the file using getline or YAML
    Make the linked list node as we go through it setting the waypoint and the
   time of seconds. Make a function (the location of the func depends on the
   data implementation structure) that has the time loop in William's timed pwm
   functionality.

    Make the publisher shared ptr for the Waypoint so that either a linked list
   or vector impelmentation works.
*/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<WaypointExecutive> Node = std::make_shared<WaypointExecutive>();
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(Node);
  std::cout << "ROS2 Waypoint running" << std::endl;
  std::jthread spin_ros([executor]()
                        { executor->spin(); });
  int ResultCode = Node->Controller();
  // This will end when it needs to ^.
  rclcpp::shutdown();
  return ResultCode;
}