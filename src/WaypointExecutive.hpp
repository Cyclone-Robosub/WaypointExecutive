#include "JSON_Parser/MissionAnalyser.hpp"
#include "Task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstddef>
#include "../../crs_common/position/position.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <utility>

/*
Notes:
  INT stands for interrupts.
  Theoretically, interrupts that occur while another interrupt is being serviced
or checked could be never attended to as the order in which the ServiceINTofTask
or services all the interrupts specifies priority. This could be solved by a
queue of Interrupts, but then there is a loss of actual priority. Thus a data
structure of the interrupts, perferably a max heap, that sorts itself is a
solution. Multithreading is another solution.


  Vision and Manipulation INT should be able to work together and apart. There
will probably be functions, but understanding the outputs and inputs from Vision
and Manipulation should solve this implementation problem.


*/
///@TODO:
// Tasks to Complete
// Interrupt Handling
//    Controller vs Task Interrupts Handling and Implementaion Location -> Done
//    Then Finish up CheckINTofStep and ServiceINTofStep -> Done
//    Upload Design Image -> Not Needed.
// ROS2
//     Vision and Manipulation and Position ROS Topics -> Waiting for External
//     (Manipulation Done) Custom MSG for Float or just get_x ... -> Done
// Manipulation Code Checkup with ManipulationTask() updated. -> Needs Review
// StopWorking mechanism in Controller -> Done
// Timer Implementation Location Change to Task.hpp -> Done
// EndReport Details and Information Handling -> Done
//  Make the CurrentWaypointPtr a parameter for SendCurrentWaypoint.

struct Interrupts {
  bool SOCDANGER{false};
  bool BINS_SPOTTED{false};
  bool DROP_INTO_BINS{false}; // READY_TO_DROP_INTO_BINS
  bool TriggerManipSendCode{false};
};

class WaypointExecutive : public rclcpp::Node {
public:
  WaypointExecutive()
      : MissionQueue("../../WaypointDecision/JSON_Parser/MissionPath.JSON"),
        Node("WaypointExecutiveNode") {
    SetupROS();
  }
int Controller();
private:
  void SetupROS();
  void SendCurrentWaypoint();
  void getNewMissionStep();
  void getNewMissionTask();
  bool isCurrentStepCompleted();
  void ManipulationStep(int code);
  void CheckINTofStep();
  void ServiceINTofStep();
  // publisher of CurrentWaypointPtr topic.
  std::optional<bool> isSOCINT;
  waypointPtr CurrentWaypointPtr;
  Task CurrentTask;
  Step CurrentStep;
  std::queue<Interrupts>
      Current_Interrupts; // Review the priority queue < opreator between two
                          // elements with void pointers.
  // Need to resolve the Time Elapsed and Counting.

  // callback ROS2 functions
  rclcpp::CallbackGroup::SharedPtr callbackINT;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      WaypointPublisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr CurrentTaskPub;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr Manipulation_Publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr VisionSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr SOCINTSub;
  void SOCIntCallback(const std_msgs::msg::Bool::SharedPtr msg);
  // Make This Pair or Class to save the interrupt details.
  bool StopWorking{false};
  MissionAnalyser MissionQueue;
  bool MetPositionandTimeReq();
  void EndReport(Interrupts interrupt = Interrupts());
};
