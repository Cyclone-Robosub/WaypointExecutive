#include "WaypointExecutive.hpp"
#include <chrono>
#include <fstream>
#include <memory>

void WaypointExecutive::SetupROS()
{
  callbackINT = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto INTOPTIONS = rclcpp::SubscriptionOptions();
  INTOPTIONS.callback_group = callbackINT;

  WaypointPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "waypoint_topic", 10);
  SOCINTSub = this->create_subscription<std_msgs::msg::Bool>(
      "SOCIntTopic", 10,
      std::bind(&WaypointExecutive::SOCIntCallback, this,
                std::placeholders::_1),
      INTOPTIONS);
  CurrentTaskPub =
      this->create_publisher<std_msgs::msg::String>("CurrentTaskTopic", 10);
  // VisionSub =
  //   this->create_subscription<std_msgs::msg::String>("VisionTopic", 10); //INTOPTIONS
  Manipulation_Publisher =
      this->create_publisher<std_msgs::msg::Int64>("manipulationCommand", 10);
}

// Need to think about a better StopWorking mechanism.
int WaypointExecutive::Controller()
{
  MissionQueue.parseJSONForMission();
  while (!MissionQueue.allTasksComplete())
  {
    getNewMissionTask();
    while (!CurrentTask.steps_queue.empty())
    {
      getNewMissionStep();
      SendCurrentWaypoint();
      while (!isCurrentStepCompleted())
      {
        // if (CurrentTask.isInterruptable) { //Think about Hard vs Soft INT
        CheckINTofStep(); // Potentially Conditional Unresponsive Function
                          // (should not be as development continues)
        if (!Current_Interrupts.empty())
        {
          ServiceINTofStep();
          if(StopWorking){
            return 0;
          }
        }
        //}
      }
    }
  }
  EndReport();
  return 1;
}
///@brief O(1) Algo and no conditional waiting.
void WaypointExecutive::SendCurrentWaypoint()
{
  auto message = std_msgs::msg::Float32MultiArray();
  message.data.resize(6);
  if (CurrentWaypointPtr != nullptr)
  {
    for (int i = 0; i < 6; ++i)
    {
      message.data[i] = (*CurrentWaypointPtr)[i];
    }
    WaypointPublisher->publish(message);
    std::cout << "sent" << std::endl;
  }
  // start the timer
}
///@brief O(1) and no conditional waiting. Returns True if the task should still
/// run.
bool WaypointExecutive::isCurrentStepCompleted()
{
  if (!MetPositionandTimeReq())
  {
    return false;
  }

  // use the vision condition option to check if done?

  if (CurrentStep.ManipulationCodeandStatus.has_value())
  {
    if (!CurrentStep.ManipulationCodeandStatus.value().second)
    {
      return false;
    }
  }
  // else
  return true;
}
///@brief Conditional waiting for some and for now. Pushes INT instance to the
/// queue of Current Queue.
void WaypointExecutive::CheckINTofStep()
{
  Interrupts generateINT;
  // Make this better if possible.
  bool didInterruptHappen = false;
  // check vision if needed -> Manipulation Tasks can be coded apart and along
  // side this vision requriment along with position and altitude.
  if (CurrentStep.VisionCommand.has_value())
  {
    const auto cmd = CurrentStep.VisionCommand.value();

    if (cmd == "BINS_SPOTTED")
    {
      // Handle BINS_SPOTTED
      // e.g., reset flag, set interrupt
      generateINT.BINS_SPOTTED = true;
      didInterruptHappen = true;
    }
    else if (cmd == "DROP_INTO_BINS")
    {
      // Handle DROP_INTO_BINS
      // e.g., check altitude, reset flag, set manip code
      generateINT.TriggerManipSendCode = true;
      didInterruptHappen = true;
    }
  }
  // listen to the vision topic;
  if (CurrentStep.ManipulationCodeandStatus.has_value())
  {
    generateINT.TriggerManipSendCode = true;
    didInterruptHappen = true;
  }
  // check battery
  if (isSOCINT.has_value())
  {
    isSOCINT.reset();
    generateINT.SOCDANGER = true;
    didInterruptHappen = true;
  }
  if (didInterruptHappen)
  {
    Current_Interrupts.push(generateINT);
  }
}

///@brief O(1) Algo and no conditional waiting. Service the INT. Clear the
/// Current Interrupt at the end.
void WaypointExecutive::ServiceINTofStep()
{
  Interrupts ServiceINT = Current_Interrupts.front();
  std::cout<< "service" << std::endl;
  if (ServiceINT.SOCDANGER)
  {
    // Battery WayPoint
    CurrentStep = Step();
    // CurrentStep.WaypointPointer = std::make_shared<waypointPtr>(); //
    // Creating a new waypoint.
    SendCurrentWaypoint();
    EndReport(ServiceINT);
  }
  if (ServiceINT.BINS_SPOTTED)
  {
    // Get Waypoint or coordinate from Vision.
    SendCurrentWaypoint();
  }
  if (ServiceINT.TriggerManipSendCode)
  {
    ManipulationStep(CurrentStep.ManipulationCodeandStatus.value().first);
    CurrentStep.ManipulationCodeandStatus.value().second = true;
  }
  Current_Interrupts.pop();
}

void WaypointExecutive::getNewMissionTask()
{
  CurrentTask = MissionQueue.popNextTask();
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = CurrentTask.name;
  CurrentTaskPub->publish(std::move(msg));
  std::cout << "new mission task " << std::endl;
}

///@brief O(1) Algo and no conditional waiting. Has authority of changing
/// CurrentTask.
void WaypointExecutive::getNewMissionStep()
{
  // fetch or predetermined waypoints.
  // Predetermined -> Waypoint Objects?
  CurrentStep = CurrentTask.steps_queue.front();
  CurrentTask.steps_queue.pop();
  if (CurrentStep.WaypointPointer != nullptr)
  {
    CurrentWaypointPtr = CurrentStep.WaypointPointer;
  }
  std::cout << "new mission step " << std::endl;
}

void WaypointExecutive::SOCIntCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
  isSOCINT = msg->data;
}

void WaypointExecutive::ManipulationStep(int code)
{
  // Send Manipulation Code over Publisher.
  auto manipulation_msg = std::make_unique<std_msgs::msg::Int64>();
  manipulation_msg->data = code;
  Manipulation_Publisher->publish(std::move(manipulation_msg));
}

///@brief O(1) Algo and no conditional waiting.
bool WaypointExecutive::MetPositionandTimeReq()
{
  // check position var (include tolerance) with CurrentWaypointPtr
  // if position not met -> if(optional) CurrentStep.StopTimer(); //Missed after
  // reaching it. then return false; else if position met {
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value())
  {
    if (!CurrentStep.isTimerOn)
    {
      CurrentStep.StartTimer();
      std::cout << CurrentStep.HoldWaypTime_TimeElapsed.value().first <<std::endl;
    }
  }
  //}

  // Time Req
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value())
  {
    CurrentStep.CalcTimer();
    if (CurrentStep.HoldWaypTime_TimeElapsed.value().first >
        CurrentStep.HoldWaypTime_TimeElapsed.value().second)
    {
      return false;
    }
    else
    {
      CurrentStep.StopTimer();
    }
  }
  std::cout << "met TIME and pos" << std::endl;
  return true;
}

/// @brief: Will Exit itself after creating Report
void WaypointExecutive::EndReport(Interrupts interrupt)
{
  std::ofstream ReportFile;
  ReportFile.open("../../End_Report.txt");
  ReportFile << "___________START OF REPORT__________" << std::endl;
  ReportFile << "Reason for Report : ";
  if (MissionQueue.allTasksComplete())
  {
    ReportFile << "All Tasks are Completed." << std::endl;
  }
  if (interrupt.SOCDANGER)
  {
    ReportFile << "State of Charge was low. Check Logs of SOC" << std::endl;
  }
  ReportFile << "___________END OF REPORT ___________" << std::endl;
  ReportFile.close();
  StopWorking = true;
}