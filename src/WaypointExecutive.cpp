#include "WaypointExecutive.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <regex>

void WaypointExecutive::SetupROS() {
  callbackINT =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callbackPosition =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto INTOPTIONS = rclcpp::SubscriptionOptions();
  INTOPTIONS.callback_group = callbackINT;
  auto PositionOptions = rclcpp::SubscriptionOptions();
  PositionOptions.callback_group = callbackPosition;

  auto VisionOptions = rclcpp::SubscriptionOptions();
  VisionOptions.callback_group = callbackINT;

  WaypointPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "waypoint_topic", 10);
  SOCINTSub = this->create_subscription<std_msgs::msg::Bool>(
      "SOCIntTopic", 10,
      std::bind(&WaypointExecutive::SOCIntCallback, this,
                std::placeholders::_1),
      INTOPTIONS);
  PositionSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "position_topic", 10,
      std::bind(&WaypointExecutive::PositionCallback, this,
                std::placeholders::_1),
      PositionOptions);
  CurrentTaskPub =
      this->create_publisher<std_msgs::msg::String>("CurrentTaskTopic", 10);
  VisionSub = this->create_subscription<std_msgs::msg::String>(
      "detections", 10,
      std::bind(&WaypointExecutive::VisionDetector, this,
                std::placeholders::_1),
      INTOPTIONS);
  Manipulation_Publisher =
      this->create_publisher<std_msgs::msg::Int64>("manipulationCommand", 10);
}

// Need to think about a better StopWorking mechanism.
int WaypointExecutive::Controller() {
  MissionQueue.parseJSONForMission();
  while (!MissionQueue.allTasksComplete()) {
    getNewMissionTask();
    while (!CurrentTask.steps_queue.empty()) {
      getNewMissionStep();
      SendCurrentWaypoint();
      while (!isCurrentStepCompleted()) {
        // if (CurrentTask.isInterruptable) { //Think about Hard vs Soft INT
        CheckINTofStep();  // Potentially Conditional Unresponsive Function
                           // (should not be as development continues)
        if (!Current_Interrupts.empty()) {
          ServiceINTofStep();
          if (StopWorking) {
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
void WaypointExecutive::SendCurrentWaypoint() {
  auto message = std_msgs::msg::Float32MultiArray();
  message.data.resize(6);
  if (CurrentWaypointPtr != nullptr) {
    for (int i = 0; i < 6; ++i) {
      message.data[i] = (*CurrentWaypointPtr)[i];
    }
    WaypointPublisher->publish(message);
    std::cout << "sent" << std::endl;
  }
  // start the timer
}
///@brief O(1) and no conditional waiting. Returns True if the task should still
/// run.
bool WaypointExecutive::isCurrentStepCompleted() {
  if (!MetPositionandTimeReq()) {
    return false;
  }

  // use the vision condition option to check if done?

  if (CurrentStep.ManipulationCodeandStatus.has_value()) {
    if (!CurrentStep.ManipulationCodeandStatus.value().second) {
      return false;
    }
  }
  // else
  return true;
}
///@brief Worst Case : O(How many objects were seen during the step). Pushes INT
///instance to the
/// queue of Current Queue.
void WaypointExecutive::CheckINTofStep() {
  Interrupts generateINT;
  // Make this better if possible.
  bool didInterruptHappen = false;
  // check vision if needed -> Manipulation Tasks can be coded apart and along
  // side this vision requriment along with position and altitude.
  if (CurrentStep.VisionINTCommand.has_value()) {
    const auto cmd = CurrentStep.VisionINTCommand.value();
    if (cmd == "GATE") {
      if (DidWeSeeObject("Reef Shark")) {
        generateINT.REEF_SHARK = true;
        didInterruptHappen = true;
      }
    }
    if (cmd == "BINS_SPOTTED") {
      // Handle BINS_SPOTTED
      // e.g., reset flag, set interrupt
      // if(last_detected_class == "")
      generateINT.BINS_SPOTTED = true;
      didInterruptHappen = true;
    } else if (cmd == "DROP_INTO_BINS") {
      // Handle DROP_INTO_BINS
      // e.g., check altitude, reset flag, set manip code
      generateINT.TriggerManipSendCode = true;
      didInterruptHappen = true;
    }
  }
  // listen to the vision topic;
  if (CurrentStep.ManipulationCodeandStatus.has_value()) {
    generateINT.TriggerManipSendCode = true;
    didInterruptHappen = true;
  }
  // check battery
  if (isSOCINT.has_value()) {
    isSOCINT.reset();
    generateINT.SOCDANGER = true;
    didInterruptHappen = true;
  }
  if (didInterruptHappen) {
    Current_Interrupts.push(generateINT);
  }
}

bool WaypointExecutive::DidWeSeeObject(std::string key) {
  std::lock_guard<std::mutex> lock(VisionVectorMutex);
  for (unsigned int i = 0; i < Last_Detected_Objects_Vector.size(); i++) {
    if (key == Last_Detected_Objects_Vector[i]) {
      return true;
    }
  }
  return false;
}

///@brief O(1) Algo and no conditional waiting. Service the INT. Clear the
/// Current Interrupt at the end.
void WaypointExecutive::ServiceINTofStep() {
  Interrupts ServiceINT = Current_Interrupts.front();
  std::cout << "service" << std::endl;
  if (ServiceINT.SOCDANGER) {
    // Battery WayPoint
    CurrentStep = Step();
    // CurrentStep.WaypointPointer = std::make_shared<waypointPtr>(); //
    // Creating a new waypoint.
    SendCurrentWaypoint();
    EndReport(ServiceINT);
  }
  if (ServiceINT.BINS_SPOTTED) {
    // Get Waypoint or coordinate from Vision.
    SendCurrentWaypoint();
  }
  if (ServiceINT.TriggerManipSendCode) {
    ManipulationStep(CurrentStep.ManipulationCodeandStatus.value().first);
    CurrentStep.ManipulationCodeandStatus.value().second = true;
  }
  Current_Interrupts.pop();
}

void WaypointExecutive::getNewMissionTask() {
  CurrentTask = MissionQueue.popNextTask();
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = CurrentTask.name;
  CurrentTaskPub->publish(std::move(msg));
  std::cout << "new mission task " << CurrentTask.name << std::endl;
}

///@brief O(1) Algo and no conditional waiting. Has authority of changing
/// CurrentTask.
void WaypointExecutive::getNewMissionStep() {
  // fetch or predetermined waypoints.
  // Predetermined -> Waypoint Objects?
  Last_Detected_Objects_Vector.clear();
  CurrentStep = CurrentTask.steps_queue.front();
  CurrentTask.steps_queue.pop();
  if (CurrentStep.WaypointPointer != nullptr) {
    CurrentWaypointPtr = CurrentStep.WaypointPointer;
  }
  std::cout << "new mission step " << std::endl;
}

void WaypointExecutive::SOCIntCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  isSOCINT = msg->data;
}
void WaypointExecutive::VisionDetector(
    const std_msgs::msg::String::SharedPtr msg) {
  std::regex class_regex(R"(Class:\s*([^,]+))");
  std::smatch match;

  if (std::regex_search(msg->data, match, class_regex)) {
    std::string detected_class = match[1];  // store in variable
    RCLCPP_INFO(this->get_logger(), "Detected class: %s",
                detected_class.c_str());

    // you can now store it in a member variable for later use
    std::string last_detected_class = detected_class;
    std::lock_guard<std::mutex> lock(VisionVectorMutex);
    Last_Detected_Objects_Vector.push_back(last_detected_class);
  } else {
    RCLCPP_WARN(this->get_logger(), "No class found in message.");
  }
}
void WaypointExecutive::PositionCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  CurrentPositionPtr =
      std::make_shared<Position>(msg->data[0], msg->data[1], msg->data[2],
                                 msg->data[3], msg->data[4], msg->data[5]);
}
void WaypointExecutive::ManipulationStep(int code) {
  // Send Manipulation Code over Publisher.
  auto manipulation_msg = std::make_unique<std_msgs::msg::Int64>();
  manipulation_msg->data = code;
  Manipulation_Publisher->publish(std::move(manipulation_msg));
}

///@brief O(1) Algo and no conditional waiting.
bool WaypointExecutive::MetPositionandTimeReq() {
  // check position var (include tolerance) with CurrentWaypointPtr
  // Check that we have to go to a position and we are currently running.
  if (CurrentWaypointPtr && CurrentPositionPtr) {
    if (CurrentWaypointPtr->AreWeThereYet(CurrentPositionPtr)) {
      // Check to see if we need to start the timer. (Don't check for time req
      // yet.)
      if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
        if (!CurrentStep.isTimerOn) {
          CurrentStep.StartTimer();
          std::cout << CurrentStep.HoldWaypTime_TimeElapsed.value().first
                    << std::endl;
        }
      }
    }
    // We ran off course.
    else {
      CurrentStep.StopTimer();
      return false;
    }
  }
  // We don't have something for all the data we need. Or we just need to wait at our current position.
  else {
    if(CurrentWaypointPtr == nullptr){
    if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
      if (!CurrentStep.isTimerOn) {
        CurrentStep.StartTimer();
        std::cout << CurrentStep.HoldWaypTime_TimeElapsed.value().first
                  << std::endl;
      }
    } else {
      //The Robot has not started the current position.
      return false;
    }
    }
  }
  //}

  // Time Req after pos met or doesn't need to be met.
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
    CurrentStep.CalcTimer();
    if (CurrentStep.HoldWaypTime_TimeElapsed.value().first >
        CurrentStep.HoldWaypTime_TimeElapsed.value().second) {
      return false;
    } else {
      CurrentStep.StopTimer();
    }
  }
  std::cout << "met TIME and pos" << std::endl;
  return true;
}

/// @brief: Will Exit itself after creating Report
void WaypointExecutive::EndReport(Interrupts interrupt) {
  std::ofstream ReportFile;
  ReportFile.open("../../End_Report.txt");
  ReportFile << "___________START OF REPORT__________" << std::endl;
  ReportFile << "Reason for Report : ";
  if (MissionQueue.allTasksComplete()) {
    ReportFile << "All Tasks are Completed." << std::endl;
  }
  if (interrupt.SOCDANGER) {
    ReportFile << "State of Charge was low. Check Logs of SOC" << std::endl;
  }
  ReportFile << "___________END OF REPORT ___________" << std::endl;
  ReportFile.close();
  StopWorking = true;
}