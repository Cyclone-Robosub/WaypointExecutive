#include "WaypointExecutive.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <queue>



class MockPublisher {
public:
  template <typename T>
  void publish(const T& msg) {}
};

class MockPosition : public Position {
public:
    MockPosition(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0)
        : Position(x, y, z, roll, pitch, yaw) {}
};

class WaypointExecutiveTest : public ::testing::Test {
protected:
    void SetUp() override {
      //Added Setup for WaypointExecutiveTest
        rclcpp::init(0, nullptr);
  std::shared_ptr<WaypointExecutive> Node = std::make_shared<WaypointExecutive>();
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(Node);
  std::cout << "ROS2 Waypoint running" << std::endl;
  std::jthread spin_ros([executor]()
                        { executor->spin(); });
  int ResultCode = Node->Controller();
    }
    void TearDown() override {
        //Clean up after each test
        rclcpp::shutdown();
    }
};


TEST_F(WaypointExecutiveTest, ControllerTest) {
   
    WaypointExecutive executive;
    //No assignment needed here, as we use the constructor.
    
    Task task;
    Step step;
    step.WaypointPointer = std::make_shared<MockPosition>(1, 1, 1, 0, 0, 0);
    task.steps_queue.push(step);
    

  //  executive.MissionQueue = mockMissionQueue;
    int result = executive.Controller();
    ASSERT_EQ(result, 1);
}

TEST_F(WaypointExecutiveTest, SendCurrentWaypointTest) {
  WaypointExecutive executive;
  executive.CurrentWaypointPtr = std::make_shared<MockPosition>(1, 2, 3, 4, 5, 6);
  executive.SendCurrentWaypoint();
}

TEST_F(WaypointExecutiveTest, isCurrentStepCompletedTest) {
  WaypointExecutive executive;
  executive.CurrentWaypointPtr = std::make_shared<MockPosition>(1, 1, 1, 0, 0, 0);
  executive.CurrentPositionPtr = std::make_shared<MockPosition>(1, 1, 1, 0, 0, 0);
  ASSERT_TRUE(executive.isCurrentStepCompleted());

  executive.CurrentPositionPtr = std::make_shared<MockPosition>(2, 2, 2, 0, 0, 0);
  ASSERT_FALSE(executive.isCurrentStepCompleted());
}

TEST_F(WaypointExecutiveTest, getNewMissionTaskTest) {
   // MockMissionAnalyser mockMissionQueue;
    WaypointExecutive executive;
    Task task;
    task.name = "TestTask";
//   executive.MissionQueue = mockMissionQueue;
    executive.getNewMissionTask();
    ASSERT_EQ(executive.CurrentTask.name, "TestTask");
}


TEST_F(WaypointExecutiveTest, getNewMissionStepTest) {
    WaypointExecutive executive;
    executive.CurrentTask.steps_queue.push({});
    executive.getNewMissionStep();
}

TEST_F(WaypointExecutiveTest, ManipulationStepTest) {
    WaypointExecutive executive;
    executive.ManipulationStep(123);
}

TEST_F(WaypointExecutiveTest, MetPositionandTimeReqTest) {
    WaypointExecutive executive;
    executive.CurrentWaypointPtr = std::make_shared<MockPosition>(1, 1, 1, 0, 0, 0);
    executive.CurrentPositionPtr = std::make_shared<MockPosition>(1, 1, 1, 0, 0, 0);
    ASSERT_TRUE(executive.MetPositionandTimeReq());

    executive.CurrentPositionPtr = std::make_shared<MockPosition>(2, 2, 2, 0, 0, 0);
    ASSERT_FALSE(executive.MetPositionandTimeReq());
}

TEST_F(WaypointExecutiveTest, EndReportTest) {
    WaypointExecutive executive;
    std::ofstream ofs("End_Report.txt", std::ofstream::trunc);
    ofs.close();

    executive.EndReport();
    std::ifstream ifs("End_Report.txt");
    std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ASSERT_NE(content, "");

    executive.EndReport({.SOCDANGER=true});
    std::remove("End_Report.txt");
}


TEST_F(WaypointExecutiveTest, DidWeSeeObjectTest) {
    WaypointExecutive executive;
    executive.Last_Detected_Objects_Vector.push_back("Object1");
    ASSERT_TRUE(executive.DidWeSeeObject("Object1"));
    ASSERT_FALSE(executive.DidWeSeeObject("Object2"));
}

TEST_F(WaypointExecutiveTest, findFileInDirectoryTest) {
    std::filesystem::path temp_dir = std::filesystem::temp_directory_path() / "test_directory";
    std::filesystem::create_directories(temp_dir);
    std::filesystem::path temp_file = temp_dir / "test_file.txt";
    std::ofstream(temp_file).close();

    WaypointExecutive executive;
    auto result = executive.findFileInDirectory(temp_dir.parent_path(), temp_dir.filename().string(), temp_file.filename().string());
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value(), temp_file);
    std::filesystem::remove_all(temp_dir);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}