#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <chrono>

#include "WaypointExecutive.hpp"

class WaypointExecutiveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    temp_node_ = rclcpp::Node::make_shared("test_node");
    exec_instance = std::make_shared<WaypointExecutive>();

    // Publishers to send fake data into the executive
    soc_pub = temp_node_->create_publisher<std_msgs::msg::Bool>("SOCIntTopic", 10);
    pos_pub = temp_node_->create_publisher<std_msgs::msg::Float32MultiArray>("position_topic", 10);
    vision_pub = temp_node_->create_publisher<std_msgs::msg::String>("detections", 10);

    // Subscribers to monitor executive outputs
    waypoint_sub = temp_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "waypoint_topic", 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
          last_waypoint = *msg;
          waypoint_received = true;
        });

    current_task_sub = temp_node_->create_subscription<std_msgs::msg::String>(
        "CurrentTaskTopic", 10,
        [this](std_msgs::msg::String::SharedPtr msg)
        {
          last_task_name = msg->data;
          task_received = true;
        });

    manip_sub = temp_node_->create_subscription<std_msgs::msg::Int64>(
        "manipulationCommand", 10,
        [this](std_msgs::msg::Int64::SharedPtr msg)
        {
          last_manip_code = msg->data;
          manip_received = true;
        });
 // ===== Continuous position publisher thread =====
    position_pub_thread = std::jthread([this]()
    {
      std_msgs::msg::Float32MultiArray pos_msg;
      pos_msg.data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      while (!position_pub_thread.get_stop_token().stop_requested())
      {
        pos_pub->publish(pos_msg);
        // Simulate some movement if you want
        for (auto &val : pos_msg.data)
          val += 0.1f;
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    });
  
    // Start executor
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(temp_node_);
    executor->add_node(exec_instance);

    spin_thread = std::jthread([this]()
                               { executor->spin(); });

    waypoint_received = false;
    task_received = false;
    manip_received = false;
  }

  void TearDown() override
  {
      position_pub_thread.request_stop();
    exec_instance->EndReport(); // Ensure file writing done
    rclcpp::shutdown();
  }

  // Nodes & ROS objects
  rclcpp::Node::SharedPtr temp_node_;
  std::shared_ptr<WaypointExecutive> exec_instance;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr soc_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pos_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_pub;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_task_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr manip_sub;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::jthread spin_thread;
  std::jthread position_pub_thread;
  // Test state
  std_msgs::msg::Float32MultiArray last_waypoint;
  bool waypoint_received;
  std::string last_task_name;
  bool task_received;
  int64_t last_manip_code;
  bool manip_received;
};

// ------------------------------------------------------------
// TEST 1: Verify SOC interrupt stops mission
// ------------------------------------------------------------
TEST_F(WaypointExecutiveTest, HandlesSOCInterrupt)
{
  std_msgs::msg::Bool soc_msg;
  soc_msg.data = true;
  soc_pub->publish(soc_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(exec_instance->Controller(), 0);
  
}

// ------------------------------------------------------------
// TEST 2: Vision detection triggers REEF_SHARK interrupt
// ------------------------------------------------------------
TEST_F(WaypointExecutiveTest, DetectsReefShark)
{
  std_msgs::msg::String vision_msg;
  vision_msg.data = R"([{"class_name":"Reef Shark"}])";
  vision_pub->publish(vision_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(exec_instance->Controller(), 1);
}

