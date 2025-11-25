#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "anomaly_detector.hpp"
#include "perception_engine.hpp"

namespace digital_twin {

class DigitalTwin : public rclcpp::Node {
public:
  DigitalTwin();
  ~DigitalTwin() = default;

private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void update_digital_twin();
  void publish_anomaly_alert(const std::string& anomaly_type, double confidence);
  
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr digital_twin_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anomaly_pub_;
  
  // Core components
  std::unique_ptr<AnomalyDetector> anomaly_detector_;
  std::unique_ptr<PerceptionEngine> perception_engine_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr expected_state_;
  
  // State variables
  cv::Mat current_frame_;
  sensor_msgs::msg::JointState current_joint_state_;
  bool model_initialized_;
  
  // Parameters
  double anomaly_threshold_;
  std::string robot_description_;
};

} // namespace digital_twin
