#include "digital_twin/digital_twin.hpp"

namespace digital_twin {

DigitalTwin::DigitalTwin() : Node("digital_twin_monitoring"), model_initialized_(false) {
  // Declare parameters
  this->declare_parameter<double>("anomaly_threshold", 0.85);
  this->declare_parameter<std::string>("robot_description", "robot_description");
  
  // Get parameters
  anomaly_threshold_ = this->get_parameter("anomaly_threshold").as_double();
  robot_description_ = this->get_parameter("robot_description").as_string();
  
  // Initialize subscribers
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    std::bind(&DigitalTwin::camera_callback, this, std::placeholders::_1));
    
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&DigitalTwin::joint_state_callback, this, std::placeholders::_1));
  
  // Initialize publishers
  digital_twin_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/digital_twin/state", 10);
  anomaly_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/digital_twin/anomalies", 10);
  
  // Initialize core components
  anomaly_detector_ = std::make_unique<AnomalyDetector>();
  perception_engine_ = std::make_unique<PerceptionEngine>();
  
  RCLCPP_INFO(this->get_logger(), "Digital Twin node initialized successfully");
}

void DigitalTwin::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    current_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    
    // Process visual data for state estimation
    auto observed_pose = perception_engine_->estimate_robot_pose(current_frame_);
    
    // Update digital twin model
    update_digital_twin();
    
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
  }
}

void DigitalTwin::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  current_joint_state_ = *msg;
  
  if (model_initialized_) {
    // Perform anomaly detection
    auto anomaly_result = anomaly_detector_->detect_anomalies(
      current_joint_state_, current_joint_state_); // In real implementation, compare with expected
    
    if (anomaly_result.is_anomaly && anomaly_result.confidence > anomaly_threshold_) {
      publish_anomaly_alert(anomaly_result.type, anomaly_result.confidence);
      
      RCLCPP_WARN(this->get_logger(), 
                  "Anomaly detected: %s (confidence: %.2f)", 
                  anomaly_result.type.c_str(), anomaly_result.confidence);
    }
  }
  
  anomaly_detector_->update_historical_data(current_joint_state_);
}

void DigitalTwin::update_digital_twin() {
  if (!model_initialized_) return;
  
  // Update expected robot state based on sensor fusion
  geometry_msgs::msg::PoseArray digital_twin_state;
  
  // Calculate expected poses based on joint states and robot model
  // This is where the digital twin model gets updated
  
  digital_twin_pub_->publish(digital_twin_state);
}

void DigitalTwin::publish_anomaly_alert(const std::string& anomaly_type, double confidence) {
  std_msgs::msg::String alert_msg;
  alert_msg.data = "ANOMALY:" + anomaly_type + ":" + std::to_string(confidence);
  anomaly_pub_->publish(alert_msg);
}

} // namespace digital_twin
