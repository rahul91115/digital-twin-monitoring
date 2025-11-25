#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <deque>
#include <Eigen/Dense>

namespace digital_twin {

class AnomalyDetector {
public:
  struct AnomalyResult {
    bool is_anomaly;
    double confidence;
    std::string type;
    std::vector<double> deviations;
  };

  AnomalyDetector();
  
  void update_historical_data(const sensor_msgs::msg::JointState& state);
  AnomalyResult detect_anomalies(const sensor_msgs::msg::JointState& observed, 
                                const sensor_msgs::msg::JointState& expected);
  void update_statistical_model();
  double calculate_health_score() const;

private:
  struct JointStatistics {
    std::deque<double> position_history;
    std::deque<double> velocity_history;
    double mean_position;
    double mean_velocity;
    double std_dev_position;
    double std_dev_velocity;
  };

  std::map<std::string, JointStatistics> joint_stats_;
  size_t max_history_size_;
  double statistical_threshold_;
  
  bool check_statistical_anomaly(const std::string& joint_name, 
                                double observed_pos, double expected_pos);
  bool check_kinematic_constraints(const sensor_msgs::msg::JointState& state);
  Eigen::VectorXd extract_features(const sensor_msgs::msg::JointState& state);
};

} // namespace digital_twin
