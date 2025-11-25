#include "digital_twin/anomaly_detector.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace digital_twin {

AnomalyDetector::AnomalyDetector() : max_history_size_(1000), statistical_threshold_(3.0) {
}

void AnomalyDetector::update_historical_data(const sensor_msgs::msg::JointState& state) {
  for (size_t i = 0; i < state.name.size(); ++i) {
    const auto& joint_name = state.name[i];
    
    if (joint_stats_.find(joint_name) == joint_stats_.end()) {
      joint_stats_[joint_name] = JointStatistics{};
    }
    
    auto& stats = joint_stats_[joint_name];
    
    // Update position history
    stats.position_history.push_back(state.position[i]);
    if (stats.position_history.size() > max_history_size_) {
      stats.position_history.pop_front();
    }
    
    // Update statistical models periodically
    if (stats.position_history.size() % 100 == 0) {
      update_statistical_model();
    }
  }
}

AnomalyDetector::AnomalyResult AnomalyDetector::detect_anomalies(
    const sensor_msgs::msg::JointState& observed, 
    const sensor_msgs::msg::JointState& expected) {
    
  AnomalyResult result{false, 0.0, "none", {}};
  std::vector<double> deviations;
  
  for (size_t i = 0; i < observed.name.size(); ++i) {
    const auto& joint_name = observed.name[i];
    double pos_deviation = std::abs(observed.position[i] - expected.position[i]);
    deviations.push_back(pos_deviation);
    
    // Check statistical anomalies
    if (check_statistical_anomaly(joint_name, observed.position[i], expected.position[i])) {
      result.is_anomaly = true;
      result.confidence = std::max(result.confidence, 0.9);
      result.type = "statistical_deviation";
    }
    
    // Check for sudden changes (impact detection)
    if (pos_deviation > 0.5) { // threshold value
      result.is_anomaly = true;
      result.confidence = std::max(result.confidence, 0.95);
      result.type = "sudden_change";
    }
  }
  
  result.deviations = deviations;
  
  // If no specific anomaly detected, calculate overall health score
  if (!result.is_anomaly) {
    result.confidence = 1.0 - calculate_health_score();
  }
  
  return result;
}

bool AnomalyDetector::check_statistical_anomaly(const std::string& joint_name, 
                                               double observed_pos, double expected_pos) {
  if (joint_stats_.find(joint_name) == joint_stats_.end()) {
    return false;
  }
  
  const auto& stats = joint_stats_.at(joint_name);
  if (stats.position_history.size() < 10) return false;
  
  double deviation = std::abs(observed_pos - expected_pos);
  double z_score = deviation / (stats.std_dev_position + 1e-6);
  
  return z_score > statistical_threshold_;
}

double AnomalyDetector::calculate_health_score() const {
  if (joint_stats_.empty()) return 0.0;
  
  double total_deviation = 0.0;
  int count = 0;
  
  for (const auto& [joint_name, stats] : joint_stats_) {
    if (!stats.position_history.empty()) {
      total_deviation += stats.std_dev_position;
      count++;
    }
  }
  
  return count > 0 ? total_deviation / count : 0.0;
}

void AnomalyDetector::update_statistical_model() {
  for (auto& [joint_name, stats] : joint_stats_) {
    if (stats.position_history.empty()) continue;
    
    // Calculate mean
    stats.mean_position = std::accumulate(
      stats.position_history.begin(), stats.position_history.end(), 0.0) 
      / stats.position_history.size();
    
    // Calculate standard deviation
    double sq_sum = std::inner_product(
      stats.position_history.begin(), stats.position_history.end(),
      stats.position_history.begin(), 0.0);
    stats.std_dev_position = std::sqrt(
      sq_sum / stats.position_history.size() - stats.mean_position * stats.mean_position);
  }
}

} // namespace digital_twin
