#include "digital_twin/digital_twin.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<digital_twin::DigitalTwin>();
  
  RCLCPP_INFO(node->get_logger(), 
              "Digital Twin Monitoring System initialized");
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
