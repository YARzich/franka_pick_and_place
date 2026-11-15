// Responsibility: Runs the standalone system health monitor process.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "system_health_monitor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manipulation::monitoring::SystemHealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
