// Responsibility: Runs the standalone stamped point-cloud TF gate node.

#include <memory>

#include "manipulation_moveit/stamped_cloud_tf_gate.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation::moveit::StampedCloudTfGate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
