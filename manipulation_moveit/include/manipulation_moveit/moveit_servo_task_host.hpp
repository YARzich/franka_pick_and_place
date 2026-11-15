// Responsibility: Declares the process-local ServoToPose composition root.

#ifndef MANIPULATION_MOVEIT__MOVEIT_SERVO_TASK_HOST_HPP_
#define MANIPULATION_MOVEIT__MOVEIT_SERVO_TASK_HOST_HPP_

#include <memory>

#include "manipulation_moveit/config/moveit_servo_process_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation::moveit
{

/** Owns one ServoToPose action facade and its borrowed MoveIt Servo adapter. */
class MoveItServoTaskHost final
{
public:
  MoveItServoTaskHost(
    rclcpp::Node & node,
    LoadedMoveItServoProcessConfig config);
  ~MoveItServoTaskHost();

  MoveItServoTaskHost(const MoveItServoTaskHost &) = delete;
  MoveItServoTaskHost & operator=(const MoveItServoTaskHost &) = delete;
  MoveItServoTaskHost(MoveItServoTaskHost &&) = delete;
  MoveItServoTaskHost & operator=(MoveItServoTaskHost &&) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__MOVEIT_SERVO_TASK_HOST_HPP_
