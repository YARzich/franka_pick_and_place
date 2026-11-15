# Responsibility: Register deterministic ServoToPose state-machine unit tests.
# Does not own: ROS, MoveIt Servo, controller, or action integration tests.

include_guard(GLOBAL)

ament_add_gtest(
  test_servo_to_pose_tracking
  test/servo/test_servo_to_pose_tracking.cpp
)
target_include_directories(
  test_servo_to_pose_tracking PRIVATE test/servo)
target_link_libraries(test_servo_to_pose_tracking manipulation_servo)

ament_add_gtest(
  test_servo_to_pose_control
  test/servo/test_servo_to_pose_control.cpp
)
target_include_directories(
  test_servo_to_pose_control PRIVATE test/servo)
target_link_libraries(test_servo_to_pose_control manipulation_servo)
