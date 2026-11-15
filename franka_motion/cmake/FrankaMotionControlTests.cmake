# Responsibility: Register focused unit and source-contract tests for Panda control code.

franka_motion_add_gtest(
  test_gripper_utils
  test/control/test_gripper_utils.cpp
)

franka_motion_add_gtest(
  test_gripper_goal_evaluator
  test/control/test_gripper_goal_evaluator.cpp
)

franka_motion_add_gtest(
  test_gripper_goal_reservation
  test/control/test_gripper_goal_reservation.cpp
)

franka_motion_add_gtest(
  test_joint_state_monitor
  test/control/test_joint_state_monitor.cpp
)
ament_target_dependencies(test_joint_state_monitor rclcpp sensor_msgs)

franka_motion_add_gtest(
  test_panda_joint_state_utils
  test/control/test_panda_joint_state_utils.cpp
)
ament_target_dependencies(test_panda_joint_state_utils sensor_msgs)

franka_motion_add_gtest(
  test_gripper_action_utils
  test/control/test_gripper_action_utils.cpp
)
ament_target_dependencies(
  test_gripper_action_utils
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)

franka_motion_add_gtest(
  test_panda_gripper_commands
  test/control/test_panda_gripper_commands.cpp
  src/control/panda_gripper_commands.cpp
)
target_include_directories(
  test_panda_gripper_commands
  PRIVATE "${PROJECT_SOURCE_DIR}/src/control"
)
ament_target_dependencies(test_panda_gripper_commands std_msgs)

franka_motion_add_gtest(
  test_panda_gripper_driver_config
  test/control/test_panda_gripper_driver_config.cpp
  src/control/panda_gripper_driver_config.cpp
)
target_include_directories(
  test_panda_gripper_driver_config
  PRIVATE "${PROJECT_SOURCE_DIR}/src/control"
)
ament_target_dependencies(test_panda_gripper_driver_config rclcpp)

ament_add_pytest_test(
  test_control_config_contract
  test/control/test_control_config_contract.py
)
ament_add_pytest_test(
  test_control_launch_overlay
  test/control/test_control_launch_overlay.py
)
ament_add_pytest_test(
  test_gripper_driver_source_contract
  test/control/test_driver_source_contract.py
)
