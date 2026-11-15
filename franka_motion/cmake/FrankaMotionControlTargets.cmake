# Responsibility: Build and install the Panda gripper ros2_control action adapter.

include_guard(GLOBAL)

add_executable(
  panda_gripper_driver
  src/control/panda_gripper_commands.cpp
  src/control/panda_gripper_driver_action.cpp
  src/control/panda_gripper_driver_config.cpp
  src/control/panda_gripper_driver_main.cpp
  src/control/panda_gripper_driver_monitor.cpp
  src/control/panda_gripper_driver_node.cpp
)
franka_motion_configure_cpp_target(panda_gripper_driver)
target_include_directories(
  panda_gripper_driver
  PRIVATE "${PROJECT_SOURCE_DIR}/src/control"
)
ament_target_dependencies(
  panda_gripper_driver
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
)

install(TARGETS panda_gripper_driver DESTINATION lib/${PROJECT_NAME})
