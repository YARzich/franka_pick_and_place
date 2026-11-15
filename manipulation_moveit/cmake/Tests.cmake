# Responsibility: Register fast V2, Servo, and Octomap tests without runtime hardware.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_ros REQUIRED)

function(manipulation_moveit_add_servo_gtest target source)
  ament_add_ros_isolated_gtest(${target} ${PROJECT_SOURCE_DIR}/${source})
  target_include_directories(${target} PRIVATE "${PROJECT_SOURCE_DIR}/src/servo")
  target_link_libraries(${target} moveit_servo_ros_adapter)
endfunction()

ament_add_gtest(
  test_arm_planning_port
  ${PROJECT_SOURCE_DIR}/test/test_arm_planning_port.cpp)
target_include_directories(
  test_arm_planning_port PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(test_arm_planning_port arm_planning)

ament_add_gtest(
  test_exact_planning_scene
  ${PROJECT_SOURCE_DIR}/test/test_exact_planning_scene.cpp
  ${PROJECT_SOURCE_DIR}/src/exact_planning_scene.cpp)
target_include_directories(
  test_exact_planning_scene PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(test_exact_planning_scene moveit_core::moveit_test_utils)
ament_target_dependencies(test_exact_planning_scene moveit_core moveit_msgs shape_msgs)

ament_add_gtest(
  test_goal_planning_cancellation
  ${PROJECT_SOURCE_DIR}/test/test_goal_planning_cancellation.cpp
  ${PROJECT_SOURCE_DIR}/src/goal_planning_cancellation.cpp)
target_compile_features(test_goal_planning_cancellation PRIVATE cxx_std_17)
target_include_directories(
  test_goal_planning_cancellation PRIVATE "${PROJECT_SOURCE_DIR}/src")

ament_add_gtest(
  test_quiescence_policy
  ${PROJECT_SOURCE_DIR}/test/test_quiescence_policy.cpp)
target_compile_features(test_quiescence_policy PRIVATE cxx_std_17)
target_include_directories(
  test_quiescence_policy PRIVATE "${PROJECT_SOURCE_DIR}/src")

ament_add_gtest(
  test_cancelable_trajectory_execution
  ${PROJECT_SOURCE_DIR}/test/test_cancelable_trajectory_execution.cpp
  ${PROJECT_SOURCE_DIR}/src/cancelable_trajectory_execution.cpp)
target_compile_features(test_cancelable_trajectory_execution PRIVATE cxx_std_17)
target_include_directories(
  test_cancelable_trajectory_execution PRIVATE "${PROJECT_SOURCE_DIR}/src")

ament_add_gtest(
  test_pick_lift_place_strategy
  ${PROJECT_SOURCE_DIR}/test/test_pick_lift_place_strategy.cpp)
target_link_libraries(test_pick_lift_place_strategy pick_lift_place_strategy)

ament_add_ros_isolated_gtest(
  test_ros_v2_physical_ports
  ${PROJECT_SOURCE_DIR}/test/test_ros_v2_physical_ports.cpp)
target_include_directories(
  test_ros_v2_physical_ports PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/test")
target_link_libraries(test_ros_v2_physical_ports pick_lift_place_ros_adapters)

ament_add_ros_isolated_gtest(
  test_ros_pick_lift_place_health_port
  ${PROJECT_SOURCE_DIR}/test/test_ros_pick_lift_place_health_port.cpp)
target_include_directories(
  test_ros_pick_lift_place_health_port PRIVATE "${PROJECT_SOURCE_DIR}/test")
target_link_libraries(
  test_ros_pick_lift_place_health_port pick_lift_place_ros_adapters)

ament_add_gtest(
  test_v2_scene_adapter_support
  ${PROJECT_SOURCE_DIR}/test/test_v2_scene_adapter_support.cpp)
target_include_directories(
  test_v2_scene_adapter_support PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(test_v2_scene_adapter_support v2_scene_adapter)

ament_add_ros_isolated_gtest(
  test_ros_v2_scene_adapter_waits
  ${PROJECT_SOURCE_DIR}/test/test_ros_v2_scene_adapter_waits.cpp
  TIMEOUT 90)
target_link_libraries(
  test_ros_v2_scene_adapter_waits
  v2_scene_adapter
  moveit_core::moveit_test_utils)

ament_add_ros_isolated_gtest(
  test_moveit_arm_planning_adapter
  ${PROJECT_SOURCE_DIR}/test/test_moveit_arm_planning_adapter.cpp)
target_include_directories(
  test_moveit_arm_planning_adapter PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/test")
target_link_libraries(test_moveit_arm_planning_adapter arm_planning)

manipulation_moveit_add_servo_gtest(
  test_moveit_servo_ros_adapter_config
  test/servo/test_moveit_servo_ros_adapter_config.cpp)
manipulation_moveit_add_servo_gtest(
  test_moveit_servo_ros_adapter_tf
  test/servo/test_moveit_servo_ros_adapter_tf.cpp)
manipulation_moveit_add_servo_gtest(
  test_moveit_servo_ros_adapter_io
  test/servo/test_moveit_servo_ros_adapter_io.cpp)

ament_add_gtest(
  test_servo_to_pose_task
  ${PROJECT_SOURCE_DIR}/test/servo/test_servo_to_pose_task.cpp)
target_include_directories(
  test_servo_to_pose_task PRIVATE
  "${PROJECT_SOURCE_DIR}/src/servo"
  "${PROJECT_SOURCE_DIR}/test/servo")
target_link_libraries(test_servo_to_pose_task moveit_servo_task)

ament_add_ros_isolated_gtest(
  test_servo_to_pose_action_server
  ${PROJECT_SOURCE_DIR}/test/servo/test_servo_to_pose_action_server.cpp)
target_include_directories(
  test_servo_to_pose_action_server PRIVATE
  "${PROJECT_SOURCE_DIR}/src/servo"
  "${PROJECT_SOURCE_DIR}/test/servo")
target_link_libraries(test_servo_to_pose_action_server moveit_servo_task)

ament_add_ros_isolated_gtest(
  test_moveit_servo_process_config
  ${PROJECT_SOURCE_DIR}/test/config/test_moveit_servo_process_config.cpp)
target_include_directories(
  test_moveit_servo_process_config PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(test_moveit_servo_process_config moveit_servo_task_host)

ament_add_ros_isolated_gtest(
  test_stamped_cloud_tf_gate
  ${PROJECT_SOURCE_DIR}/test/test_stamped_cloud_tf_gate.cpp)
target_link_libraries(test_stamped_cloud_tf_gate octomap_tf_gate_support)
ament_target_dependencies(test_stamped_cloud_tf_gate geometry_msgs tf2_ros)

ament_add_ros_isolated_gtest(
  test_fail_closed_pointcloud_octomap_updater
  ${PROJECT_SOURCE_DIR}/test/test_fail_closed_pointcloud_octomap_updater.cpp
)
target_include_directories(
  test_fail_closed_pointcloud_octomap_updater PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(
  test_fail_closed_pointcloud_octomap_updater fail_closed_octomap_updater)

ament_add_pytest_test(
  test_generic_boundary
  ${PROJECT_SOURCE_DIR}/test/test_generic_boundary.py
  TIMEOUT 30
)

list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cppcheck)
ament_lint_auto_find_test_dependencies()

# The automatic hook cannot pass the GoogleTest macro configuration.
ament_cppcheck(
  LANGUAGE c++
  LIBRARIES googletest
  INCLUDE_DIRS
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/src"
    "${PROJECT_SOURCE_DIR}/src/servo"
    "${PROJECT_SOURCE_DIR}/test"
    "${PROJECT_SOURCE_DIR}/test/servo"
)
# Jazzy ament_cppcheck otherwise skips every cppcheck 2.x release.
set_property(TEST cppcheck APPEND PROPERTY ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
