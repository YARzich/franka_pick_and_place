# Responsibility: Register retained diagnostics and private V2 support tests.

function(franka_motion_add_gtest target_name)
  ament_add_gtest(${target_name} ${ARGN})
  if(TARGET ${target_name})
    franka_motion_configure_cpp_target(${target_name})
    target_link_libraries(${target_name} manipulation::validation)
  endif()
endfunction()

franka_motion_add_gtest(test_pose_utils test/test_pose_utils.cpp)
target_include_directories(test_pose_utils PRIVATE "${PROJECT_SOURCE_DIR}/src")
ament_target_dependencies(test_pose_utils rclcpp tf2_ros tf2_geometry_msgs geometry_msgs)

franka_motion_add_gtest(test_node_parameter_utils test/test_node_parameter_utils.cpp)
ament_target_dependencies(test_node_parameter_utils rclcpp)

franka_motion_add_gtest(
  test_moveit_sequence_scenario
  test/test_moveit_sequence_scenario.cpp
  src/moveit_sequence_scenario.cpp
)
target_include_directories(test_moveit_sequence_scenario PRIVATE "${PROJECT_SOURCE_DIR}/src")
ament_target_dependencies(
  test_moveit_sequence_scenario
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

franka_motion_add_gtest(test_robot_profile_arguments test/test_robot_profile_arguments.cpp)
target_link_libraries(test_robot_profile_arguments franka_motion_robot_profile_bootstrap)

franka_motion_add_gtest(test_moveit_execution_utils test/test_moveit_execution_utils.cpp)
ament_target_dependencies(
  test_moveit_execution_utils
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
)

foreach(snapshot_test
    test_moveit_parameter_sync
    test_moveit_parameter_snapshot_failures)
  franka_motion_add_gtest(${snapshot_test} test/${snapshot_test}.cpp)
  target_include_directories(${snapshot_test} PRIVATE "${PROJECT_SOURCE_DIR}/src")
  ament_target_dependencies(${snapshot_test} rcl_interfaces rclcpp)
  target_link_libraries(${snapshot_test} franka_motion_moveit_parameter_snapshot)
endforeach()

franka_motion_add_gtest(test_process_utils test/test_process_utils.cpp)
target_include_directories(test_process_utils PRIVATE "${PROJECT_SOURCE_DIR}/src")
ament_target_dependencies(
  test_process_utils
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

franka_motion_add_gtest(test_cube_geometry test/test_cube_geometry.cpp)

franka_motion_add_gtest(test_geometry_conversion test/mtc/test_geometry_conversion.cpp)
target_include_directories(test_geometry_conversion PRIVATE "${PROJECT_SOURCE_DIR}/src/mtc")
target_link_libraries(test_geometry_conversion Eigen3::Eigen)
ament_target_dependencies(test_geometry_conversion geometry_msgs)

franka_motion_add_gtest(test_mtc_stop_monitor test/mtc/test_mtc_stop_monitor.cpp)
target_include_directories(test_mtc_stop_monitor PRIVATE "${PROJECT_SOURCE_DIR}/src/mtc")
