# Responsibility: Register health-policy, diagnostics-adapter, and monitor tests.
# Does not own: PlanningScene registry or manipulation workflow tests.

include_guard(GLOBAL)

ament_add_gtest(test_freshness_monitor test/monitoring/test_freshness_monitor.cpp)
target_link_libraries(test_freshness_monitor manipulation_monitoring)

ament_add_gtest(test_health_aggregator test/monitoring/test_health_aggregator.cpp)
target_link_libraries(test_health_aggregator manipulation_monitoring)

ament_add_gtest(
  test_perception_health_policy
  test/monitoring/test_perception_health_policy.cpp
)
target_link_libraries(test_perception_health_policy manipulation_monitoring)

ament_add_gtest(
  test_perception_diagnostics_adapter
  test/monitoring/test_perception_diagnostics_adapter.cpp
  src/monitoring/perception_diagnostics_adapter.cpp
)
target_include_directories(test_perception_diagnostics_adapter PRIVATE src/monitoring)
target_link_libraries(test_perception_diagnostics_adapter manipulation_monitoring)
ament_target_dependencies(test_perception_diagnostics_adapter diagnostic_msgs rclcpp)

ament_add_gtest(
  test_duration_conversion
  test/monitoring/test_duration_conversion.cpp
  src/monitoring/duration_conversion.cpp
)
target_include_directories(test_duration_conversion PRIVATE src/monitoring)

ament_add_gtest(
  test_service_request_tracker
  test/monitoring/test_service_request_tracker.cpp
  src/monitoring/service_request_tracker.cpp
)
target_include_directories(test_service_request_tracker PRIVATE src/monitoring)

add_launch_test(
  test/monitoring/test_system_health_monitor_launch.py
  TIMEOUT 45
)
add_launch_test(
  test/monitoring/test_system_health_perception_launch.py
  TIMEOUT 45
)
add_launch_test(
  test/monitoring/test_system_health_planning_scene_launch.py
  TIMEOUT 45
)
add_launch_test(
  test/monitoring/test_system_health_startup_launch.py
  TIMEOUT 30
)
add_launch_test(
  test/monitoring/test_system_health_invalid_stamps_launch.py
  TIMEOUT 30
)
