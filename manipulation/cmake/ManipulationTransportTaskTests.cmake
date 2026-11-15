# Responsibility: Register transport, task-control, and validation unit tests.
# Does not own: Scene, workflow, client, or monitoring test registration.

include_guard(GLOBAL)

ament_add_gtest(
  test_deadline_service_call
  test/transport/test_deadline_service_call.cpp
)
target_link_libraries(test_deadline_service_call manipulation_transport)
ament_target_dependencies(test_deadline_service_call manipulation_msgs rclcpp)

ament_add_gtest(test_execution_control test/task/test_execution_control.cpp)
target_link_libraries(test_execution_control manipulation_task)

ament_add_gtest(test_task_status test/task/test_task_status.cpp)
target_link_libraries(test_task_status manipulation_task)

ament_add_gtest(test_frame_id_validation test/validation/test_frame_id.cpp)
target_compile_features(test_frame_id_validation PRIVATE cxx_std_17)
target_include_directories(test_frame_id_validation PRIVATE include)
