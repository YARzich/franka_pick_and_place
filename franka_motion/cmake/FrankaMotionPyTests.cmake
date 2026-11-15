# Responsibility: Register fast Python contracts for canonical Panda launch,
# runtime, task, and public ROS compatibility behavior.
ament_add_pytest_test(test_launch_helpers test/test_launch_helpers.py)
ament_add_pytest_test(test_gigapose_dataset_cli test/test_gigapose_dataset_cli.py)
ament_add_pytest_test(
  test_gigapose_launch_contract
  test/test_gigapose_launch_contract.py
)
ament_add_pytest_test(
  test_perception_diagnostics_profile
  test/test_perception_diagnostics_profile.py
)
ament_add_pytest_test(test_launch_events test/test_launch_events.py)
ament_add_pytest_test(
  test_public_compatibility_contract
  test/test_public_compatibility_contract.py
)
ament_add_pytest_test(test_robot_description test/test_robot_description.py)

foreach(task_test
    task_active_cancel_config
    task_active_cancel_motion
    task_active_cancel_validation
    task_launch
    task_model
    task_sequence_client_policy
    task_sequence_client_process
    task_sequence_client_runtime
    task_sequence_client_waits
    task_sequence_goal_guard
    task_sequence_messages)
  ament_add_pytest_test(
    "test_${task_test}"
    "test/tasks/test_${task_test}.py"
  )
endforeach()

foreach(bringup_test
    backend_contract
    optional_curobo_pipeline
    gazebo_stack
    isaac_stack
    launch_description_contract
    octomap_profile
    process_group_runner
    sequence_workflow_wiring
    source_contract
    workflow_wiring)
  ament_add_pytest_test(
    "test_bringup_${bringup_test}"
    "test/bringup/test_${bringup_test}.py"
  )
endforeach()
ament_add_pytest_test(
  test_description_contract
  test/description/test_description_contract.py
)
ament_add_pytest_test(
  test_moveit_config_model_contract
  test/moveit_config/test_model_contract.py
)
ament_add_pytest_test(
  test_moveit_config_robot_profile_contract
  test/moveit_config/test_robot_profile_contract.py
)
ament_add_pytest_test(
  test_moveit_configs_builder
  test/moveit_config/test_moveit_configs_builder.py
)
ament_add_pytest_test(
  test_moveit_config_yaml_contract
  test/moveit_config/test_yaml_contract.py
)
ament_add_pytest_test(
  test_runtime_managed_process_wrapper
  test/test_runtime_managed_process_wrapper.py
)
ament_add_pytest_test(
  test_runtime_ros_gz_bridge_wrapper
  test/test_runtime_ros_gz_bridge_wrapper.py
)

foreach(runtime_validation_test
    detected_round_trip_profile
    frame_id
    gpu_perception_profile
    runtime_config
    runtime_harness
    runtime_process
    runtime_process_fault
    tooling_portability)
  ament_add_pytest_test(
    "test_${runtime_validation_test}"
    "test/runtime_validation/test_${runtime_validation_test}.py"
    TIMEOUT 60
  )
endforeach()

foreach(architecture_test
    active_cancel_architecture
    file_responsibility_contract
    frame_validation_architecture
    package_dependency_architecture
    pose_header_architecture
    repository_policy_contract)
  ament_add_pytest_test(
    "test_${architecture_test}"
    "test/architecture/test_${architecture_test}.py"
    TIMEOUT 60
  )
endforeach()
