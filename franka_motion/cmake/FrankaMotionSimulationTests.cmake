# Responsibility: Register fast simulator resource and helper contract tests.

ament_add_pytest_test(
  test_gazebo_backend
  test/simulation/gazebo/test_gazebo_backend.py
  TIMEOUT 60
)

foreach(isaac_test
    asset_license_contract
    install_contract
    launch_contract
    physical_grasp
    runtime_config
    runtime_environment
    runtime_lifecycle
    runtime_ros_graph)
  ament_add_pytest_test(
    "test_isaac_${isaac_test}"
    "test/simulation/isaac/test_${isaac_test}.py"
    TIMEOUT 60
  )
endforeach()
