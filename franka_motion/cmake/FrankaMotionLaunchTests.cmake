# Responsibility: Register compatibility launch tests, including opt-in simulator
# integration coverage.
add_launch_test(test/launch/test_panda_gripper_driver_launch.py TIMEOUT 30)

if(FRANKA_MOTION_ENABLE_SIMULATOR_LAUNCH_TESTS)
  # These compatibility tests start real simulator processes. Keep them out of
  # the hosted CPU gate and enable them only on a runtime-capable host.
  ament_add_pytest_test(test_shutdown_cleanup test/test_shutdown_cleanup.py TIMEOUT 240)
  add_launch_test(test/launch/test_franka_gazebo_moveit_launch.py TIMEOUT 120)
  add_launch_test(test/launch/test_franka_gazebo_moveit_table_launch.py TIMEOUT 120)
  add_launch_test(test/launch/test_franka_gazebo_moveit_table_camera_launch.py TIMEOUT 180)
  add_launch_test(
    test/simulation/isaac/launch/test_franka_isaac_moveit_launch.py
    TIMEOUT 300
  )
  add_launch_test(
    test/simulation/isaac/launch/test_franka_isaac_moveit_table_launch.py
    TIMEOUT 300
  )
  add_launch_test(
    test/simulation/isaac/launch/test_franka_isaac_moveit_table_camera_launch.py
    TIMEOUT 420
  )

  set_tests_properties(
    test_shutdown_cleanup
    test_launch_test_franka_gazebo_moveit_launch.py
    test_launch_test_franka_gazebo_moveit_table_launch.py
    test_launch_test_franka_gazebo_moveit_table_camera_launch.py
    test_launch_test_franka_isaac_moveit_launch.py
    test_launch_test_franka_isaac_moveit_table_launch.py
    test_launch_test_franka_isaac_moveit_table_camera_launch.py
    PROPERTIES RUN_SERIAL TRUE
  )
endif()
