# Responsibility: Install robot-specific Python helpers shared by simulator bringup.

include_guard(GLOBAL)

ament_python_install_package(
  franka_motion_runtime
  PACKAGE_DIR "${PROJECT_SOURCE_DIR}/franka_motion_runtime"
)

# Isaac starts with a deliberately isolated PYTHONPATH. Install the same
# canonical sources beside the stable runner path so its bootstrap remains
# independent from the system ROS Python environment.
install(
  FILES
    franka_motion_runtime/__init__.py
    franka_motion_runtime/frame_id.py
  DESTINATION share/${PROJECT_NAME}/scripts/franka_motion_runtime
)
install(
  DIRECTORY franka_motion_runtime/simulation
  DESTINATION share/${PROJECT_NAME}/scripts/franka_motion_runtime
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  PROGRAMS scripts/simulation/run_franka_isaac_sim.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(
  PROGRAMS
    scripts/runtime/franka_runtime_harness
    scripts/runtime/franka_runtime_process_fault
    scripts/tasks/franka_transfer_sequence_active_cancel_probe
    scripts/tasks/franka_transfer_sequence_client
  DESTINATION lib/${PROJECT_NAME}
)
