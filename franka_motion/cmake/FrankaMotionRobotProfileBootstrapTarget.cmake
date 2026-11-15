# Responsibility: Build and install the argv bootstrap for the package-owned robot profile.
# Does not own: Robot-profile values, compatibility executables, or MTC runtime targets.

include_guard(GLOBAL)

add_library(
  franka_motion_robot_profile_bootstrap
  SHARED
  src/robot_profile_arguments.cpp
)
target_compile_features(franka_motion_robot_profile_bootstrap PUBLIC cxx_std_17)
target_include_directories(
  franka_motion_robot_profile_bootstrap
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
set_target_properties(
  franka_motion_robot_profile_bootstrap
  PROPERTIES EXPORT_NAME robot_profile_bootstrap
)
ament_target_dependencies(
  franka_motion_robot_profile_bootstrap
  ament_index_cpp
  rclcpp
)

install(
  TARGETS franka_motion_robot_profile_bootstrap
  EXPORT export_franka_motion_scenario
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
