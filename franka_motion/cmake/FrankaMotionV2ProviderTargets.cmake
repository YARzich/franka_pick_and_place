# Responsibility: Build the private Panda catalog, observation and MTC provider layer.

include_guard(GLOBAL)

add_library(
  franka_motion_v2_providers
  STATIC
  src/workflow_v2/mtc_grasp_provider.cpp
  src/workflow_v2/object_catalog_yaml.cpp
  src/workflow_v2/observation_providers.cpp
  src/workflow_v2/process_config.cpp
  src/workflow_v2/provider_config_yaml.cpp
  src/workflow_v2/provider_runtime.cpp
)
franka_motion_configure_cpp_target(franka_motion_v2_providers)
target_include_directories(
  franka_motion_v2_providers
  PRIVATE
    "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  franka_motion_v2_providers
  Eigen3::Eigen
  manipulation::validation
  manipulation::workflow
  yaml-cpp
)
ament_target_dependencies(
  franka_motion_v2_providers
  ament_index_cpp
  builtin_interfaces
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_task_constructor_core
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
)

# Intentionally no install/export rule: these concrete providers belong only to
# the Franka composition root and are not a reusable package API.
