# Responsibility: Compose the reusable manipulation libraries, ROS adapters, and tests.
cmake_minimum_required(VERSION 3.16)
project(manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(cmake/ManipulationDependencies.cmake)
include(cmake/ManipulationLibraries.cmake)
include(cmake/ManipulationExecutables.cmake)

install(
  TARGETS
    manipulation_validation
    manipulation_transport
    manipulation_task
    manipulation_servo
    manipulation_skill
    manipulation_world
    manipulation_recording
    manipulation_scene_registry
    manipulation_scene_client
    manipulation_scene_support
    manipulation_transfer_domain
    manipulation_workflow
    manipulation_monitoring
  EXPORT export_manipulation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
include(cmake/ManipulationCompatibilityInstall.cmake)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES API.md CHANGELOG.rst LICENSE README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include(cmake/ManipulationTests.cmake)
endif()

ament_export_targets(export_manipulation HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  manipulation_msgs
  moveit_msgs
  object_recognition_msgs
  OpenSSL
  rclcpp
  rclcpp_action
  shape_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  unique_identifier_msgs
  urdf
)
ament_export_include_directories(include)
ament_package()
