# Responsibility: Install generic perception modules, ROS adapters, workers, and tests.
cmake_minimum_required(VERSION 3.16)
project(manipulation_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Backend workers may run under dedicated Python interpreters. Installing the
# package beside the executables preserves imports in those environments.
install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

set(perception_entrypoints
  scripts/common/disparity_frame_fix_node.py
  scripts/common/stereo_sgbm_pointcloud_node.py
  scripts/yolo/yolo_visualizer_node.py
  scripts/pose/foundationpose_depth_clean_node.py
  scripts/pose/foundationpose_detection_to_pose_node.py
  scripts/pose/foundationpose_input_frame_fix_node.py
  scripts/pose/megapose_detection_node.py
  scripts/pose/workers/gigapose_refiner_worker.py
  scripts/pose/workers/gigapose_worker.py
  scripts/pose/workers/megapose_worker.py
  scripts/validation/gpu_perception_evidence_probe.py
)
install(PROGRAMS ${perception_entrypoints} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  FILES
    API.md
    CHANGELOG.rst
    dependencies.repos
    LICENSE
    README.md
    THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_flake8_CONFIG_FILE "${PROJECT_SOURCE_DIR}/setup.cfg")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  include(cmake/ManipulationPerceptionTests.cmake)
endif()

ament_package()
