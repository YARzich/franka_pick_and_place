# Responsibility: Build and install the optional cuRobo planning reference backend
# and MoveIt plugin.
cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
project(manipulation_curobo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(curobo_process_support STATIC
  src/child_process.cpp
  src/planning_admission.cpp
  src/temporary_file.cpp
)
set_target_properties(curobo_process_support PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(curobo_process_support PRIVATE cxx_std_17)
target_include_directories(curobo_process_support
  PRIVATE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
)

add_library(curobo_planning_io STATIC
  src/planning_goal_extraction.cpp
  src/planning_request_serialization.cpp
  src/planning_response.cpp
)
set_target_properties(curobo_planning_io PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(curobo_planning_io PRIVATE cxx_std_17)
target_include_directories(curobo_planning_io PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(curobo_planning_io Eigen3::Eigen yaml-cpp)
ament_target_dependencies(curobo_planning_io
  builtin_interfaces
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  shape_msgs
  trajectory_msgs
  yaml_cpp_vendor
)

add_library(curobo_moveit_plugin SHARED
  src/curobo_planner_manager.cpp
  src/curobo_planning_context.cpp
  src/package_resource.cpp
)
target_compile_features(curobo_moveit_plugin PRIVATE cxx_std_17)
target_include_directories(curobo_moveit_plugin
  PRIVATE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
)
target_include_directories(curobo_moveit_plugin PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(
  curobo_moveit_plugin
  curobo_planning_io
  curobo_process_support
  yaml-cpp
)
ament_target_dependencies(curobo_moveit_plugin
  ament_index_cpp
  builtin_interfaces
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  trajectory_msgs
  yaml_cpp_vendor
)

install(
  TARGETS curobo_moveit_plugin
  LIBRARY DESTINATION lib
)
install(PROGRAMS scripts/curobo_motion_bridge.py DESTINATION share/${PROJECT_NAME}/scripts)
install(
  FILES
    API.md
    CHANGELOG.rst
    LICENSE
    MIGRATION.md
    README.md
    THIRD_PARTY_NOTICES.md
    curobo_moveit_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  moveit_core curobo_moveit_plugin_description.xml)

ament_python_install_package(manipulation_curobo_runtime)

# Incremental overlays retain files removed by a newer package revision. Delete
# only exact historical wire/runtime and formerly exported helper artifacts.
install(CODE [[
  set(_install_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
  file(REMOVE_RECURSE
    "${_install_root}/share/manipulation_curobo/action"
    "${_install_root}/share/manipulation_curobo/rust"
    "${_install_root}/include/manipulation_curobo/manipulation_curobo/action"
  )
  file(REMOVE
    "${_install_root}/share/ament_index/resource_index/rosidl_interfaces/manipulation_curobo"
    "${_install_root}/share/ament_index/resource_index/rust_packages/manipulation_curobo"
    "${_install_root}/lib/manipulation_curobo/curobo_planner_server.py"
  )
  file(REMOVE_RECURSE
    "${_install_root}/include/manipulation_curobo"
  )
  file(REMOVE
    "${_install_root}/lib/libcurobo_planning_io.a"
    "${_install_root}/lib/libcurobo_process_support.so"
    "${_install_root}/lib/libcurobo_process_support.a"
    "${_install_root}/share/manipulation_curobo/cmake/ament_cmake_export_dependencies-extras.cmake"
    "${_install_root}/share/manipulation_curobo/cmake/ament_cmake_export_include_directories-extras.cmake"
    "${_install_root}/share/manipulation_curobo/cmake/ament_cmake_export_targets-extras.cmake"
  )
  file(GLOB _legacy_rosidl_artifacts
    "${_install_root}/lib/libmanipulation_curobo__rosidl*"
    "${_install_root}/share/manipulation_curobo/cmake/*rosidl*"
    "${_install_root}/local/lib/python*/dist-packages/manipulation_curobo"
    "${_install_root}/lib/python*/site-packages/manipulation_curobo"
  )
  if(_legacy_rosidl_artifacts)
    file(REMOVE_RECURSE ${_legacy_rosidl_artifacts})
  endif()
  file(GLOB _legacy_cmake_exports
    "${_install_root}/share/manipulation_curobo/cmake/export_manipulation_curobo*"
  )
  if(_legacy_cmake_exports)
    file(REMOVE ${_legacy_cmake_exports})
  endif()
  file(GLOB _runtime_install_roots
    "${_install_root}/local/lib/python*/dist-packages/manipulation_curobo_runtime"
    "${_install_root}/lib/python*/site-packages/manipulation_curobo_runtime"
  )
  file(GLOB _legacy_runtime_metadata
    "${_install_root}/local/lib/python*/dist-packages/manipulation_curobo_runtime-0.1.0*.egg-info"
    "${_install_root}/lib/python*/site-packages/manipulation_curobo_runtime-0.1.0*.egg-info"
  )
  if(_legacy_runtime_metadata)
    file(REMOVE_RECURSE ${_legacy_runtime_metadata})
  endif()
  set(_legacy_runtime_modules
    goal_admission
    planner_server
    planning_contract
    planning_worker
    reference_runtime
    world_config
  )
  foreach(_runtime_root IN LISTS _runtime_install_roots)
    foreach(_module IN LISTS _legacy_runtime_modules)
      file(REMOVE "${_runtime_root}/${_module}.py")
      file(GLOB _legacy_bytecode "${_runtime_root}/__pycache__/${_module}*.pyc")
      if(_legacy_bytecode)
        file(REMOVE ${_legacy_bytecode})
      endif()
    endforeach()
  endforeach()
]])

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(manipulation REQUIRED)
  find_package(manipulation_moveit REQUIRED)

  ament_add_gtest(test_child_process test/test_child_process.cpp)
  target_include_directories(test_child_process PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(test_child_process curobo_process_support)

  ament_add_gtest(test_temporary_file test/test_temporary_file.cpp)
  target_include_directories(test_temporary_file PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(test_temporary_file curobo_process_support)

  ament_add_gtest(test_planning_admission test/test_planning_admission.cpp)
  target_include_directories(test_planning_admission PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(test_planning_admission curobo_process_support)

  ament_add_gtest(test_planning_io test/test_planning_io.cpp)
  target_include_directories(
    test_planning_io PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
  target_link_libraries(
    test_planning_io
    curobo_planning_io
    curobo_process_support
    moveit_core::moveit_test_utils
    yaml-cpp
  )
  ament_target_dependencies(test_planning_io rclcpp)

  ament_add_gtest(test_plugin_loader test/test_plugin_loader.cpp)
  target_include_directories(test_plugin_loader PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(
    test_plugin_loader curobo_process_support moveit_core::moveit_test_utils)
  ament_target_dependencies(
    test_plugin_loader moveit_core moveit_ros_planning pluginlib rclcpp)

  ament_add_gtest(
    test_v2_curobo_composition test/test_v2_curobo_composition.cpp)
  target_include_directories(
    test_v2_curobo_composition PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(
    test_v2_curobo_composition
    curobo_process_support
    manipulation::workflow
    manipulation_moveit::pick_lift_place_strategy
    moveit_core::moveit_test_utils
  )
  ament_target_dependencies(
    test_v2_curobo_composition
    manipulation
    manipulation_moveit
    moveit_core
    moveit_ros_planning
    pluginlib
    rclcpp
  )

  ament_add_gtest(test_package_resource test/test_package_resource.cpp)
  target_include_directories(test_package_resource PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_link_libraries(test_package_resource curobo_moveit_plugin)

  ament_add_pytest_test(
    test_curobo_motion_bridge
    test/test_curobo_motion_bridge.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_scene_conversion
    test/test_scene_conversion.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_generic_boundary
    test/test_generic_boundary.py
    TIMEOUT 60
  )

  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()

  # The automatic hook cannot pass the GoogleTest macro configuration.
  ament_cppcheck(
    LANGUAGE c++
    LIBRARIES googletest
    INCLUDE_DIRS
      "${PROJECT_SOURCE_DIR}/include"
      "${PROJECT_SOURCE_DIR}/src"
  )
  # Jazzy ament_cppcheck otherwise skips every cppcheck 2.x release.
  set_property(TEST cppcheck APPEND PROPERTY ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
endif()

ament_package()
