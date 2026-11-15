# Responsibility: Generate robot-independent actions, services, and status
# messages shared by manipulation applications and backend adapters.
cmake_minimum_required(VERSION 3.16)
project(manipulation_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

set(message_files
  "msg/HealthComponentStatus.msg"
  "msg/ManipulationTaskResult.msg"
  "msg/ManipulationTaskState.msg"
  "msg/MotionTaskResult.msg"
  "msg/PlanningSceneCoordinatorState.msg"
  "msg/PlanningSceneOperationResult.msg"
  "msg/SystemHealthSnapshot.msg"
  "msg/TransferObjectStepV2.msg"
)

set(action_files
  "action/ServoToPose.action"
  "action/TransferObjectSequenceV2.action"
)

set(service_files
  "srv/AcquirePlanningSceneLease.srv"
  "srv/ApplyPlanningSceneDiff.srv"
  "srv/GetSystemHealth.srv"
  "srv/GetPlanningSceneState.srv"
  "srv/ReconcilePlanningScene.srv"
  "srv/ReleasePlanningSceneLease.srv"
  "srv/RenewPlanningSceneLease.srv"
  "srv/SetTaskPaused.srv"
  "srv/ValidatePlanningSceneLease.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  ${action_files}
  ${service_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    moveit_msgs
    unique_identifier_msgs
)

install(FILES CHANGELOG.rst LICENSE MIGRATION.md README.md DESTINATION share/${PROJECT_NAME})
install(
  FILES
    docs/public_wire_contract.v1.json
    docs/public_wire_contract.v2.json
    docs/transfer_v1_removal_inventory.md
  DESTINATION share/${PROJECT_NAME}/docs
)

# Incremental installs do not remove generated files for deleted interfaces.
# Retire only the reviewed V1 types so an upgraded overlay cannot still resolve
# their source definitions, generated headers, or private Python modules.
install(CODE [[
  set(_install_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
  set(_legacy_action_stems
    cu_robo_plan_pose
    perception_resolved_transfer_sequence
    transfer_object
    transfer_object_sequence)
  set(_legacy_message_stems
    perception_resolved_transfer_step
    static_box_transfer_step)

  foreach(_stem IN LISTS _legacy_action_stems)
    file(REMOVE
      "${_install_root}/include/manipulation_msgs/manipulation_msgs/action/${_stem}.h"
      "${_install_root}/include/manipulation_msgs/manipulation_msgs/action/${_stem}.hpp")
    file(GLOB _generated_detail
      "${_install_root}/include/manipulation_msgs/manipulation_msgs/action/detail/${_stem}__*")
    if(_generated_detail)
      file(REMOVE ${_generated_detail})
    endif()
  endforeach()
  foreach(_stem IN LISTS _legacy_message_stems)
    file(REMOVE
      "${_install_root}/include/manipulation_msgs/manipulation_msgs/msg/${_stem}.h"
      "${_install_root}/include/manipulation_msgs/manipulation_msgs/msg/${_stem}.hpp")
    file(GLOB _generated_detail
      "${_install_root}/include/manipulation_msgs/manipulation_msgs/msg/detail/${_stem}__*")
    if(_generated_detail)
      file(REMOVE ${_generated_detail})
    endif()
  endforeach()

  foreach(_name IN ITEMS
      CuRoboPlanPose
      PerceptionResolvedTransferSequence
      TransferObject
      TransferObjectSequence)
    file(GLOB _legacy_definition
      "${_install_root}/share/manipulation_msgs/action/${_name}.*")
    if(_legacy_definition)
      file(REMOVE ${_legacy_definition})
    endif()
  endforeach()
  foreach(_name IN ITEMS
      PerceptionResolvedTransferStep
      StaticBoxTransferStep)
    file(GLOB _legacy_definition
      "${_install_root}/share/manipulation_msgs/msg/${_name}.*")
    if(_legacy_definition)
      file(REMOVE ${_legacy_definition})
    endif()
  endforeach()

  file(GLOB _python_package_roots
    "${_install_root}/lib/python*/site-packages/manipulation_msgs")
  foreach(_python_root IN LISTS _python_package_roots)
    foreach(_stem IN LISTS _legacy_action_stems)
      file(REMOVE
        "${_python_root}/action/_${_stem}.py"
        "${_python_root}/action/_${_stem}_s.c")
      file(GLOB _legacy_bytecode
        "${_python_root}/action/__pycache__/_${_stem}.*.pyc")
      if(_legacy_bytecode)
        file(REMOVE ${_legacy_bytecode})
      endif()
    endforeach()
    foreach(_stem IN LISTS _legacy_message_stems)
      file(REMOVE
        "${_python_root}/msg/_${_stem}.py"
        "${_python_root}/msg/_${_stem}_s.c")
      file(GLOB _legacy_bytecode
        "${_python_root}/msg/__pycache__/_${_stem}.*.pyc")
      if(_legacy_bytecode)
        file(REMOVE ${_legacy_bytecode})
      endif()
    endforeach()
  endforeach()

]])

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosidl_adapter REQUIRED)
  find_package(rosidl_parser REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_public_wire_contract
    test/test_public_wire_contract.py
  )
  ament_add_pytest_test(
    test_transfer_object_sequence_v2_contract
    test/test_transfer_object_sequence_v2_contract.py
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
