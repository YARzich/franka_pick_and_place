# Responsibility: Install the historical franka_motion surface from canonical local
# resources and explicit compatibility forwarders.

include_guard(GLOBAL)

# Earlier migration stages installed these paths as links to split packages. Remove
# only those stale links so an incremental install can replace them with local files.
install(CODE "
  foreach(_local_asset IN ITEMS
      models/cube_yolo26s_isaac_seg_v5s.pt
      models/foundationpose/cube_5cm.mtl
      models/foundationpose/cube_5cm.obj
      models/foundationpose/cube_5cm_texture.ppm)
    set(_installed_asset
      \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/\${_local_asset}\")
    if(IS_SYMLINK \"\${_installed_asset}\")
      file(REMOVE \"\${_installed_asset}\")
    endif()
  endforeach()
")

install(
  DIRECTORY launch config description worlds models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN ".thumbs" EXCLUDE
)

function(franka_motion_install_resource_forwarder canonical_share relative_path)
  set(canonical_relative_path "${relative_path}")
  if(ARGC GREATER 2)
    set(canonical_relative_path "${ARGV2}")
  endif()
  get_filename_component(legacy_relative_parent "${relative_path}" DIRECTORY)
  file(
    RELATIVE_PATH forward_target
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${legacy_relative_parent}"
    "${canonical_share}/${canonical_relative_path}"
  )
  install(CODE "
    set(_legacy_path
      \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${relative_path}\")
    get_filename_component(_legacy_parent \"\${_legacy_path}\" DIRECTORY)
    file(MAKE_DIRECTORY \"\${_legacy_parent}\")
    if(EXISTS \"\${_legacy_path}\" OR IS_SYMLINK \"\${_legacy_path}\")
      file(REMOVE_RECURSE \"\${_legacy_path}\")
    endif()
    execute_process(
      COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
        \"${forward_target}\"
        \"\${_legacy_path}\"
      RESULT_VARIABLE _symlink_result
    )
    if(NOT _symlink_result EQUAL 0)
      message(FATAL_ERROR
        \"Failed to install compatibility resource ${relative_path}\")
    endif()
  ")
endfunction()

# MoveItConfigsBuilder discovers planning pipelines at config/<name>_planning.yaml.
# Keep one canonical profile in config/moveit and expose the standard package path.
franka_motion_install_resource_forwarder(
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
  config/ompl_planning.yaml
  config/moveit/ompl_planning.yaml
)
franka_motion_install_resource_forwarder(
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
  config/curobo_planning.yaml
  config/moveit/curobo_planning.yaml
)

foreach(control_profile
    initial_positions.yaml
    panda_gripper_driver.yaml
    panda_ros2_control_base.yaml
    panda_ros2_control_gazebo.yaml
    panda_ros2_control_isaac.yaml)
  franka_motion_install_resource_forwarder(
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
    "config/${control_profile}"
    "config/control/${control_profile}"
  )
endforeach()

franka_motion_install_resource_forwarder(
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
  description/table_camera.sdf
  description/gazebo/table_camera.sdf
)
franka_motion_install_resource_forwarder(
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
  config/franka_table_stereo_pointcloud.rviz
  config/rviz/franka_table_stereo_pointcloud.rviz
)
franka_motion_install_resource_forwarder(
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
  config/table_camera_perception.yaml
  config/perception/table_camera_perception.yaml
)
foreach(world_name
    panda_empty.world.sdf
    table_world.sdf
    table_world_camera.sdf)
  franka_motion_install_resource_forwarder(
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
    "worlds/${world_name}"
    "worlds/gazebo/${world_name}"
  )
endforeach()

foreach(isaac_world_resource
    FrankaPanda
    table_world_usd.usd
    table_world_usd_clean.usd)
  franka_motion_install_resource_forwarder(
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
    "worlds/${isaac_world_resource}"
    "worlds/isaac/${isaac_world_resource}"
  )
endforeach()

install(
  FILES
    .setup_assistant
    CHANGELOG.rst
    README.md
    LICENSE
    THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY LICENSES/
  DESTINATION share/${PROJECT_NAME}/LICENSES
)

install(
  DIRECTORY include/
  DESTINATION include
  PATTERN "moveit_parameter_sync.hpp" EXCLUDE
  PATTERN "moveit_parameter_snapshot_contract.hpp" EXCLUDE
)

# Preserve the old share-path command; the Panda scenario owns its cube assets.
install(
  PROGRAMS scripts/perception/prepare_gigapose_cube_dataset.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(
  DIRECTORY scripts/isaac_runtime
  DESTINATION share/${PROJECT_NAME}/scripts
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/runtime/managed_process_wrapper.py
    scripts/runtime/ros_gz_bridge_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)

set(franka_motion_perception_compatibility_executables
  disparity_frame_fix_node.py
  foundationpose_depth_clean_node.py
  foundationpose_detection_to_pose_node.py
  foundationpose_input_frame_fix_node.py
  gigapose_refiner_worker.py
  gigapose_worker.py
  megapose_detection_node.py
  megapose_worker.py
  stereo_sgbm_pointcloud_node.py
  yolo_visualizer_node.py
)
foreach(executable_name IN LISTS franka_motion_perception_compatibility_executables)
  install(
    PROGRAMS scripts/compat/perception_exec_wrapper.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME "${executable_name}"
  )
endforeach()

install(
  FILES franka_motion_runtime/processes/process_group_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(
  isaac_runtime
  PACKAGE_DIR "${PROJECT_SOURCE_DIR}/scripts/isaac_runtime"
)
