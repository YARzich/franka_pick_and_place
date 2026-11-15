# Responsibility: Remove the exact V1 transfer surface left by incremental installs.

include_guard(GLOBAL)

# A clean V2 install never creates these paths. CMake directory installs do not
# remove files retired from the source tree, so explicitly delete only the
# migrated V1 headers, executables, and archive after installing current files.
install(CODE [[
  set(_install_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")

  file(REMOVE_RECURSE
    "${_install_root}/include/manipulation/client")

  foreach(_retired_header IN ITEMS
      workflow/detail/transfer_object_session.hpp
      workflow/perception_resolved_transfer_sequence_action_server.hpp
      workflow/perception_resolved_transfer_sequence_goal_validation.hpp
      workflow/perception_resolved_transfer_target_policy.hpp
      workflow/transfer_object_action_server.hpp
      workflow/transfer_object_goal_validation.hpp
      workflow/transfer_object_result_policy.hpp
      workflow/transfer_object_sequence_action_context.hpp
      workflow/transfer_object_sequence_action_server.hpp
      workflow/transfer_object_sequence_goal_validation.hpp
      workflow/transfer_object_sequence_runner.hpp
      workflow/transfer_object_task_context.hpp)
    file(REMOVE
      "${_install_root}/include/manipulation/${_retired_header}")
  endforeach()

  file(REMOVE
    "${_install_root}/lib/manipulation/transfer_object_client"
    "${_install_root}/lib/manipulation/transfer_object_sequence_server"
    "${_install_root}/lib/libmanipulation_transfer_object_client.a")
]])
