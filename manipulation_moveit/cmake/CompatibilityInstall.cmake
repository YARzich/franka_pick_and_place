# Responsibility: Remove exact V1 artifacts retained by incremental overlay installs.

# CMake installs do not prune files that disappear from a newer package
# revision. Run this cleanup before installing the current targets and headers
# so only reviewed V1 artifacts are removed and the V2 surface is repopulated.
install(CODE [[
  set(_install_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
  file(REMOVE_RECURSE
    "${_install_root}/include/manipulation_moveit"
    "${_install_root}/share/manipulation_moveit/config"
  )
  file(REMOVE
    "${_install_root}/lib/libmoveit_motion_host.a"
    "${_install_root}/lib/libmoveit_motion_host.so"
    "${_install_root}/lib/libmoveit_transfer_task.a"
    "${_install_root}/lib/libmoveit_transfer_task.so"
    "${_install_root}/lib/libtransfer_backend.a"
    "${_install_root}/lib/libtransfer_backend.so"
    "${_install_root}/lib/manipulation_moveit/moveit_transfer_object_server"
  )
]])
