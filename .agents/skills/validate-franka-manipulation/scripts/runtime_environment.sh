#!/usr/bin/env bash
# Responsibility: Resolve repository/workspace paths and load only the selected
# ROS environment for the portable Franka validation and diagnosis wrappers.

resolve_franka_paths() {
  local script_dir candidate
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  REPO_ROOT="$(cd "${script_dir}/../../../.." && pwd)"
  if [[ -f "${REPO_ROOT}/franka_motion/package.xml" ]]; then
    FAMILY_ROOT="${REPO_ROOT}"
  elif [[ -f "${REPO_ROOT}/manipulation/franka_motion/package.xml" ]]; then
    FAMILY_ROOT="${REPO_ROOT}/manipulation"
  else
    echo "Cannot find the Franka package beside the repository skills" >&2
    return 2
  fi

  if [[ -n "${WS_DIR:-}" ]]; then
    WS_DIR="$(cd "${WS_DIR}" && pwd)" || return 2
  else
    WS_DIR=""
    candidate="${REPO_ROOT}"
    while [[ "${candidate}" != / ]]; do
      if [[ "${candidate##*/}" == src ]]; then
        WS_DIR="$(dirname "${candidate}")"
        break
      fi
      candidate="$(dirname "${candidate}")"
    done
  fi
  export WS_DIR
  ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
}

require_franka_workspace() {
  if [[ -z "${WS_DIR}" ]]; then
    echo "Set WS_DIR to the colcon workspace, or place the checkout below <workspace>/src" >&2
    return 2
  fi
}

source_franka_setup() {
  if [[ ! -f "${ROS_SETUP_FILE}" ]]; then
    echo "ROS setup not found: ${ROS_SETUP_FILE}" >&2
    return 2
  fi
  local restore_nounset="false"
  if [[ "${-}" == *u* ]]; then
    restore_nounset="true"
    set +u
  fi
  # shellcheck source=/dev/null
  source "${ROS_SETUP_FILE}"
  if [[ -n "${WS_DIR}" ]]; then
    if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
      echo "Workspace is not built: ${WS_DIR}/install/setup.bash" >&2
      [[ "${restore_nounset}" != "true" ]] || set -u
      return 2
    fi
    # shellcheck source=/dev/null
    source "${WS_DIR}/install/setup.bash"
  fi
  [[ "${restore_nounset}" != "true" ]] || set -u
}

franka_package_share() {
  local share
  share="$(ros2 pkg prefix --share franka_motion)" || return 2
  if [[ -n "${WS_DIR}" && "${share}" != "${WS_DIR}/install/"* ]]; then
    echo "franka_motion resolves outside the selected workspace: ${share}" >&2
    return 2
  fi
  printf '%s\n' "${share}"
}
