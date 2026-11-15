#!/usr/bin/env bash
set -euo pipefail

# Responsibility: Resolve the installed Franka runtime matrix and invoke its
# token-owned harness without duplicating simulator or ROS graph knowledge.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime_environment.sh
source "${SCRIPT_DIR}/runtime_environment.sh"

PROFILE="gazebo_smoke_v2"
BUILD_FIRST="true"
ALLOW_EXISTING_RUNTIME="false"
ARTIFACT_ROOT="${FRANKA_RUNTIME_ROOT:-/tmp/franka-runtime}"
LIST_ONLY="false"

usage() {
  cat <<'USAGE'
Usage: run_profile.sh [options]

Options:
  --profile <matrix-profile>          Default: gazebo_smoke_v2
  --build-first <true|false>          Default: true
  --allow-existing-runtime <true|false>
                                      Default: false
  --artifact-root <absolute-path>     Default: /tmp/franka-runtime
  --list                              List profiles from the source matrix (no ROS needed)
  --help

Environment:
  WS_DIR          Colcon workspace; inferred only from an ancestor src directory
  ROS_SETUP_FILE  Underlay setup file; default: /opt/ros/jazzy/setup.bash
USAGE
}

require_bool() {
  local name="$1"
  local value="$2"
  if [[ "${value}" != "true" && "${value}" != "false" ]]; then
    printf '%s must be true or false\n' "${name}" >&2
    exit 2
  fi
}

matrix_path() {
  local share
  share="$(franka_package_share)" || return 2
  printf '%s/config/runtime_validation/runtime_matrix.v3.json\n' "${share}"
}

list_profiles() {
  python3 - "$1" <<'PY'
import json
from pathlib import Path
import sys

value = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
for profile_id, profile in value["profiles"].items():
    print(f"{profile_id}\t{profile['platform']}\t{profile['evidence_kind']}")
PY
}

profile_exists() {
  python3 - "$1" "$2" <<'PY'
import json
from pathlib import Path
import sys

value = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
raise SystemExit(0 if sys.argv[2] in value["profiles"] else 1)
PY
}

existing_runtime() {
  pgrep -af '(ros2 launch franka_motion|run_franka_isaac_sim\.py|isaac-sim(\.streaming)?\.sh|omni\.kit|gz sim)' \
    | grep -v -E '(pgrep -af|run_profile\.sh)' || true
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="$2"
      shift 2
      ;;
    --build-first)
      BUILD_FIRST="$2"
      shift 2
      ;;
    --allow-existing-runtime)
      ALLOW_EXISTING_RUNTIME="$2"
      shift 2
      ;;
    --artifact-root)
      ARTIFACT_ROOT="$2"
      shift 2
      ;;
    --list)
      LIST_ONLY="true"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_bool "--build-first" "${BUILD_FIRST}"
require_bool "--allow-existing-runtime" "${ALLOW_EXISTING_RUNTIME}"
if [[ "${ARTIFACT_ROOT}" != /* ]]; then
  echo "--artifact-root must be absolute" >&2
  exit 2
fi

resolve_franka_paths
if [[ "${LIST_ONLY}" == "true" ]]; then
  MATRIX="${FAMILY_ROOT}/franka_motion/config/runtime_validation/runtime_matrix.v3.json"
  list_profiles "${MATRIX}"
  exit 0
fi

if [[ "${BUILD_FIRST}" == "true" ]]; then
  require_franka_workspace
  env -i HOME="${HOME}" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
    WS_DIR="${WS_DIR}" FAMILY_ROOT="${FAMILY_ROOT}" ROS_SETUP_FILE="${ROS_SETUP_FILE}" \
    /bin/bash --noprofile --norc -c '
    set -e
    source "${ROS_SETUP_FILE}"
    cd "${WS_DIR}"
    source_paths=("${FAMILY_ROOT}")
    if [[ -d "${WS_DIR}/src" ]]; then
      if [[ "${FAMILY_ROOT}" == "${WS_DIR}/src/"* ]]; then
        source_paths=("${WS_DIR}/src")
      else
        source_paths+=("${WS_DIR}/src")
      fi
    fi
    colcon build --base-paths "${source_paths[@]}" --packages-up-to franka_motion \
      --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
  '
fi
source_franka_setup

MATRIX="$(matrix_path)"
if [[ ! -f "${MATRIX}" ]]; then
  echo "Installed runtime matrix not found: ${MATRIX}" >&2
  exit 2
fi
if ! profile_exists "${MATRIX}" "${PROFILE}"; then
  echo "Unknown runtime profile: ${PROFILE}" >&2
  list_profiles "${MATRIX}" >&2
  exit 2
fi

mkdir -p "${ARTIFACT_ROOT}/state" "${ARTIFACT_ROOT}/runs"
STATE_FILE="${ARTIFACT_ROOT}/state/${PROFILE}.json"
if [[ -f "${STATE_FILE}" ]]; then
  CLEANUP_DIR="${ARTIFACT_ROOT}/runs/${PROFILE}-cleanup-$(date -u +%Y%m%dT%H%M%SZ)-${BASHPID}"
  echo "Cleaning interrupted token registered in ${STATE_FILE}"
  ros2 run franka_motion franka_runtime_harness cleanup \
    --matrix "${MATRIX}" \
    --profile "${PROFILE}" \
    --artifact-dir "${CLEANUP_DIR}" \
    --state-file "${STATE_FILE}"
fi

CONFLICTS="$(existing_runtime)"
if [[ -n "${CONFLICTS}" && "${ALLOW_EXISTING_RUNTIME}" != "true" ]]; then
  echo "Another ROS/simulator runtime is active; stop it before validation:" >&2
  printf '%s\n' "${CONFLICTS}" >&2
  exit 3
fi

export FRANKA_ISAAC_PYTHON="${FRANKA_ISAAC_PYTHON:-${ISAAC_SIM_ROOT:+${ISAAC_SIM_ROOT}/python.sh}}"
export FRANKA_ISAAC_PYTHON="${FRANKA_ISAAC_PYTHON:-python3}"
if [[ -z "${GITHUB_SHA:-}" ]] && REVISION="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null)"; then
  export GITHUB_SHA="${REVISION}"
fi

RUN_DIR="${ARTIFACT_ROOT}/runs/${PROFILE}-$(date -u +%Y%m%dT%H%M%SZ)-${BASHPID}"
mkdir -p "${RUN_DIR}"
echo "Runtime artifacts: ${RUN_DIR}"

cleanup_registered_token() {
  local exit_code=$?
  if [[ -f "${STATE_FILE}" ]]; then
    ros2 run franka_motion franka_runtime_harness cleanup \
      --matrix "${MATRIX}" \
      --profile "${PROFILE}" \
      --artifact-dir "${RUN_DIR}/exit-cleanup" \
      --state-file "${STATE_FILE}" >/dev/null 2>&1 || true
  fi
  return "${exit_code}"
}
trap cleanup_registered_token EXIT

ros2 run franka_motion franka_runtime_harness run \
  --matrix "${MATRIX}" \
  --profile "${PROFILE}" \
  --artifact-dir "${RUN_DIR}" \
  --state-file "${STATE_FILE}"

trap - EXIT
echo "Runtime result: ${RUN_DIR}/runtime-result.v2.json"
