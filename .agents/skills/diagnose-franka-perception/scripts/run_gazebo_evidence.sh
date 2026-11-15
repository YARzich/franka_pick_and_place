#!/usr/bin/env bash
set -euo pipefail

# Responsibility: Bind explicitly configured GPU/model assets to the
# project-owned Gazebo perception evidence profile.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_RUNNER="${SCRIPT_DIR}/../../validate-franka-manipulation/scripts/run_profile.sh"
# shellcheck source=../../validate-franka-manipulation/scripts/runtime_environment.sh
source "${SCRIPT_DIR}/../../validate-franka-manipulation/scripts/runtime_environment.sh"

BUILD_FIRST="true"
PROFILE="gazebo_gpu_perception_v2"
FORWARDED_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-first)
      if [[ $# -lt 2 || "$2" == --* ]]; then
        echo "$1 requires a value" >&2
        exit 2
      fi
      BUILD_FIRST="$2"
      shift 2
      ;;
    --profile)
      if [[ $# -lt 2 || "$2" == --* ]]; then
        echo "$1 requires a value" >&2
        exit 2
      fi
      PROFILE="$2"
      shift 2
      ;;
    --artifact-root|--allow-existing-runtime)
      if [[ $# -lt 2 || "$2" == --* ]]; then
        echo "$1 requires a value" >&2
        exit 2
      fi
      FORWARDED_ARGS+=("$1" "$2")
      shift 2
      ;;
    --help|-h)
      echo "Usage: run_gazebo_evidence.sh [--build-first <true|false>] [--profile <gpu-matrix-profile>] [run_profile options]"
      echo "Profiles needing the FRANKA_GPU_* bindings: gazebo_gpu_perception_v2 (default), gazebo_detected_round_trip_v2."
      exit 0
      ;;
    *)
      echo "Unsupported argument: $1" >&2
      echo "Only --build-first, --profile, --artifact-root and --allow-existing-runtime are accepted." >&2
      exit 2
      ;;
  esac
done

if [[ "${BUILD_FIRST}" != "true" && "${BUILD_FIRST}" != "false" ]]; then
  echo "--build-first must be true or false" >&2
  exit 2
fi
if [[ ! -x "${VALIDATION_RUNNER}" ]]; then
  echo "Repository validation runner is missing: ${VALIDATION_RUNNER}" >&2
  exit 2
fi

resolve_franka_paths
FRANKA_SHARE="${FAMILY_ROOT}/franka_motion"
export FRANKA_GPU_PERCEPTION_CONTRACT="${FRANKA_GPU_PERCEPTION_CONTRACT:-${FRANKA_SHARE}/config/runtime_validation/gpu_perception/gazebo_gigapose_cube.v1.json}"
export FRANKA_GPU_YOLO_MODEL="${FRANKA_GPU_YOLO_MODEL:-${FRANKA_SHARE}/models/cube_yolo26s_isaac_seg_v5s.pt}"
export FRANKA_GPU_CUBE_MESH="${FRANKA_GPU_CUBE_MESH:-${FRANKA_SHARE}/models/foundationpose/cube_5cm.obj}"

required_bindings=(
  FRANKA_GPU_ESS_ENGINE FRANKA_GPU_YOLO_PYTHON
  FRANKA_GPU_MEGAPOSE_PYTHON FRANKA_GPU_MEGAPOSE_DATA_DIR
  FRANKA_GPU_GIGAPOSE_PYTHON FRANKA_GPU_GIGAPOSE_REPO_DIR
  FRANKA_GPU_DINOV2_REPO_DIR FRANKA_GPU_GIGAPOSE_DATA_DIR
  FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE
  FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE
)
missing_bindings=()
for binding in "${required_bindings[@]}"; do
  if [[ -z "${!binding:-}" ]]; then
    missing_bindings+=("${binding}")
  fi
done
if [[ ${#missing_bindings[@]} -gt 0 ]]; then
  printf 'Set the following environment variables for this GPU profile:\n' >&2
  printf '  %s\n' "${missing_bindings[@]}" >&2
  printf 'See %s/models/README.md for the backend setup.\n' "${FRANKA_SHARE}" >&2
  exit 2
fi

export FRANKA_GPU_GIGAPOSE_CHECKPOINT="${FRANKA_GPU_GIGAPOSE_CHECKPOINT:-${FRANKA_GPU_GIGAPOSE_DATA_DIR}/pretrained/gigaPose_v1.ckpt}"
export FRANKA_GPU_MEGAPOSE_COARSE_CHECKPOINT="${FRANKA_GPU_MEGAPOSE_COARSE_CHECKPOINT:-${FRANKA_GPU_MEGAPOSE_DATA_DIR}/megapose-models/coarse-rgb-906902141/checkpoint.pth.tar}"
export FRANKA_GPU_MEGAPOSE_REFINER_CHECKPOINT="${FRANKA_GPU_MEGAPOSE_REFINER_CHECKPOINT:-${FRANKA_GPU_MEGAPOSE_DATA_DIR}/megapose-models/refiner-rgbd-288182519/checkpoint.pth.tar}"
# FoundationPose model arguments are mandatory launch inputs even for
# mode:=gigaPose; the files themselves are only opened by mode:=foundationPose.

exec "${VALIDATION_RUNNER}" \
  --profile "${PROFILE}" \
  --build-first "${BUILD_FIRST}" \
  "${FORWARDED_ARGS[@]}"
