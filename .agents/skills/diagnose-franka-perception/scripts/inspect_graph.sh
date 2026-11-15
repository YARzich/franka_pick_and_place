#!/usr/bin/env bash
set -euo pipefail

# Responsibility: Probe an already running table-camera graph stage by stage
# using the public stamped topic and diagnostic contracts.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../validate-franka-manipulation/scripts/runtime_environment.sh
source "${SCRIPT_DIR}/../../validate-franka-manipulation/scripts/runtime_environment.sh"
MODE="gigapose"
TIMEOUT_SEC=60

usage() {
  cat <<'USAGE'
Usage: inspect_graph.sh [--mode <stereo|ess|ecc|yolo|foundationpose|megapose|gigapose>] [--timeout <sec>]

The script attaches to an already running franka_motion table-camera graph.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2,,}"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
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

case "${MODE}" in
  stereo|ess|ecc|yolo|foundationpose|megapose|gigapose) ;;
  *)
    echo "Unsupported mode: ${MODE}" >&2
    usage >&2
    exit 2
    ;;
esac
if ! [[ "${TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--timeout must be a positive integer" >&2
  exit 2
fi

resolve_franka_paths
source_franka_setup
franka_package_share >/dev/null

declare -a TOPICS=(
  "/clock|rosgraph_msgs/msg/Clock|clock"
  "/table_stereo/left/image_raw|sensor_msgs/msg/Image|camera-left"
  "/table_stereo/right/image_raw|sensor_msgs/msg/Image|camera-right"
  "/table_stereo/left/camera_info|sensor_msgs/msg/CameraInfo|camera-info-left"
  "/table_stereo/right/camera_info|sensor_msgs/msg/CameraInfo|camera-info-right"
)

if [[ "${MODE}" == "ecc" || "${MODE}" == "stereo" ]]; then
  TOPICS+=("/table_stereo/disparity|stereo_msgs/msg/DisparityImage|stereo-disparity")
else
  TOPICS+=("/table_stereo/disparity_fixed|stereo_msgs/msg/DisparityImage|stereo-disparity")
fi
TOPICS+=(
  "/table_stereo/depth|sensor_msgs/msg/Image|stereo-depth"
  "/table_stereo/points|sensor_msgs/msg/PointCloud2|stereo-cloud"
)

if [[ "${MODE}" == "yolo" || "${MODE}" == "foundationpose" || "${MODE}" == "megapose" || "${MODE}" == "gigapose" ]]; then
  TOPICS+=("/table_stereo/cube_detection2d|vision_msgs/msg/Detection2DArray|detector")
fi
if [[ "${MODE}" == "foundationpose" ]]; then
  TOPICS+=(
    "/table_stereo/foundationpose/output|vision_msgs/msg/Detection3DArray|pose-backend"
    "/table_stereo/cube_pose|geometry_msgs/msg/PoseStamped|world-pose"
  )
elif [[ "${MODE}" == "megapose" || "${MODE}" == "gigapose" ]]; then
  TOPICS+=(
    "/table_stereo/megapose/output|vision_msgs/msg/Detection3DArray|pose-backend"
    "/table_stereo/cube_pose|geometry_msgs/msg/PoseStamped|world-pose"
  )
fi

topic_contracts_ready() {
  local graph
  graph="$(ros2 topic list -t 2>/dev/null)" || return 1
  local item topic expected_type role
  for item in "${TOPICS[@]}"; do
    IFS='|' read -r topic expected_type role <<<"${item}"
    if ! grep -Fx "${topic} [${expected_type}]" <<<"${graph}" >/dev/null; then
      return 1
    fi
  done
}

deadline=$((SECONDS + TIMEOUT_SEC))
until topic_contracts_ready; do
  if (( SECONDS >= deadline )); then
    echo "Topic/type contracts did not appear within ${TIMEOUT_SEC}s" >&2
    ros2 topic list -t >&2 || true
    exit 1
  fi
  sleep 1
done

sample_topic() {
  local topic="$1"
  local type="$2"
  local role="$3"
  local filter="True"
  if [[ "${role}" != "clock" ]]; then
    filter="bool(m.header.frame_id) and (m.header.stamp.sec != 0 or m.header.stamp.nanosec != 0)"
  fi
  case "${role}" in
    detector|pose-backend)
      filter="${filter} and len(m.detections) > 0"
      ;;
    camera-left|camera-right|stereo-depth|stereo-cloud)
      filter="${filter} and len(m.data) > 0"
      ;;
  esac
  if ros2 topic echo --once --timeout 10 --filter "${filter}" "${topic}" "${type}" >/dev/null 2>&1; then
    printf 'OK   %-18s %s [%s]\n' "${role}" "${topic}" "${type}"
    return 0
  fi
  printf 'FAIL %-18s %s: no valid fresh sample\n' "${role}" "${topic}" >&2
  return 1
}

failure=0
for item in "${TOPICS[@]}"; do
  IFS='|' read -r topic expected_type role <<<"${item}"
  sample_topic "${topic}" "${expected_type}" "${role}" || failure=1
done

declare -a DIAGNOSTICS=()
if [[ "${MODE}" == "ecc" || "${MODE}" == "stereo" ]]; then
  DIAGNOSTICS+=("stereo_sgbm_pointcloud: perception/stereo_depth_pointcloud")
else
  DIAGNOSTICS+=("ess_depth_pointcloud: perception/stereo_depth_pointcloud")
fi
if [[ "${MODE}" == "yolo" || "${MODE}" == "foundationpose" || "${MODE}" == "megapose" || "${MODE}" == "gigapose" ]]; then
  DIAGNOSTICS+=("yolo_visualizer: perception/yolo_detection")
fi
if [[ "${MODE}" == "foundationpose" ]]; then
  DIAGNOSTICS+=("foundationpose_detection_to_pose: perception/pose_estimation")
elif [[ "${MODE}" == "megapose" || "${MODE}" == "gigapose" ]]; then
  DIAGNOSTICS+=(
    "megapose_detection: perception/megapose_detection"
    "megapose_detection_to_pose: perception/pose_estimation"
  )
fi

for status_name in "${DIAGNOSTICS[@]}"; do
  if ros2 topic echo --once --timeout 10 \
    --filter "any(s.name == '${status_name}' and s.level < 2 for s in m.status)" \
    /diagnostics diagnostic_msgs/msg/DiagnosticArray >/dev/null 2>&1; then
    printf 'OK   diagnostic         %s\n' "${status_name}"
  else
    printf 'FAIL diagnostic         %s: missing or ERROR/STALE\n' "${status_name}" >&2
    failure=1
  fi
done

if [[ "${failure}" -ne 0 ]]; then
  exit 1
fi
echo "Perception graph contract is healthy for mode=${MODE}"
