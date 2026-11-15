<!-- Responsibility: Document public executables, ROS graph, parameters, timing, and failures. -->
# manipulation_perception API

The package provides generic adapters only. A robot/scenario package owns every
topic, frame, model, mesh, object identifier and external-runtime path.

## Executables

| Executable | Responsibility |
| --- | --- |
| `disparity_frame_fix_node.py` | Validate and repair stamped `DisparityImage` metadata. |
| `stereo_sgbm_pointcloud_node.py` | Local SGBM or external disparity to depth/cloud/table products. |
| `yolo_visualizer_node.py` | Optional YOLO/SAHI detection, stamped bbox, mask and debug output. |
| `megapose_detection_node.py` | Exact-input coordination and MegaPose/GigaPose inference. |
| `foundationpose_depth_clean_node.py` | Convert stamped depth to finite `32FC1`. |
| `foundationpose_input_frame_fix_node.py` | Explicitly rewrite paired image/calibration metadata. |
| `foundationpose_detection_to_pose_node.py` | Transform selected `Detection3D` into pose/markers. |
| `megapose_worker.py` | JSON-lines MegaPose worker for a dedicated interpreter. |
| `gigapose_worker.py` | JSON-lines GigaPose coarse worker. |
| `gigapose_refiner_worker.py` | JSON-lines pose-refinement worker. |
| `gpu_perception_evidence_probe.py` | Read-only, scenario-driven GPU perception evidence collector. |

All ROS nodes reject incomplete configuration before creating their processing
pipeline. Worker executables read one JSON object per line from stdin and emit
one flushed correlated response per non-shutdown line. Malformed and non-object
JSON produces `{"id": null, "ok": false, ...}` without terminating the worker.

## Stereo And Point Clouds

Required graph parameters:

- `left_camera_info_topic`, `disparity_topic`, `depth_topic`;
- `point_cloud_topic`, `octomap_point_cloud_topic`;
- `point_cloud_frame`, `workspace_frame`, `robot_base_frame`;
- `baseline_m`, `robot_reach_m`, `table_plane_coeffs_topic`.

Local SGBM additionally requires `left_image_topic`, `right_image_topic`,
`left_image_frame_id`, `right_image_frame_id`, and
`local_stereo_frame_id`. `local_stereo_frame_id` must equal
`left_image_frame_id`: generated XYZ coordinates physically use the left
optical frame. Left image, right image and left CameraInfo use an exact-time
three-way synchronizer. CameraInfo must match the left image by exact
`stamp + frame_id`; image/calibration resolutions must match. The retained
`sync_slop_sec` parameter must be zero.

External mode sets `use_external_disparity=true` and requires
`external_disparity_topic`. The outer disparity header, embedded image header
and CameraInfo header must be identical. Positive per-sample `f` and `t` are
authoritative; explicit zero selects validated CameraInfo/config fallbacks.

Optional stamped ROI filtering sets `use_detection_bbox_roi=true` and
`detection_bbox_stamped_topic`. A depth sample is matched only to a
`Detection2DArray` with the same positive stamp and frame. Missing, stale or
empty detection fails closed to empty/NaN public depth and clouds; the
unstamped compact bbox payload is never used for synchronization.

All floating-point stereo parameters must be finite at startup.
`detection_bbox_timeout_sec` must be positive and
`min_valid_disparity_px` non-negative. A negative `min_depth_m` normalizes to
zero; non-positive `max_depth_m` disables the upper bound, while an enabled
upper bound must be greater than `min_depth_m`.
`plane_normal_max_tilt_deg` must be in `[0, 90)`. OpenCV SGBM constraints and
the existing marker/workspace clamps retain their normalization semantics.

Input sensor subscriptions and high-bandwidth outputs use sensor-data QoS.
Depth is metres, disparity is pixels, point clouds are XYZ metres, and TF uses
the source message timestamp. `allow_latest_tf_fallback=false` is the default;
enabling it is an explicit scenario policy.
The stereo and detection-to-pose TF buffers share their owning node's ROS clock,
so a backward clock jump clears transforms from the previous epoch.

## YOLO Adapter

`yolo_visualizer_node.py` requires `image_topic` and an existing `model_path`.
When bbox publication is enabled it also requires `detection_bbox_topic` and
`detection_bbox_stamped_topic`; mask publication requires
`detection_mask_topic`. `detection_bbox_frame_id`, when supplied, must be a
canonical relative frame. The stamped `Detection2DArray` and optional mask
preserve the source image stamp; the compact `Int32MultiArray` bbox is legacy
output only.

Model thresholds, NMS, SAHI slicing and temporal confirmation have
backend-neutral defaults. Model/checkpoint files are never bundled here.
Temporal confirmation counts at most one hit per track in each processed frame,
including when overlapping candidates survive NMS and create new tracks.
Opening the optional control panel preserves exact configured values, including
values outside a slider's display range or precision. Only an edited control
updates its setting; other parameters retain their exact values. SAHI's
independent matching threshold follows IoU only after that control is edited.

## Pose Adapter

`megapose_detection_node.py` always requires:

- `image_topic`, `camera_info_topic`, `detection_bbox_stamped_topic`;
- `output_detections_topic`, canonical `output_frame_id`;
- `coarse_estimator` (`megapose` or `gigapose`) and `depth_mode`.

Any enabled depth mode also requires `depth_topic`. Related image, CameraInfo,
bbox, mask and depth inputs are cached by exact positive `stamp + frame_id`.
No latest-message or payload fallback exists. ROS clock jumps clear the cache
and invalidate in-flight generations; newer empty output suppresses late
positive inference for the same/older stamp.

MegaPose configuration requires an executable `megapose_python`, data
directory, model name, canonical `object_label`, mesh file and `mesh_units`
(`m` or `mm`). GigaPose additionally requires its Python executable, repository,
data directory, checkpoint, canonical dataset name and positive BOP object ID.
An empty `gigapose_checkpoint_path` resolves to
`<gigapose_data_dir>/pretrained/gigaPose_v1.ckpt`; an explicit path overrides
that convention. `gigapose_min_matching_score` is a finite threshold in
`[0, 1]` with generic default `0.0`. Missing, malformed or non-finite coarse
scores, and a best score below the threshold, reject the frame before
refinement. A finite backend `pose_data.score` in `[0, 1]` is published as the
`Detection3D` hypothesis score; only an absent backend score uses the configured
static fallback, while malformed scores fail closed.
Optional persistent workers use the same request/response schema as one-shot
workers. A worker that exits between requests has its remaining process group
terminated before replacement; concurrent shutdown prevents that replacement.
Persistent workers, one-shot workers and the MegaPose CLI share exclusive
process-group ownership. Exit observation retains the waitable leader until
the final group signal; cleanup closes signaling before reaping, so repeated
or concurrent shutdown cannot signal a reused PID/PGID. SIGTERM gives a live
leader a bounded grace period; after leader exit or that deadline, remaining
group members receive SIGKILL. Descendants that create another process group
are outside this ownership contract. One-shot/CLI output is captured in
temporary files, preserving stdout, stderr, exit status and timeout errors
without a concurrent implicit reap. Cleanup confirms leader reap, rather than
claiming that every descendant has already disappeared.
Runtime image writes are checked; write failure removes stale files
and aborts the request.
Each node owns a unique temporary runtime workspace, including MegaPose examples,
GigaPose inputs/predictions, derived refiner meshes and mutable caches. Worker
requests and CLI use that private data-root view; only immutable model/dataset
trees reference the configured asset roots. Template preprocessing and imported
MegaPose joblib/log directories also stay under the instance root. A pre-existing
GigaPose torch cache is copied lazily once into the workspace, so cache updates
cannot modify shared assets or force a second download; this may add startup I/O
for a large cache.
Cache preparation does not hold the process-state lock: stop closes admission
immediately, and a prepared environment cannot spawn a child after that stop.
Shutdown stops worker processes and joins inference/prewarm writers before
removing only that workspace. If a writer has not joined or any subprocess reap
was uncertain (including a handle already discarded after a timeout), its
workspace is retained. Repeated stop does not erase that uncertainty.
Abrupt process death may leave a temporary directory for later
operator cleanup. Two instances using the same object label and assets never
share mutable runtime paths.

## FoundationPose Adapters

- Input-frame adapter: `input_image_topic`, `input_camera_info_topic`,
  `output_image_topic`, `output_camera_info_topic`, `output_frame_id`.
- Depth cleaner: `input_depth_topic`, `output_depth_topic`.
- Detection-to-pose: `input_detections_topic`, `output_pose_topic`,
  `output_marker_topic`, `output_frame`, `marker_namespace`, and positive
  `fallback_object_size_m`.

The frame adapter rewrites metadata only because the scenario explicitly
asserts equivalence; it performs no image rotation or TF lookup. The
detection-to-pose adapter performs timestamped TF and publishes deletion
markers when no valid pose is available. Both result poses and bbox fallback
poses require finite XYZ coordinates and a finite unit quaternion within the
same `1e-3` norm tolerance as the MegaPose message builder. Accepted quaternion
roundoff is normalized on a copy. Invalid geometry produces no pose publication
or successful-output diagnostic event.

## Diagnostics And Failure Policy

ROS adapters publish standard `diagnostic_msgs/DiagnosticArray` through
`diagnostic_updater` on `/diagnostics`. Status covers input/output freshness,
frequency, processing latency, drops, inference failures and exact-sync
availability. Operator text is not a machine contract.

Every adapter using `PipelineObserver` exposes the same parameter contract:

| Parameter | Type | Default | Semantics |
| --- | --- | --- | --- |
| `diagnostics.enabled` | `bool` | `true` | Register the updater task and timer. `false` disables publication only; pipeline metrics are still recorded. |
| `diagnostics.require_success` | `bool` | `false` | Report WARN until the first successful output when no explicit ERROR/WARN threshold already applies. |
| `diagnostics.window_size` | `int` | `120` | Maximum retained steady-clock processing-latency samples; values below one normalize to one. Event counters remain cumulative. |
| `diagnostics.update_period_sec` | `double` | `1.0` | Updater period; values below `0.1` normalize to `0.1` seconds. |
| `diagnostics.minimum_samples` | `int` | `10` | Minimum latency samples for p95 checks and minimum inputs for drop-ratio checks; values below one normalize to one. |
| `diagnostics.warn_processing_latency_p95_sec` | `double` | `0.0` | WARN when windowed processing p95 is greater than or equal to this threshold after `minimum_samples`. |
| `diagnostics.error_processing_latency_p95_sec` | `double` | `0.0` | ERROR equivalent of the p95 threshold. |
| `diagnostics.warn_last_success_age_sec` | `double` | `0.0` | WARN when steady age since the last success reaches the threshold; after the same process uptime, no success also triggers it. |
| `diagnostics.error_last_success_age_sec` | `double` | `0.0` | ERROR equivalent of the last-success-age threshold. |
| `diagnostics.warn_drop_or_skip_ratio` | `double` | `0.0` | WARN when cumulative `drop_or_skip_count / input_count` reaches the threshold after `minimum_samples` inputs. Values are clamped to `[0, 1]`. |
| `diagnostics.error_drop_or_skip_ratio` | `double` | `0.0` | ERROR equivalent of the drop/skip ratio threshold. |
| `diagnostics.warn_error_count` | `int` | `0` | WARN when the cumulative processing error count reaches the threshold. |
| `diagnostics.error_error_count` | `int` | `0` | ERROR when the cumulative processing error count reaches the threshold. |

All threshold values normalize to non-negative values; zero disables that
individual check. If both WARN and ERROR thresholds of one metric are enabled,
ERROR must not be lower than WARN or startup fails. ERROR reasons take
precedence over WARN reasons. Latency, uptime and last-success age use a
monotonic clock; source ROS stamps are retained only as correlation labels.
The registered task is `perception/<pipeline_name>` (the standard updater wire
name also includes the ROS node prefix). `diagnostics.update_period_sec` is the
default passed to `diagnostic_updater`; an explicit standard
`diagnostic_updater.period` parameter override is the effective timer period.
The standard `diagnostic_updater.use_fqn` switch controls whether that node
prefix includes the namespace.

The following `DiagnosticStatus.values` names are a stable machine contract;
order is not significant and future producers may add extra keys:

```text
input_count, processed_count, output_count, error_count,
drop_or_skip_count, drop_or_skip_ratio, worker_timeout_count,
worker_restart_count, latency_sample_count, processing_latency_p50_sec,
processing_latency_p95_sec, processing_latency_max_sec,
last_success_age_sec, last_success_source, steady_uptime_sec
```

## GPU Perception Evidence Probe

`gpu_perception_evidence_probe.py` observes an already running pipeline. It
does not publish, start a backend, alter GPU state, or recover failed nodes.
Its required application arguments are:

```text
--contract ABSOLUTE_JSON
--artifact-dir ABSOLUTE_DIRECTORY
--run-token TOKEN
--revision 40_OR_64_CHARACTER_LOWERCASE_GIT_OBJECT_ID
--asset LOGICAL_NAME ABSOLUTE_EXISTING_PATH   # repeated for every contract asset
```

The contract schema is installed as
`share/manipulation_perception/config/gpu_perception_probe_contract.schema.v1.json`.
It contains exact topic names, message types, expected frame IDs, correlation
groups, exact full DiagnosticStatus names, freshness/sample policy, logical
asset names and a fixed read-only `nvidia-smi` query. Every topic belongs to a
correlation group. Scenario-specific asset paths are CLI values, never stored
in the installed contract or resolved through a home-directory search.

Each diagnostic entry also names `source_topic_role`. Its latest
`last_success_source` stamp and frame must match a valid message actually
observed on that role; the matched topic sample is persisted beside the
diagnostic delta. The GPU command is fixed to `/usr/bin/nvidia-smi` plus the
read-only identity query arguments, so a contract cannot substitute another
executable.

`capture_payload_digest` is explicit per topic. It should normally be false for
high-rate image/depth inputs and true only where a serialized digest adds value;
header/type/frame correlation remains strict either way. `maximum_rate_hz`
defines a bounded retention budget over `observation_timeout_sec`; exhausting
that declared budget fails the evidence instead of silently evicting a delayed
backend's source stamp.

The output is `gpu-perception-evidence.v1.json`, validated by the installed
`gpu-perception-evidence.schema.v1.json`. PASS requires one exact positive stamp
across each group, a fresh finite terminal `PoseStamped`, fresh diagnostics for
stereo/detector/pose backend/pose adapter, positive input/output/processed
deltas, no ERROR/STALE, no new processing errors, and zero worker
timeouts/restarts. The artifact records p50/p95/max, drops, freshness, GPU
identity, explicit file SHA256, revision and run token. It deliberately claims
runtime correlation only: `accuracy=false` and `replay=false`.

Invalid parameters, frames, identifiers, files or backend paths fail startup.
Invalid/zero stamps, mismatched frames, stale correlated inputs, failed image
writes and malformed worker responses fail closed; adapters do not substitute
latest sensor data. External backend exceptions are bounded by configured
timeouts and reported without publishing an uncorrelated pose.

Third-party runtime and asset terms are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md); pinned source revisions are in
[dependencies.repos](dependencies.repos).
