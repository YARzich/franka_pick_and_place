<!-- Responsibility: Document Panda table-camera model assets and how to recreate the external perception backends on a clean machine. -->

# Runtime models

Vendored checkpoints in this directory:

- `cube_yolo26s_isaac_seg_v5s.pt`: default project-trained detector used by
  the Panda table-camera scenario
- `cube_yolo26n_isaac_seg_v3.pt`: previous YOLO segment model kept as a stable baseline
- `cube_yolo26n_isaac_seg_v1.pt`: older YOLO segment model kept for comparison
- `cube_yolo26n_isaac_seg_v2.pt`: intermediate segment-model checkpoint
- `cube_yolo26n_isaac_seg_v4c.pt`: hard-shadow segment-model checkpoint
- `cube_yolo26n_isaac_v2.pt`: legacy bbox-only YOLO model
- `foundationpose/cube_5cm.obj`: cube mesh shared by MegaPose, GigaPose and
  FoundationPose modes

These project-trained YOLO checkpoints stay committed: retraining took many
GPU-hours, there is no alternative download source, and nothing in the
table-camera scenario works without a detector. The source distribution includes
[training tools, saved run settings and dataset manifests](../../training/README.md).
They are offline tools, separate from the seven ROS packages.

All six checkpoints contain an **AGPL-3.0** license marker from Ultralytics
8.4.19. Fine-tuning does not make them MIT-licensed project assets. Their
unchanged SHA-256 hashes, observed provenance and distribution terms
are in [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md); the corresponding
license text is in [`LICENSES/AGPL-3.0.txt`](LICENSES/AGPL-3.0.txt). The package's
MIT license applies to its own source, not to these third-party-derived weights.
Distributing or deploying a combined YOLO application also requires following
the applicable AGPL terms; the source files' MIT notices do not waive them.
The checkpoint metadata retains historical training paths; inference uses the
explicit model path and does not require those training directories.

The robot-independent inference adapters live in `manipulation_perception`;
scenario-specific checkpoints and cube geometry remain in `franka_motion`.

External backends below are **not** vendored. The matching launch arguments
have no personal path baked in: each one falls back to a same-named
`FRANKA_GPU_*` environment variable, and `ros2 launch` fails closed and names
the missing argument/variable instead of using a path that only exists on one
machine. Set these once (e.g. in a small file such as `~/.franka_gpu_env`
sourced from your shell profile) and every launch command picks them up
without repeating them on the command line; an explicit `argname:=value` on
the command line still overrides the environment.

## YOLO runtime

Required: `yolo_python` argument, or `FRANKA_GPU_YOLO_PYTHON` environment
variable.

The detector runs in a separate Python environment with `ultralytics`; the
system ROS Python is not used for inference. Verified environment: Python
3.12, `torch==2.10.0+cu128`, `ultralytics==8.4.19`.

```bash
python3 -m venv ~/venvs/yolo_full
~/venvs/yolo_full/bin/pip install 'torch==2.10.0+cu128' --index-url https://download.pytorch.org/whl/cu128
~/venvs/yolo_full/bin/pip install 'ultralytics==8.4.19'
```

Pass `yolo_python:=~/venvs/yolo_full/bin/python`. The model arguments
(`yolo_model_path`, `yolo_mask_model_path`) default to the vendored checkpoint
above and normally do not need to be set.

These commands pin the two verified inference packages; they are not a complete
environment lock. Preserve `pip freeze` from the working environment when exact
transitive-dependency reproduction is required. For the source backends below,
also record checkout commits and any local patches; cloning a moving default
branch alone does not reproduce an earlier validation run.

## External pose source checkouts

Use the revisions in the [source manifest](../../manipulation_perception/dependencies.repos)
for the documented environments. From a ROS development shell with `vcs`
installed, set `MANIPULATION_SRC` to the family root as in INSTALL and run:

```bash
export BACKEND_SRC="$HOME/manipulation_backends"
mkdir -p "$BACKEND_SRC"
vcs import "$BACKEND_SRC" < "$MANIPULATION_SRC/manipulation_perception/dependencies.repos"
```

This explicitly fetches the three source repositories, not pretrained models.
It creates `third_party/{megapose6d,gigapose,dinov2}` below `BACKEND_SRC`.
Read the [version-specific licenses](../../manipulation_perception/THIRD_PARTY_NOTICES.md)
before preparing an optional backend. Updating these revisions needs a separate
compatibility check; default-branch clones do not reproduce this environment.

## MegaPose

Required: `megapose_python`/`megapose_data_dir` arguments, or
`FRANKA_GPU_MEGAPOSE_PYTHON`/`FRANKA_GPU_MEGAPOSE_DATA_DIR` environment
variables.

- `mode:=megaPose` uses the vendored `models/foundationpose/cube_5cm.obj` mesh
  by default. The mesh is stored in meters; `megapose_detection_node.py` writes
  a scaled millimeter copy into its own temporary runtime workspace. Shared
  `<megapose_data_dir>` remains the source of model assets; inference does not
  write an `examples` directory there. The node cleans up its private workspace.
- MegaPose models are large and are downloaded into `<megapose_data_dir>`
  instead of being vendored.
- Install [`megapose6d`](https://github.com/megapose6d/megapose6d) into its own
  Python environment (micromamba/conda) from
  `$BACKEND_SRC/third_party/megapose6d`, using `pip install -e . --no-deps`
  inside that environment after installing its dependencies.
- Blackwell-generation GPUs (e.g. RTX 5070 Ti) need newer CUDA wheels than the
  upstream environment file: the verified environment pins
  `torch==2.7.1+cu128`, `torchvision==0.22.1+cu128` and `numpy<2` on Python 3.9.

Download the models with the environment active:

```bash
export MEGAPOSE_DATA_DIR=<megapose_data_dir>
python -m megapose.scripts.download --megapose_models
```

Useful launch arguments:

- `megapose_python`: Python executable with `megapose` installed.
- `megapose_data_dir`: shared model-asset directory; workers receive a private
  `MEGAPOSE_DATA_DIR` with links to the required assets.
- `megapose_model`: `auto` by default; with the default depth mode this selects `megapose-1.0-RGBD`.
- `megapose_depth_mode`: `on` by default; use `off` for RGB-only or `icp` for the experimental RGB+ICP path.
- `megapose_persistent_worker`: keeps the MegaPose model loaded in a long-running worker process.
- `megapose_bsz_images`: lower values use less VRAM, higher values may run the coarse stage faster.

## GigaPose

Required: `gigapose_python`/`gigapose_repo_dir`/`gigapose_dinov2_repo_dir`/
`gigapose_data_dir` arguments, or `FRANKA_GPU_GIGAPOSE_PYTHON`/
`FRANKA_GPU_GIGAPOSE_REPO_DIR`/`FRANKA_GPU_DINOV2_REPO_DIR`/
`FRANKA_GPU_GIGAPOSE_DATA_DIR` environment variables.

`mode:=gigaPose` runs the [nv-nguyen/gigapose](https://github.com/nv-nguyen/gigapose)
coarse retrieval stage and refines its hypotheses with the MegaPose refiner, so
the MegaPose section above must be completed first (the refiner checkpoints are
read from `<megapose_data_dir>/megapose-models/`).

The DINOv2 revision recorded in `manipulation_perception/dependencies.repos`
uses **CC-BY-NC-4.0**, despite newer upstream revisions having a different
license. Review the version-specific
[`perception notices`](../../manipulation_perception/THIRD_PARTY_NOTICES.md)
before preparing this optional backend. The pinned checkout is not a
permissively licensed default for commercial use; switching to a later
Apache-2.0 revision requires a separate compatibility check.

1. Use the pinned checkouts above. Set `gigapose_repo_dir` to
   `$BACKEND_SRC/third_party/gigapose` and `gigapose_dinov2_repo_dir` to
   `$BACKEND_SRC/third_party/dinov2`.
2. Create the GigaPose environment following that checkout's README (its
   `environment.yml` targets Python 3.9). On Blackwell-generation GPUs replace
   the pinned torch with CUDA 12.8 wheels; the verified environment uses
   `torch==2.7.1+cu128` on Python 3.9.
3. Download the pretrained checkpoint per the gigapose README and place it at
   `<gigapose_data_dir>/pretrained/gigaPose_v1.ckpt` (or pass an explicit
   `gigapose_checkpoint_path`).
4. Render BOP-style templates for the object with the gigapose template
   tooling. The launch validation expects this layout under
   `<gigapose_data_dir>` for `gigapose_dataset_name:=franka_cube` and
   `gigapose_object_id:=1`:

```text
<gigapose_data_dir>/
  pretrained/gigaPose_v1.ckpt
  datasets/franka_cube/models/models_info.json
  datasets/templates/franka_cube/000001/          # rendered template views
  datasets/templates/franka_cube/object_poses/000001.npy
```

Keep `gigapose_dinov2_repo_dir` explicit for the pinned setup. An empty value
allows the upstream Torch Hub download path and does not reproduce the
revision or license recorded in this manifest.

## ESS

Required: `ess_engine_file_path` argument, or `FRANKA_GPU_ESS_ENGINE`
environment variable (used by `mode:=ess`, `mode:=yolo` depth,
`mode:=foundationPose`, `mode:=megaPose` and `mode:=gigaPose`).

The ESS stereo-depth engine comes from NVIDIA Isaac ROS; follow the official
model preparation in the
[isaac_ros_ess quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_ess/index.html)
(download the DNN stereo disparity model and build the TensorRT engine for
your GPU). The verified engine is
`~/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx_trt10.13/ess.engine`;
pass the resulting path as `ess_engine_file_path`.

## FoundationPose

Required: `foundationpose_refine_model_file_path`/
`foundationpose_refine_engine_file_path`/
`foundationpose_score_model_file_path`/
`foundationpose_score_engine_file_path` arguments, or
`FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL`/
`FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE`/
`FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL`/
`FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE` environment variables.

These are mandatory launch inputs, but the files themselves are only
opened by `mode:=foundationPose`; other modes accept paths to not-yet-created
engines. Follow the official
[isaac_ros_foundationpose quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html)
to download the refine/score ONNX models and build both TensorRT engines. The
verified layout is `~/isaac_ros_assets/models/foundationpose/{refine_model.onnx,refine_trt_engine.plan,score_model.onnx,score_trt_engine.plan}`.
The object mesh and texture default to the vendored cube assets in this
directory.
