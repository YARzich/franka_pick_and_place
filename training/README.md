<!-- Responsibility: Explain the portable offline tool sources, preserved training evidence and reproduction limits. -->

# Offline cube training tools

This source bundle accompanies the six checkpoints in
[`franka_motion/models`](../franka_motion/models/README.md). It is not a ROS
package and is not installed or run by the manipulation runtime. Keep
`training/` beside `franka_motion/` in the source checkout; this works both in
the standalone repository and inside the original monorepo's `manipulation/`.

The six original tools were copied from this project's `perception_training`
working tree, including its saved local edits, without changing that project.
That package declares MIT for its source. The project [MIT license](../LICENSE)
covers these project-owned scripts; checkpoints, upstream Ultralytics code and
simulator assets retain their separate [terms](../franka_motion/models/THIRD_PARTY_NOTICES.md).

## What is preserved

[`provenance.yaml`](provenance.yaml) records the original relative file names,
SHA-256 hashes before copying, hashes of the adapted copies, and the scope of
each adaptation. These are current source snapshots. An exact historical
script revision for each training run was not retained and is not invented.

The six recipes preserve **all 109 saved settings** from each run's `args.yaml`.
Only four path fields (`model`, `data`, `project`, `save_dir`) were replaced with
explicit placeholders. Dataset manifests preserve the recorded train/validation
lists and class names, with their absolute paths replaced by `${DATASETS_ROOT}`.
Original YAML hashes are retained in the provenance inventory.

| Bundled checkpoint | Saved recipe | Dataset manifest |
|---|---|---|
| `cube_yolo26n_isaac_seg_v1.pt` | [v1](recipes/cube_yolo26n_isaac_seg_v1.yaml) | [synthetic segmentation v1](datasets/cube_synthetic_yolo_seg_v1/data.yaml) |
| `cube_yolo26n_isaac_seg_v2.pt` | [v2](recipes/cube_yolo26n_isaac_seg_v2.yaml) | [synthetic segmentation v2](datasets/cube_synthetic_yolo_seg_v2/data.yaml) |
| `cube_yolo26n_isaac_seg_v3.pt` | [v3](recipes/cube_yolo26n_isaac_seg_v3.yaml) | [hybrid lighting v3](datasets/cube_hybrid_lighting_v3/data_with_replicator.yaml) |
| `cube_yolo26n_isaac_seg_v4c.pt` | [v4c](recipes/cube_yolo26n_isaac_seg_v4c.yaml) | [hybrid hard v4b](datasets/cube_hybrid_v4b_hard/data.yaml) |
| `cube_yolo26n_isaac_v2.pt` | [bbox v2](recipes/cube_yolo26n_isaac_v2.yaml) | [synthetic detection v2](datasets/cube_synthetic_yolo_v2/data.yaml) |
| `cube_yolo26s_isaac_seg_v5s.pt` | [v5s](recipes/cube_yolo26s_isaac_seg_v5s.yaml) | [hybrid balanced v5s](datasets/cube_hybrid_v5s_640_balanced/data.yaml) |

Input images, labels and the upstream base checkpoints are not included here.
The v2/v3 runs start from the bundled `cube_yolo26n_isaac_seg_v1.pt`; place that
file under the selected weights root. The v4c recipe needs the intermediate
`cube_yolo26n_hard_shadow_seg_v4b2/weights/best.pt`, which is not one of the six
bundled checkpoints. These inputs are identified explicitly, not downloaded or
substituted by the tools.

## Prepare a saved recipe without training

Use Python with PyYAML installed. The helper does not import Ultralytics,
PyTorch, image libraries or Isaac, contact the network, or start training.
Replace the example paths with your local directories:

```bash
export MANIPULATION_SRC="/path/to/source-checkout"
export TRAINING="$MANIPULATION_SRC/training"
python3 "$TRAINING/scripts/prepare_recipe.py" \
  --checkpoint cube_yolo26s_isaac_seg_v5s.pt \
  --datasets-root /path/to/datasets \
  --weights-root /path/to/input-weights \
  --runs-root /path/to/new-training-runs \
  --output-dir /path/to/new-prepared-config \
  --check-inputs
```

The tool creates `data.yaml` and `train.yaml` in a new directory, refuses to
overwrite an existing destination, and prints a native `yolo train cfg=...`
command. Inspect the files before intentionally running that command in the
chosen YOLO environment. The derived historical `save_dir` is omitted from the
prepared config so that its `project` and `name` select the new output location;
the archived recipe still contains the original normalized field.

`--check-inputs` verifies that the local input checkpoint and every referenced
train/validation path exist. Omit it only to inspect the bound YAML without
datasets; do not run the printed command until the required inputs are ready.
Path checks do not deserialize models or validate every image and label.

The preserved recipes contain `amp: true`. The actual Ultralytics training
runtime may fetch auxiliary AMP-checker weights or fonts even when the requested
input checkpoint exists. Full offline execution requires preparing those assets
in the selected environment. Recipe preparation makes no claim that the upstream
training process is network-free. Changing AMP, device, batch size or another
setting is a new experiment and should be recorded as such.

For an explicitly configured new experiment, the smaller historical CLI wrapper
is also available:

```bash
"$YOLO_PYTHON" "$TRAINING/scripts/train_cube_yolo.py" \
  --task segment --data /path/to/data.yaml \
  --weights /path/to/local/base-model.pt \
  --project /path/to/new-training-runs --name cube_experiment
```

Set `YOLO_PYTHON` to your chosen environment's interpreter. This wrapper rejects
missing data/base-weight files before importing Ultralytics and never switches
to another private virtual environment. Its CLI defaults are not a substitute
for the complete saved recipes above.

## Dataset tools

| Tool | Input and output |
|---|---|
| `generate_cube_synthetic_dataset.py` | Renders the sibling Franka scene in Isaac Sim and creates YOLO detection or segmentation images/labels. |
| `create_cube_screenshot_seg_dataset.py` | Reads explicit `--source_dir` directories, extracts cube masks and writes a new segmentation dataset. |
| `augment_yolo_seg_photometric.py` | Reads `--source_dataset`, writes lighting/color/noise variants to `--output_dir` and preserves labels. |
| `augment_yolo_seg_hard_cases.py` | Reads `--source_dataset`, writes shadow, near/far, edge and warp variants to `--output_dir`. |
| `augment_yolo_seg_edges.py` | Adds edge/corner variants **inside** the selected `--dataset_root`; use a working copy when preserving original data. |
| `train_cube_yolo.py` | Fine-tunes an explicitly supplied local checkpoint with Ultralytics. |

Each tool's `--help` works without its image, ML or simulator dependencies.
Image-processing tools require NumPy and OpenCV when actually run. Outputs use
`images/{train,val}`, `labels/{train,val}` and class `0: cube`. `--overwrite`
replaces the tool's documented output, so select separate input and output
directories for augmentation and screenshot extraction.

The generator must run in an existing Isaac Sim Python environment:

```bash
"$ISAAC_PYTHON" "$TRAINING/scripts/generate_cube_synthetic_dataset.py" \
  --task segment --output_dir /path/to/new-synthetic-dataset \
  --num_train 4000 --num_val 800 --seed 42 --headless
```

This is an example of a new generation run, not a recovered historical command.
Set `ISAAC_PYTHON` explicitly. The generator reuses the sibling
`franka_motion_runtime` bootstrap, discovers the SDK in that interpreter, and
uses the current `worlds/isaac`, `worlds/gazebo` and `description/gazebo` resources.
No installed ROS overlay is required for these source helpers. An explicit
`--install_path` remains available for a resource-root override; no personal SDK
path is guessed. The dataset output is explicit, and runtime preparation happens
before output creation so that a re-exec does not leave a partial dataset.
It does not accept the SDK license or remove shared crash dumps.

## Dataset provenance and limits

The saved manifests show three direct synthetic datasets and three composite
datasets. The lighting v3 composite names synthetic, photometric, Gazebo
screenshot and composite-image inputs. The v4b/v5s manifests additionally name
the `cube_problem*` and hard-augmentation datasets; v5s references 11 training
directories and 5 validation directories. The six YAML files record the exact
observed lists.

These directory references and the current generation/augmentation scripts
describe the available preparation pipeline. They do not establish the origin
of every image, exact historical generation commands, a historical scene asset
revision, or a complete audit of split independence. Reusing this bundle does
not promise bit-for-bit recovery of the supplied checkpoints. Record the input
dataset versions, actual CLI arguments, environment and results for each new run.

## Dependency versions and checks

Checkpoint metadata identifies **Ultralytics 8.4.19**. The direct package versions
in [requirements.txt](requirements.txt) were observed in the local YOLO
environment while preparing this bundle. Its Python was 3.12, PyTorch reported
`2.10.0+cu128`, and torchvision package metadata reported `0.25.0`. Select and
install the appropriate PyTorch/torchvision wheels separately before the remaining
requirements. This records a reference environment, not the missing complete
historical lock. Isaac uses its own SDK environment; do not install the YOLO pins
into it. No dependency, SDK, weights or dataset is downloaded by packaging.

The quick checks use only the standard library, pytest and PyYAML:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -B -m pytest -q \
  -p no:cacheprovider "$TRAINING/tests"
```

They check CLI inspection without optional libraries, explicit local inputs,
portable resource discovery, recipe binding and preservation, input preflight,
output ownership, and YAML-sensitive paths. They do not run training, render
datasets, evaluate model quality or validate the current Isaac SDK runtime.
