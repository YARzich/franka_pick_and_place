<!-- Responsibility: Record observed fine-tuning settings and dataset provenance without machine-specific paths. -->

# Saved YOLO training evidence

All six bundled checkpoints are fine-tuned Ultralytics models, not models
claimed to have been designed from scratch. The project owner confirmed this.
The settings below were read from the locally retained `runs/train/<run>/args.yaml`
files and cross-checked against checkpoint path metadata without executing pickle.
Each checkpoint identifies Ultralytics 8.4.19.

## Saved run settings

`epochs` is the configured maximum; early stopping may have ended a run earlier.
This table is an inventory, not a promise of bit-for-bit reproducibility.

| Checkpoint | Training run | Input weights | Dataset manifest | Epoch limit | Image size | Batch | Optimizer | Seed |
|---|---|---|---|---:|---:|---:|---|---:|
| `cube_yolo26n_isaac_seg_v1.pt` | `cube_yolo26n_isaac_seg_v1` | `yolo26n-seg.pt` | `cube_synthetic_yolo_seg_v1/data.yaml` | 100 | 640 | 16 | auto | 42 |
| `cube_yolo26n_isaac_seg_v2.pt` | `cube_yolo26n_isaac_seg_v2` | `cube_yolo26n_isaac_seg_v1.pt` | `cube_synthetic_yolo_seg_v2/data.yaml` | 80 | 640 | 16 | auto | 84 |
| `cube_yolo26n_isaac_seg_v3.pt` | `cube_yolo26n_lighting_seg_v3` | `cube_yolo26n_isaac_seg_v1.pt` | `cube_hybrid_lighting_v3/data_with_replicator.yaml` | 60 | 640 | 16 | AdamW | 5302 |
| `cube_yolo26n_isaac_seg_v4c.pt` | `cube_yolo26n_hard_shadow_seg_v4c_short` | `cube_yolo26n_hard_shadow_seg_v4b2/weights/best.pt` | `cube_hybrid_v4b_hard/data.yaml` | 8 | 768 | 40 | AdamW | 5489 |
| `cube_yolo26n_isaac_v2.pt` | `cube_yolo26n_isaac_v2` | `yolo26n.pt` | `cube_synthetic_yolo_v2/data.yaml` | 40 | 640 | 16 | auto | 42 |
| `cube_yolo26s_isaac_seg_v5s.pt` | `cube_yolo26s_seg_v5s_640_balanced` | `yolo26s-seg.pt` | `cube_hybrid_v5s_640_balanced/data.yaml` | 50 | 640 | 32 | AdamW | 5620 |

## Dataset pipeline

The saved dataset YAML files reference local cube datasets: synthetic Isaac Sim
renders, Gazebo screenshots and their lighting/color/edge augmentations.
The [training source bundle](../../training/README.md) preserves the relevant
`perception_training` tools in this source distribution:

- `generate_cube_synthetic_dataset.py`: Isaac scene rendering and automatic labels;
- `create_cube_screenshot_seg_dataset.py`: cube segmentation labels from screenshots;
- `augment_yolo_seg_edges.py`, `augment_yolo_seg_photometric.py` and
  `augment_yolo_seg_hard_cases.py`: augmentation of existing images and labels;
- `train_cube_yolo.py`: calls Ultralytics `YOLO(...).train(...)`.

The latest `cube_hybrid_v5s_640_balanced` manifest combines synthetic cube sets,
photometric variants, Gazebo screenshots, and the `cube_problem*` hard-example
sets. No external downloaded image corpus is referenced in that manifest.
This describes the observed files and scripts; directory names alone do not
prove the provenance of every individual image or the version of every scene asset.

## Public packaging

Keep the upstream model license and notices: fine-tuning does not justify
relabeling Ultralytics checkpoints as MIT. The source bundle includes the six
tools above, complete saved run settings, normalized dataset manifests, source
hashes and a helper that resolves portable paths without starting training.
See [training/provenance.yaml](../../training/provenance.yaml) for the snapshot
origin and changes made for publication. The original training workspace was
not modified. This is the available source snapshot, not a claim that the
exact historical script bytes of every training run were preserved.

This evidence resolves what the local files can establish about training. It
does not claim that the model license universally requires publishing every
historical training image. See [license notices](THIRD_PARTY_NOTICES.md) for
the distribution scope and original sources. Dataset images, intermediate
checkpoints and third-party SDKs are not bundled or downloaded automatically;
the training README states which inputs the saved recipes need.
