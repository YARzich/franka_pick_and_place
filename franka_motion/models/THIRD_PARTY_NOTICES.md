<!-- Responsibility: Record YOLO checkpoint identities, licenses and corresponding source locations. -->

# Model provenance and licenses

The six YOLO checkpoints in this directory were fine-tuned with Ultralytics
8.4.19. Each checkpoint's serialized metadata contains the license marker
`AGPL-3.0 (https://ultralytics.com/license)` and the version `8.4.19`.
These observations were made by reading the ZIP archive's pickle opcodes with
`pickletools`, without importing PyTorch or executing the serialized objects.
The checkpoint bytes have not been modified by publication preparation.

The [Ultralytics 8.4.19 source license][ultralytics-license] is copied verbatim
to [`LICENSES/AGPL-3.0.txt`](LICENSES/AGPL-3.0.txt). Ultralytics' own
[licensing guidance][ultralytics-guidance] also identifies its trained models
as AGPL-3.0 by default. The MIT license of this project's own source does not
relicense the checkpoints. A separately obtained commercial agreement cannot
be assumed from these files.

## Checkpoint identity

| File | SHA-256 |
|---|---|
| `cube_yolo26n_isaac_seg_v1.pt` | `a060dfe0a22db80bb11d157a252f90c6e78de1d4e60af8e247c8a0b0bfbca39e` |
| `cube_yolo26n_isaac_seg_v2.pt` | `21a73ad6bd489e7f1a52c2dc264542994a056df58cabfbc0abfc61c528ac8564` |
| `cube_yolo26n_isaac_seg_v3.pt` | `dd35119f5964ef6431aadcbbbfe2cd4c5837591df27ad84e79d2cef202df8a00` |
| `cube_yolo26n_isaac_seg_v4c.pt` | `252a5823116600686a77923abff0f40167419601d2e945f27cd073d0023972a5` |
| `cube_yolo26n_isaac_v2.pt` | `1bfbfd8369acdcb87df24ed0362f3b4c2a752103c10a5c4ee257e5b94f12ef31` |
| `cube_yolo26s_isaac_seg_v5s.pt` | `f9b7c0a4d30e4ce720a9da62d3b0b000fd5adde6e83c8d8f2a6e9320e486eaa4` |

The observed training inputs are listed by basename, not by historical machine
path. These metadata names identify the runs; they do not independently prove
the identity or license of the training corpus.

| Checkpoint | Input weights in metadata | Dataset directory in metadata |
|---|---|---|
| `cube_yolo26n_isaac_seg_v1.pt` | `yolo26n-seg.pt` | `cube_synthetic_yolo_seg_v1` |
| `cube_yolo26n_isaac_seg_v2.pt` | `cube_yolo26n_isaac_seg_v1.pt` | `cube_synthetic_yolo_seg_v2` |
| `cube_yolo26n_isaac_seg_v3.pt` | `cube_yolo26n_isaac_seg_v1.pt` | `cube_hybrid_lighting_v3` |
| `cube_yolo26n_isaac_seg_v4c.pt` | `cube_yolo26n_hard_shadow_seg_v4b2/weights/best.pt` | `cube_hybrid_v4b_hard` |
| `cube_yolo26n_isaac_v2.pt` | `yolo26n.pt` | `cube_synthetic_yolo_v2` |
| `cube_yolo26s_isaac_seg_v5s.pt` | `yolo26s-seg.pt` | `cube_hybrid_v5s_640_balanced` |

## Source distribution

This repository distributes the checkpoints together with the application
source and [training source bundle](../../training/README.md). The bundle
contains the six preparation/training tools, all six saved run configurations,
normalized dataset manifests and source hashes. [TRAINING.md](TRAINING.md)
describes the observed runs and dataset pipeline. The original ML framework
source is [Ultralytics v8.4.19][ultralytics-source]; it is an external pinned
dependency, not modified or vendored here. Its source includes the model
architecture and training implementation used by the local wrapper.

Keep the complete source tree, license texts and notices available alongside
these weights at the same repository revision. If publishing binary packages
or separate model downloads, provide the matching source distribution at the
download location as well. The standalone ROS install tree by itself is not
this complete source distribution. Build and runtime instructions are in
[INSTALL.md](../../INSTALL.md) and [README.md](README.md).

A combined application using these models and Ultralytics is subject to the
applicable AGPL-3.0 terms, including corresponding-source and network-use
requirements where they apply. MIT notices on this project's independent
source files do not waive those terms. This source release does not authorize
future deployments to omit required source or notices.

The bundle records the available script snapshot, not the exact historical
script revision for every run. Dataset images, intermediate checkpoints and
SDKs are not included; the recipes identify their required inputs and do not
promise bit-for-bit retraining. Any later redistribution of training images
or scene resources must preserve their own terms. This inventory does not
assume that every historical training image is necessarily corresponding
source merely because it was used in training.

The simple cube geometry, material and constant-color texture in
`foundationpose/` are separate scenario resources. Their location does not
make them copies of the external NVlabs FoundationPose research code or models.

[ultralytics-license]: https://github.com/ultralytics/ultralytics/blob/v8.4.19/LICENSE
[ultralytics-source]: https://github.com/ultralytics/ultralytics/tree/v8.4.19
[ultralytics-guidance]: https://www.ultralytics.com/license
