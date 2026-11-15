<!-- Responsibility: Record third-party resources redistributed by franka_motion. -->

# Third-party notices

## Franka Panda USD

The robot assets under `worlds/isaac/FrankaPanda/` are distributed under the
Apache License 2.0. The original license text is retained at
`worlds/isaac/FrankaPanda/franka-LICENSE.txt`; the package-level copy is
`LICENSES/Apache-2.0.txt`.

The [NVIDIA Isaac Sim 6.2 asset catalogue][franka-assets] identifies
`FrankaRobotics/FrankaPanda/franka.usd` and its accessories as Apache-2.0.
The bundled tree includes the associated `DetailedProps/Materials/Textures`
resources. The exact original asset-download revision was not recorded; the
retained license and catalogue entry are the available provenance evidence.

Isaac Sim itself is an external runtime and is not distributed by this ROS 2
package. The historical `franka_motion_sim_isaac/worlds/materials/` directory
is intentionally not copied or installed: supported USD roots do not reference
it, and the large textures have no confirmed open-source provenance.

## MoveIt Panda configuration

The adapted semantic and planner profiles under `config/moveit/` originate
from `moveit_resources_panda_moveit_config`, whose 3.1.0 manifest declares
`BSD` without a clause count. MoveIt's official source documentation identifies
BSD-3-Clause and includes `moveit_resources` in its scope. The pinned source
revision, authorship credits and local modifications are recorded in
[`config/moveit/THIRD_PARTY_NOTICES.md`](config/moveit/THIRD_PARTY_NOTICES.md).
[`LICENSES/BSD-3-Clause.txt`](LICENSES/BSD-3-Clause.txt) includes the attribution
and standard three conditions and disclaimer. It does not invent a copyright
holder or claim to be a verbatim copy of a package-level upstream LICENSE.

## Optional Ultralytics runtime

The table-camera example loads the external `ultralytics` Python package
through the replaceable `manipulation_perception` detector adapter. The
verified runtime version, 8.4.19, is AGPL-3.0 and is not distributed by this
package. All six bundled YOLO checkpoints also explicitly record AGPL-3.0 in
their metadata; they are not covered by this package's MIT license.

See [`models/THIRD_PARTY_NOTICES.md`](models/THIRD_PARTY_NOTICES.md) for hashes,
upstream source references, training provenance and source-distribution terms.
The source tree includes the training tools, six saved run configurations and
dataset manifests. Its license text is installed beside the weights at
`models/LICENSES/AGPL-3.0.txt`. A source-only MIT claim for the entire package
would therefore be incorrect. A combined YOLO application is subject to the
applicable AGPL terms; MIT notices on independent source files do not waive
those requirements. Other detectors can use the same stamped ROS contract.

[franka-assets]: https://docs.isaacsim.omniverse.nvidia.com/6.2.0/assets/usd_assets_robots_manipulator.html#frankarobotics
