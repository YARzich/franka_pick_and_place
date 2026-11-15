<!-- Responsibility: Record version-specific licensing of the external cuRobo runtime. -->

# Third-Party Notice

This package calls the external cuRobo Python API but does not redistribute the
cuRobo source, models or assets.

## Earlier evaluated checkout

- project: NVlabs/cuRobo
- commit: `ebb71702f3f70e767f40fd8e050674af0288abe8`
- [license at that revision][legacy-license]
- license characteristic: non-commercial research/evaluation use limitation

That revision is not MIT, BSD or Apache-2.0. Its research/evaluation restriction
must not be generalized to newer releases.

## Installed 0.8.0 distribution

The inspected `nvidia-curobo==0.8.0` distribution declares `Apache-2.0` in its
package metadata and includes `licenses/LICENSE` and `licenses/LICENSE_ASSETS`
in its `.dist-info` directory. This matches the official
[`v0.8.0` license][v2-license]. External robot resources retain the separate
terms recorded in [`LICENSE_ASSETS`][v2-assets]. No cuRobo resources are copied
into this package.

The 0.8.0 release also rewrites the public API. This license inspection is not
a GPU/runtime validation of the adapter against 0.8.0. Use the exact runtime
version appropriate for the adapter and its validated configuration; do not
substitute a newer version solely because its license differs.

The canonical MoveIt/OMPL path does not depend on cuRobo. This package neither
declares cuRobo/PyTorch as a generic ROS dependency nor downloads them during
build or tests. Enabling the plugin is an explicit choice; review the selected
runtime and asset licenses for that version.

[legacy-license]: https://github.com/NVlabs/curobo/blob/ebb71702f3f70e767f40fd8e050674af0288abe8/LICENSE
[v2-license]: https://github.com/NVlabs/curobo/blob/v0.8.0/LICENSE
[v2-assets]: https://github.com/NVlabs/curobo/blob/v0.8.0/LICENSE_ASSETS
