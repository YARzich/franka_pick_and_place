<!-- Responsibility: Record the provenance and license of adapted Panda MoveIt resources. -->

# Third-party notices

The semantic model and the kinematics, joint-limit, controller, and OMPL
defaults in this directory are adapted from
[`moveit_resources_panda_moveit_config`][upstream-panda].
The inspected release is **3.1.0**, commit
`9e3c48401260a3d225903b901ecb596ce609219f`. Its
[package manifest][upstream-manifest] declares `BSD` and lists
**Mike Lautman <mike@picknik.ai>** as author and maintainer. The local package
metadata preserves that upstream `BSD` spelling.

MoveIt's [official source documentation][moveit-license] identifies the
project's license as BSD-3-Clause and explicitly includes `moveit_resources`
in its list of source repositories. The standard three conditions and
disclaimer, as linked by that page to the [OSI license text][bsd-terms], are
included in [`../../LICENSES/BSD-3-Clause.txt`](../../LICENSES/BSD-3-Clause.txt),
together with the package attribution. This is a documented assembly of the
publisher's license declaration, attribution and standard terms, not a claim
to have copied a nonexistent package-level license file verbatim.

## Source history and attribution

- The [initial Panda import][initial-import], commit
  `5c71455fb7c0164597eeb8301b04f223c12add00`, was authored by **Mohmmad Ayman**.
  It already supplied the `BSD` declaration and credited Mike Lautman in the
  package manifest.
- [Alignment with the official Panda configuration][panda-alignment], commit
  `ad32e7ebcd7895f029bd6d6d9b75f082954fd1b5`, was authored by **Robert Haschke**.
- Later contributors and changes are recorded in the pinned
  [upstream changelog][upstream-changelog] and source history.

These credits preserve the observed authorship and source lineage. They do not
assign copyright ownership or invent copyright dates. The 3.1.0 configuration
package has no separate LICENSE file or per-file copyright notices to copy;
that is a limit of its notice detail, not an absence of the publisher's BSD
license declaration. The Apache license of the separate `panda_description`
package is not used to relicense these configuration files.

## Local modifications

The upstream model was adjusted for this project as follows:

- the floating `world` virtual joint was removed because supported backends
  fix `panda_link0` explicitly;
- planner selection was reduced to the arm-only RRTConnect contract;
- the URDF wrapper imports the canonical model from `franka_motion`;
- simulator-specific execution margins live in a node-scoped override;
- the table-camera occupancy profile selects the reusable
  `manipulation_moveit` updater plugin.

Redistribution retains these attribution details, the license conditions and
disclaimer, and the notices identifying the local modifications. No
upstream-authored copyright notice has been removed or replaced.

[upstream-panda]: https://github.com/moveit/moveit_resources/tree/9e3c48401260a3d225903b901ecb596ce609219f/panda_moveit_config
[upstream-manifest]: https://github.com/moveit/moveit_resources/blob/9e3c48401260a3d225903b901ecb596ce609219f/panda_moveit_config/package.xml
[moveit-license]: https://moveit.ai/documentation/source-code-api/
[bsd-terms]: https://opensource.org/license/BSD-3-Clause
[initial-import]: https://github.com/moveit/moveit_resources/commit/5c71455fb7c0164597eeb8301b04f223c12add00
[panda-alignment]: https://github.com/moveit/moveit_resources/commit/ad32e7ebcd7895f029bd6d6d9b75f082954fd1b5
[upstream-changelog]: https://github.com/moveit/moveit_resources/blob/9e3c48401260a3d225903b901ecb596ce609219f/panda_moveit_config/CHANGELOG.rst
