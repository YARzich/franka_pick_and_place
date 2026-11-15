<!-- Responsibility: Define repository-wide compatibility, versioning, and review rules. -->

# Change Control

This repository is a pre-1.0 reference codebase. Its manipulation candidate
package family contains
`manipulation_msgs`, `manipulation`, `manipulation_moveit`,
`manipulation_perception`, `manipulation_perception_isaac_ros`,
`manipulation_curobo`, and `franka_motion`. Deprecated compatibility packages
are migration aids, not API owners for new code. Package locations and status
are tracked in the family [README](../README.md) and [ROADMAP](../ROADMAP.md).

## Public Surface

A surface is public when package documentation presents it for downstream use.
This includes:

- ROS message, service, and action definitions, including numeric constants;
- documented topic, service, action, parameter, frame, unit, QoS, and clock semantics;
- installed executable and launch names, launch arguments, and documented resource URIs;
- documented plugin IDs, exported CMake targets, and installed C++ headers;
- explicitly documented Python import APIs. Other Python modules are implementation details.

Merely placing a file under `include/` or exporting a target can expose it to a
consumer. Such surfaces must be documented or made private before a stability
claim is made.

## Compatibility Rules

Existing `.msg`, `.srv`, and `.action` definitions are stable wire contracts.
Changing a field, field order, type, constant value, package-qualified type, or
meaning requires a new versioned interface or an explicit breaking migration;
an unchanged endpoint name must never conceal a changed wire type.

`manipulation_msgs/docs/public_wire_contract.v2.json` (relative to the family root)
records the normalized wire/API shape checked by the local contract tests.
The V1 snapshot is retained as migration history. Updating the current
snapshot is review evidence, not permission to mutate an existing interface.
Compatible growth adds a new interface or appends a previously unused constant
while preserving all prior entries and values. An incompatible shape requires
a new versioned type plus an explicit endpoint migration or bridge.

ROS graph compatibility also includes frame and unit meaning, QoS
compatibility, timestamp and clock rules, error semantics, and safe defaults.
New optional endpoints or parameters must be disabled or behavior-preserving by
default. Machine clients branch on typed values, not operator-facing text.

The current C++ promise is source compatibility only for documented installed
headers and exported targets. Binary ABI is not guaranteed. Downstream C++
consumers must be rebuilt with the package set, and no `SOVERSION` or stable
symbol-layout claim is implied. A binary ABI promise requires a separate policy,
visibility controls, versioned shared libraries, and automated ABI comparison.

## Versioning

Package versions follow Semantic Versioning independently:

- patch: compatible fixes, documentation, and internal refactoring;
- minor while below `1.0.0`: additive public API, or a deliberately reviewed
  breaking migration with release notes and a consumer migration path;
- major after `1.0.0`: an incompatible public-surface change.

Stable ROS wire definitions should be replaced by a new versioned type rather
than edited in place, even before `1.0.0`. Promotion to `1.0.0` requires a
complete public-surface inventory, per-package changelogs, and the evidence
claimed by the quality documentation.

## Required Review

Run the applicable local build and test checks from [INSTALL.md](../INSTALL.md)
and the contribution procedure in [CONTRIBUTING.md](../CONTRIBUTING.md).

Every public-surface change must:

1. identify the owning package and compatibility class;
2. update its README/API and applicable migration documentation;
3. update or add contract tests before implementation consumers are changed;
4. update the package version and changelog when release semantics change;
5. preserve typed failures, cancellation, ownership, frame, clock, and cleanup invariants;
6. record only evidence actually produced by the tested environment.

Deprecation must name the canonical replacement and a removal gate. Removal is
allowed only after repository consumers are migrated and every migration check
listed by the migration documentation has produced evidence. Security fixes may
shorten a deprecation period, but must still document the compatibility impact.
