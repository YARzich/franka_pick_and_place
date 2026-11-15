<!-- Responsibility: State the repository trust model, reporting route, and safety boundaries. -->

# Security Policy

## Scope And Reporting

This policy applies to the current default-branch versions of the
candidate package family listed in `docs/CHANGE_CONTROL.md`. Deprecated compatibility
packages receive only fixes needed for their documented migration gates.

Send vulnerability reports to [yaric002@gmail.com](mailto:yaric002@gmail.com),
the maintainer address confirmed for this project. If the repository host has
private vulnerability reporting enabled, that route can also be used. Include
the affected revision, impact, and a minimal reproduction without credentials
or live robot addresses. Do not post an unpatched exploit or unsafe motion
details in a public issue. No response deadline is promised.

## Trust Model

The default ROS 2 graph is a trusted deployment network, not an authentication
boundary. PlanningScene leases coordinate ownership; they do not authorize
callers. Diagnostic allowlists prevent accidental coupling but do not establish
publisher identity. Deployments with untrusted participants must use SROS 2 or
DDS security, network isolation, least-privilege host accounts, and explicit
access control outside these packages.

Launch files, parameter files, robot descriptions, package resources, external
executable paths, model files, and ML checkpoints are trusted operator inputs.
Python and ML checkpoint formats may execute code while loading; do not load
untrusted or unpinned artifacts. External subprocesses must receive argv arrays,
run without a shell, support bounded cancellation and cleanup, and remain owned
until reaped. Operation-specific deadlines do not impose a whole-task deadline
on a long-running worker or accepted transfer goal.

## Secrets And Evidence

Robot IPs, credentials, tokens, and private keys must not be committed or
written to logs or artifacts. Real-hardware site configuration belongs in the
protected deployment environment. Runtime evidence may contain revision, platform,
ROS/RMW, simulator, stage, timing, and cleanup identity, but not argv or secret
environment values.

Third-party runtimes, models, and assets keep their own security and license
terms. Use pinned revisions where the repository supplies source manifests,
review each owning package's `THIRD_PARTY_NOTICES.md`, and keep optional
GPU/reference backends out of the default production graph.

## Robot Safety Boundary

The local checks described in [INSTALL.md](INSTALL.md) provide development
evidence, not a functional-safety layer. A
real-hardware deployment must independently enforce workspace and velocity
limits, guarded-cell access, emergency stop, watchdog/dead-man behavior,
bounded motion, and fail-safe stop if the controlling process, network, or
task execution fails. Hardware verification requires a deployment-specific
procedure that owns credentials, access control and physical recovery policy.

After a report, fixes must preserve typed failure disposition, fail-closed
admission, exact resource ownership, bounded cancellation, and cleanup evidence.
Public release notes should describe the affected versions and mitigation
without exposing secrets or unsafe operational detail.
