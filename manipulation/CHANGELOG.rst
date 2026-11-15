^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Ignore malformed JointState and simulation-clock stamps before refreshing
  health observations; valid samples recover without restarting the monitor.
* Preserve current-step progress and checkpoint/recovery counters during final
  scene readback. Isolate target geometry in a private policy with direct tests.
* Acknowledge safe V2 pause in feedback while renewing the scene lease, and
  clear the acknowledgement on resume or cancel.
* Preserve completed steps, the failed index, and confirmed physical evidence
  when a provider or strategy throws. Retain scene ownership and request stop
  with command-resource quarantine when strategy completion is unproven.

0.2.0 (2026-07-23)
------------------
* Removed the legacy ``TransferObject``, static-box sequence, and
  perception-resolved action implementations, clients, executables, and CMake
  export.
* Kept one ``TransferObjectSequenceV2`` facade over the backend-neutral resolved
  workflow and renamed the shared lifecycle session to remove V1 compatibility
  aliases from installed headers.
* Linearized downstream dispatch with pause/cancel, added stop-only heartbeat
  and lease-maintenance gates, and kept lease renewal active during safe pause
  and accepted physical-command waits.
* Made V2 goal-scoped scene lease release conditional on terminal object and
  physical-command evidence; ambiguous scene mutations, unsafe dispositions,
  and command quarantine now retain ownership, including through outer
  cancel/deadline/shutdown normalization.
* Added typed wait completion outcomes with preserved diagnostics and exact
  cleanup of retired V1 artifacts from incremental installs.
* Added catalog-declared ``ObjectProfile`` local proper-rotation symmetries and
  limited nearest-equivalent orientation selection to
  ``TARGET_INITIAL_SOURCE``; raw observations and absolute targets remain
  unchanged.

0.1.0 (2026-07-14)
------------------
* Established reusable task lifecycle, PlanningScene coordination, ordered
  workflow, bounded client transport, and health-monitoring components.
