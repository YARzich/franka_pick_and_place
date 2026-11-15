"""
Regression checks for the V2-only surface after an incremental install.

Responsibility: Prove exact retirement of V1 artifacts without deleting the
current V2 install surface.
"""

from __future__ import annotations

import os
from pathlib import Path


def _install_prefix() -> Path:
    value = os.environ.get('MANIPULATION_TEST_INSTALL_PREFIX')
    assert value, 'MANIPULATION_TEST_INSTALL_PREFIX is required'
    prefix = Path(value)
    assert prefix.is_dir(), f'manipulation install prefix is missing: {prefix}'
    return prefix


def test_incremental_install_removes_exact_v1_surface() -> None:
    """A prior V1 install must not leak headers, binaries, or its client archive."""
    prefix = _install_prefix()
    retired = (
        'include/manipulation/client',
        'include/manipulation/workflow/detail/transfer_object_session.hpp',
        'include/manipulation/workflow/perception_resolved_transfer_sequence_action_server.hpp',
        'include/manipulation/workflow/perception_resolved_transfer_sequence_goal_validation.hpp',
        'include/manipulation/workflow/perception_resolved_transfer_target_policy.hpp',
        'include/manipulation/workflow/transfer_object_action_server.hpp',
        'include/manipulation/workflow/transfer_object_goal_validation.hpp',
        'include/manipulation/workflow/transfer_object_result_policy.hpp',
        'include/manipulation/workflow/transfer_object_sequence_action_context.hpp',
        'include/manipulation/workflow/transfer_object_sequence_action_server.hpp',
        'include/manipulation/workflow/transfer_object_sequence_goal_validation.hpp',
        'include/manipulation/workflow/transfer_object_sequence_runner.hpp',
        'include/manipulation/workflow/transfer_object_task_context.hpp',
        'lib/manipulation/transfer_object_client',
        'lib/manipulation/transfer_object_sequence_server',
        'lib/libmanipulation_transfer_object_client.a',
    )
    remaining = [
        relative
        for relative in retired
        if os.path.lexists(prefix / relative)
    ]
    assert not remaining


def test_incremental_install_preserves_current_v2_surface() -> None:
    """The cleanup allowlist must not match current V2 contracts or policies."""
    prefix = _install_prefix()
    current = (
        'include/manipulation/workflow/transfer_execution_context.hpp',
        'include/manipulation/workflow/transfer_object_sequence_v2_action_server.hpp',
        'include/manipulation/workflow/transfer_object_sequence_v2_goal_validation.hpp',
        'include/manipulation/workflow/transfer_sequence_task_context.hpp',
        'lib/libmanipulation_transfer_domain.a',
        'lib/libmanipulation_workflow.a',
    )
    missing = [relative for relative in current if not (prefix / relative).is_file()]
    assert not missing
