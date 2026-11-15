"""
Validation of completed command exit and output contracts.

Responsibility:
- Compare one completed result with its configured exit and regex contract.

Does not own:
- Process creation, log capture, retry policy, deadlines or evidence claims.
"""

from __future__ import annotations

import re

from franka_motion_runtime.validation.runtime_deadline import (
    CommandResult,
    ProcessError,
)
from franka_motion_runtime.validation.runtime_types import CommandSpec


def validate_command_result(
    command: CommandSpec,
    result: CommandResult,
) -> None:
    """Reject unexpected exits or missing configured output expressions."""
    if result.return_code not in command.expected_return_codes:
        raise ProcessError(
            f"{command.name} exited with {result.return_code}; "
            f"expected {command.expected_return_codes}"
        )

    output = result.log_path.read_text(encoding="utf-8", errors="replace")
    missing_expressions = [
        expression
        for expression in command.stdout_regex
        if re.search(expression, output) is None
    ]
    if missing_expressions:
        raise ProcessError(
            f"{command.name} output did not match: "
            + ", ".join(repr(value) for value in missing_expressions)
        )
