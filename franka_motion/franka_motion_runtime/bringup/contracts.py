"""
Typed backend and scenario selection for Franka bringup.

Responsibility:
- Parse launch strings into closed enums before selecting a leaf launch.
- Reject unsupported planner, simulator and scenario combinations.

Does not own:
- Resolving package paths or constructing launch actions.
- Simulator, motion, controller or perception implementation details.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TypeVar

_EnumT = TypeVar("_EnumT", bound=Enum)


class InvalidBringupSelection(ValueError):
    """Report a malformed or unsupported bringup selection."""


class SimulatorBackend(str, Enum):
    """Supported simulator process backends."""

    GAZEBO = "gazebo"
    ISAAC = "isaac"

    @classmethod
    def parse(cls, value: str) -> "SimulatorBackend":
        """Parse an exact simulator name with a useful allowed-values error."""
        return _parse_enum(cls, value, "simulator")


class ScenarioProfile(str, Enum):
    """Supported simulation scene profiles."""

    EMPTY = "empty"
    TABLE = "table"
    TABLE_CAMERA = "table_camera"

    @classmethod
    def parse(cls, value: str) -> "ScenarioProfile":
        """Parse an exact scenario name with a useful allowed-values error."""
        return _parse_enum(cls, value, "scenario")


class PlannerBackend(str, Enum):
    """Motion backends exposed by the canonical bringup router."""

    MOVEIT = "moveit"
    CUROBO = "curobo"

    @classmethod
    def parse(cls, value: str) -> "PlannerBackend":
        """Parse an exact planner name with a useful allowed-values error."""
        return _parse_enum(cls, value, "planner_backend")


@dataclass(frozen=True)
class BringupSelection:
    """Validated typed key used to choose one stable leaf launch."""

    simulator: SimulatorBackend
    scenario: ScenarioProfile
    planner_backend: PlannerBackend

    def __post_init__(self) -> None:
        """Reject combinations for which no maintained leaf launch exists."""
        if (
            self.planner_backend is PlannerBackend.CUROBO
            and self.scenario is ScenarioProfile.TABLE_CAMERA
        ):
            raise InvalidBringupSelection(
                "planner_backend='curobo' cannot represent the table-camera "
                "Octomap; use scenario='empty' or scenario='table'"
            )

    @classmethod
    def from_strings(
        cls,
        *,
        simulator: str,
        scenario: str,
        planner_backend: str,
    ) -> "BringupSelection":
        """Parse and validate raw launch configuration strings."""
        return cls(
            simulator=SimulatorBackend.parse(simulator),
            scenario=ScenarioProfile.parse(scenario),
            planner_backend=PlannerBackend.parse(planner_backend),
        )


def _parse_enum(
    enum_type: type[_EnumT],
    value: str,
    argument_name: str,
) -> _EnumT:
    """Parse one closed enum while preserving its concrete return type."""
    try:
        return enum_type(value)
    except ValueError as error:
        allowed_values = ", ".join(member.value for member in enum_type)
        raise InvalidBringupSelection(
            f"{argument_name}={value!r} is invalid; expected one of: "
            f"{allowed_values}"
        ) from error
