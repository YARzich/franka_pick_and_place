"""
Test Isaac resource ownership and failure-preserving cleanup.

Responsibility:
- Verify partial startup cleanup, best-effort shutdown ordering, and errors.
"""

import threading

import pytest

from franka_motion_runtime.simulation.isaac.runner import (
    IsaacStartupCancelled,
    _raise_if_stopping,
)
from franka_motion_runtime.simulation.isaac.runtime.lifecycle import (
    IsaacRuntimeOwner,
    run_with_runtime_owner,
)
from franka_motion_runtime.simulation.isaac.runtime.scene import (
    _configure_franka_urdf_import,
    initialize_robot,
)


def test_startup_checkpoint_turns_signal_into_owned_cancellation() -> None:
    """SIGINT/TERM state must leave startup through the owner cleanup path."""
    stop_event = threading.Event()
    stop_event.set()

    with pytest.raises(IsaacStartupCancelled):
        _raise_if_stopping(stop_event)


def test_robot_initialization_observes_startup_cancellation() -> None:
    """Articulation warmup must not continue stepping after a stop request."""
    world = _InitializationWorld()
    robot = _InitializationRobot()

    initialized = initialize_robot(
        world,
        robot,
        should_stop=lambda: True,
    )

    assert initialized is False
    assert world.reset_calls == 1
    assert world.step_calls == 0
    assert robot.initialize_calls == 0


def test_headless_urdf_import_uses_stable_franka_position_drives() -> None:
    """Do not inherit weak legacy URDF damping in the Isaac articulation."""
    import_config = _ImportConfig()

    _configure_franka_urdf_import(import_config, fix_base=False)

    assert import_config.fix_base is False
    assert import_config.merge_fixed_joints is True
    assert import_config.import_inertia_tensor is True
    assert import_config.default_drive_type == 1
    assert import_config.default_drive_strength == 1_000.0
    assert import_config.default_position_drive_damping == 100.0


def test_robot_initialization_applies_and_verifies_franka_drive_gains() -> None:
    """Loaded USD and imported URDF assets must receive Franka-like gains."""
    world = _InitializationWorld()
    robot = _InitializedRobot()

    initialized = initialize_robot(world, robot)

    assert initialized is True
    assert robot.controller.set_kps == [
        60_000.0,
        60_000.0,
        60_000.0,
        60_000.0,
        25_000.0,
        15_000.0,
        5_000.0,
        100.0,
        100.0,
    ]
    assert robot.controller.set_kds == [3_000.0] * 7 + [20.0, 20.0]
    assert robot.position_iterations == 64
    assert robot.velocity_iterations == 64


def test_robot_initialization_maps_drive_gains_by_joint_name() -> None:
    """Per-joint gains must not depend on Isaac's reported DOF order."""
    world = _InitializationWorld()
    robot = _InitializedRobot()
    robot.dof_names.reverse()

    initialized = initialize_robot(world, robot)

    assert initialized is True
    assert robot.controller.set_kps == [
        100.0,
        100.0,
        5_000.0,
        15_000.0,
        25_000.0,
        60_000.0,
        60_000.0,
        60_000.0,
        60_000.0,
    ]
    assert robot.controller.set_kds == [20.0, 20.0] + [3_000.0] * 7


def test_robot_initialization_rejects_unexpected_articulation_dofs() -> None:
    """Do not apply a Franka gain vector to an unrelated articulation."""
    world = _InitializationWorld()
    robot = _InitializedRobot()
    robot.dof_names[-1] = 'unexpected_joint'

    with pytest.raises(RuntimeError, match='неожиданный набор DOF'):
        initialize_robot(world, robot)


def test_runtime_owner_closes_app_when_world_was_not_created() -> None:
    """A setup error after SimulationApp creation must still close the app."""
    app = _CleanupRecorder("app")

    def fail_after_app_creation(owner: IsaacRuntimeOwner) -> None:
        owner.simulation_app = app
        raise ValueError("primary setup failure")

    with pytest.raises(ValueError, match="primary setup failure"):
        run_with_runtime_owner(fail_after_app_creation)

    assert app.calls == ["app.close"]


def test_cleanup_attempts_every_step_and_preserves_primary_error() -> None:
    """Cleanup failures must not mask the primary runtime failure."""
    calls: list[str] = []
    owner = IsaacRuntimeOwner(
        _CleanupRecorder("app", calls),
        _FailingWorld(calls),
        _FailingWorldClass(calls),
    )

    with pytest.raises(ValueError, match="primary") as error_info:
        try:
            raise ValueError("primary")
        finally:
            owner.close()

    assert calls == ["world.stop", "world.clear", "app.close"]
    assert any("cleanup also failed" in note for note in error_info.value.__notes__)


def test_cleanup_reports_failure_without_primary_error() -> None:
    """A standalone cleanup error must remain observable after app.close()."""
    calls: list[str] = []
    owner = IsaacRuntimeOwner(
        _CleanupRecorder("app", calls),
        _FailingWorld(calls),
        _FailingWorldClass(calls),
    )

    with pytest.raises(RuntimeError, match="cleanup failed"):
        owner.close()

    assert calls == ["world.stop", "world.clear", "app.close"]


class _CleanupRecorder:
    """Record SimulationApp cleanup without importing Isaac runtime types."""

    def __init__(self, name: str, calls: list[str] | None = None) -> None:
        self._name = name
        self.calls = [] if calls is None else calls

    def close(self) -> None:
        """Record the expected cleanup call."""
        call = f"{self._name}.close" if self._name != "app" else "app.close"
        self.calls.append(call)


class _FailingWorld:
    """Fake World whose stop method fails after recording the attempt."""

    def __init__(self, calls: list[str]) -> None:
        self._calls = calls

    def stop(self) -> None:
        """Simulate a World shutdown failure."""
        self._calls.append("world.stop")
        raise RuntimeError("stop failed")


class _FailingWorldClass:
    """Fake World class whose singleton cleanup also fails."""

    def __init__(self, calls: list[str]) -> None:
        self._calls = calls

    def clear_instance(self) -> None:
        """Simulate failure while clearing the World singleton."""
        self._calls.append("world.clear")
        raise RuntimeError("clear failed")


class _InitializationWorld:
    """Record world calls made during cancellable articulation warmup."""

    def __init__(self) -> None:
        self.reset_calls = 0
        self.step_calls = 0

    def reset(self) -> None:
        """Record the mandatory reset before initialization attempts."""
        self.reset_calls += 1

    def step(self, *, render: bool) -> None:
        """Record a warmup step while preserving the production signature."""
        del render
        self.step_calls += 1


class _InitializationRobot:
    """Record robot initialization attempts without Isaac imports."""

    handles_initialized = False
    dof_names: list[str] = []

    def __init__(self) -> None:
        self.initialize_calls = 0

    def initialize(self) -> None:
        """Record one articulation initialization attempt."""
        self.initialize_calls += 1


class _ImportConfig:
    """Mutable stand-in for Isaac's URDF import configuration."""

    fix_base: bool | None = None
    merge_fixed_joints: bool | None = None
    import_inertia_tensor: bool | None = None
    default_drive_type: int | None = None
    default_drive_strength: float | None = None
    default_position_drive_damping: float | None = None


class _InitializedRobot:
    """Initialized canonical Franka articulation with a recording controller."""

    handles_initialized = True

    def __init__(self) -> None:
        self.dof_names = [
            *[f'panda_joint{index}' for index in range(1, 8)],
            'panda_finger_joint1',
            'panda_finger_joint2',
        ]
        self.controller = _GainController()
        self.position_iterations = 0
        self.velocity_iterations = 0

    def initialize(self) -> None:
        """Match the production initialization call."""

    def get_articulation_controller(self):
        """Return the fake controller exposed by Isaac's Robot API."""
        return self.controller

    def set_solver_position_iteration_count(self, count: int) -> None:
        """Record the position-solver iteration budget."""
        self.position_iterations = count

    def get_solver_position_iteration_count(self) -> int:
        """Read back the position-solver iteration budget."""
        return self.position_iterations

    def set_solver_velocity_iteration_count(self, count: int) -> None:
        """Record the velocity-solver iteration budget."""
        self.velocity_iterations = count

    def get_solver_velocity_iteration_count(self) -> int:
        """Read back the velocity-solver iteration budget."""
        return self.velocity_iterations


class _GainController:
    """Record and read back drive gains without importing Isaac."""

    def __init__(self) -> None:
        self.set_kps = [100.0] * 7 + [250.0, 250.0]
        self.set_kds = [10.0] * 7 + [25.0, 25.0]

    def set_gains(self, *, kps, kds) -> None:
        """Record the gain vectors."""
        self.set_kps = kps.tolist()
        self.set_kds = kds.tolist()

    def get_gains(self):
        """Return the applied vectors using Isaac's tuple contract."""
        import numpy as np

        return np.asarray(self.set_kps), np.asarray(self.set_kds)
