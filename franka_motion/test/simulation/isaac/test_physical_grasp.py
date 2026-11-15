"""
Fast contracts for the Isaac physical cube attachment barrier.

Responsibility:
- Prove ROS-side waits do not mutate USD from the callback thread.
- Prove replies are released only after the matching physics step.
- Verify shutdown completion and collision-filter ownership helpers.
"""

from types import SimpleNamespace
from types import ModuleType
import math
import sys
import threading
import time

import pytest

from franka_motion_runtime.simulation.isaac.runtime.loop import run_simulation_loop
from franka_motion_runtime.simulation.isaac.runtime.physical_grasp import (
    IsaacPickCubeAttachmentService,
    PhysicalAttachmentBarrier,
    SetBoolAttachmentCallback,
    UsdPhysicalAttachmentBackend,
    _MetricRelativePose,
    _format_cube_world_position_m,
    _make_metric_relative_pose,
    _merge_required_targets,
    _relative_pose_errors,
    _remove_owned_targets,
    _require_capture_distance,
    _require_relative_pose_close,
    _require_usd_write_succeeded,
)


def test_request_applies_on_simulation_thread_and_replies_after_step() -> None:
    """The service thread only queues; before/after hooks own apply and ACK."""
    backend = _Backend()
    barrier = PhysicalAttachmentBarrier(backend)
    thread, results = _start_request(barrier, True)

    barrier.before_step()
    assert backend.calls == [("apply", True)]
    assert backend.thread_ids == [threading.get_ident()]
    assert thread.is_alive()

    barrier.after_step()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert results[0].success is True
    assert backend.calls == [
        ("apply", True),
        ("verify", True),
        ("confirm", True),
    ]
    assert set(backend.thread_ids) == {threading.get_ident()}


def test_apply_failure_is_reported_only_after_following_step() -> None:
    """A distant or incompatible object is a stepped negative response."""
    backend = _Backend(apply_error=RuntimeError("cube is distant"))
    barrier = PhysicalAttachmentBarrier(backend)
    thread, results = _start_request(barrier, True)

    barrier.before_step()
    assert thread.is_alive()
    assert results == []

    barrier.after_step()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert results[0].success is False
    assert results[0].message == "cube is distant"
    assert backend.calls == [("apply", True)]


def test_verification_failure_is_returned_after_physics_step() -> None:
    """USD authoring alone is not accepted when PhysX state is not confirmed."""
    backend = _Backend(verify_error=RuntimeError("joint inactive"))
    barrier = PhysicalAttachmentBarrier(backend)
    thread, results = _start_request(barrier, True)

    barrier.before_step()
    barrier.after_step()
    thread.join(timeout=1.0)

    assert results[0].success is False
    assert results[0].message == "joint inactive"
    assert backend.calls == [("apply", True), ("verify", True)]


def test_success_response_includes_stepped_ground_truth_detail() -> None:
    """The unchanged SetBool response carries stable cube-world evidence."""
    detail = "cube_world_m=(0.400000,0.000000,0.030000)"
    backend = _Backend(confirm_detail=detail)
    barrier = PhysicalAttachmentBarrier(backend)
    thread, results = _start_request(barrier, True)

    barrier.before_step()
    barrier.after_step()
    thread.join(timeout=1.0)

    assert results[0].message == (
        "pick cube physical state confirmed attached; " + detail
    )


def test_cube_world_position_detail_has_stable_metric_format() -> None:
    """Formatting normalizes negative zero and keeps fixed coordinate precision."""
    assert _format_cube_world_position_m((0.4, -1e-9, 0.03)) == (
        "cube_world_m=(0.400000,0.000000,0.030000)"
    )


def test_relative_pose_error_is_metric_and_quaternion_sign_invariant() -> None:
    """Equivalent quaternion signs must not create a false rotation error."""
    expected = _MetricRelativePose((0.02, -0.01, 0.03), (1.0, 0.0, 0.0, 0.0))
    actual = _MetricRelativePose((0.021, -0.01, 0.03), (-2.0, 0.0, 0.0, 0.0))

    translation_error_m, rotation_error_rad = _relative_pose_errors(
        expected,
        actual,
    )

    assert translation_error_m == pytest.approx(0.001)
    assert rotation_error_rad == pytest.approx(0.0)
    _require_relative_pose_close(
        expected,
        _MetricRelativePose(
            (0.0205, -0.01, 0.03),
            (-1.0, 0.0, 0.0, 0.0),
        ),
    )


@pytest.mark.parametrize(
    ("actual", "diagnostic"),
    [
        (
            _MetricRelativePose((0.003, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
            "translation_error",
        ),
        (
            _MetricRelativePose(
                (0.0, 0.0, 0.0),
                (
                    math.cos(math.radians(1.5)),
                    0.0,
                    0.0,
                    math.sin(math.radians(1.5)),
                ),
            ),
            "rotation_error",
        ),
    ],
)
def test_relative_pose_verification_rejects_post_step_drift(
    actual: _MetricRelativePose,
    diagnostic: str,
) -> None:
    """Translation above 1 mm or rotation above 1 degree fails closed."""
    expected = _MetricRelativePose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))

    with pytest.raises(RuntimeError, match=diagnostic):
        _require_relative_pose_close(expected, actual)


def test_metric_relative_pose_converts_stage_units_and_normalizes_rotation() -> None:
    """The safety comparison is independent of authored stage units."""
    pose = _make_metric_relative_pose(
        (40.0, 0.0, 3.0),
        (2.0, 0.0, 0.0, 0.0),
        meters_per_unit=0.01,
    )

    assert pose.translation_m == pytest.approx((0.4, 0.0, 0.03))
    assert pose.rotation_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    "rotation",
    [
        (0.0, 0.0, 0.0, 0.0),
        (float("nan"), 0.0, 0.0, 0.0),
        (float("inf"), 0.0, 0.0, 0.0),
    ],
)
def test_metric_relative_pose_rejects_invalid_rotation(
    rotation: tuple[float, float, float, float],
) -> None:
    """Invalid transforms cannot be accepted as physical evidence."""
    with pytest.raises(RuntimeError, match="quaternion"):
        _make_metric_relative_pose(
            (0.0, 0.0, 0.0),
            rotation,
            meters_per_unit=1.0,
        )


def test_relative_pose_verification_rejects_nonfinite_translation() -> None:
    """Nonfinite post-step translation cannot pass the physical ACK."""
    expected = _MetricRelativePose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    actual = _MetricRelativePose(
        (float("nan"), 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    )

    with pytest.raises(RuntimeError, match="translation"):
        _require_relative_pose_close(expected, actual)


def test_backend_uses_fresh_post_step_relative_pose() -> None:
    """USD backend verification compares a fresh transform to the attach capture."""
    expected = _MetricRelativePose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    backend = object.__new__(UsdPhysicalAttachmentBackend)
    backend._captured_relative_pose = expected
    backend._touch_body_paths = ("/hand", "/left", "/right")
    backend._require_owned_joint = lambda: object()
    backend._filtered_targets = lambda: backend._touch_body_paths
    backend._current_relative_pose = lambda: _MetricRelativePose(
        (0.004, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    )

    with pytest.raises(RuntimeError, match="translation_error"):
        backend.verify_attached(True)


@pytest.mark.parametrize("result", [False, None, 1, object()])
def test_usd_write_requires_an_explicit_true(result: object) -> None:
    """Truthy or missing USD results are not authoring evidence."""
    with pytest.raises(RuntimeError, match="critical operation"):
        _require_usd_write_succeeded(result, "critical operation")

    _require_usd_write_succeeded(True, "critical operation")


def test_shutdown_completes_pending_request_without_scene_mutation() -> None:
    """Controlled close must unblock callbacks before the Isaac World stops."""
    backend = _Backend()
    barrier = PhysicalAttachmentBarrier(backend)
    thread, results = _start_request(barrier, False)

    barrier.close()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert results[0].success is False
    assert "shutting down" in results[0].message
    assert backend.calls == []


def test_shutdown_completes_request_applied_before_failed_world_step() -> None:
    """If world.step fails, the service returns failure instead of a false ACK."""
    backend = _Backend()
    barrier = PhysicalAttachmentBarrier(backend)
    thread, results = _start_request(barrier, True)

    barrier.before_step()
    barrier.close()
    thread.join(timeout=1.0)

    assert results[0].success is False
    assert backend.calls == [("apply", True)]


def test_service_close_detaches_owned_state_even_when_ros_close_fails() -> None:
    """USD cleanup is attempted after pending callbacks stop, despite ROS errors."""
    backend = _Backend()
    ros_owner = _ServiceOwner(close_error=RuntimeError("executor failed"))
    attachment = IsaacPickCubeAttachmentService(
        backend,
        service_factory=lambda *_args, **_kwargs: ros_owner,
    )

    with pytest.raises(RuntimeError, match="полностью закрыть"):
        attachment.close()

    assert ros_owner.closed is True
    assert backend.calls == [
        ("apply", False),
        ("verify", False),
        ("confirm", False),
    ]


def test_preexisting_joint_at_owned_path_is_preserved(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A schema-compatible canonical-path joint is foreign until this backend creates it."""
    pxr_module = ModuleType("pxr")
    pxr_module.Sdf = SimpleNamespace()
    pxr_module.UsdPhysics = SimpleNamespace()
    monkeypatch.setitem(sys.modules, "pxr", pxr_module)

    stage = _ForeignJointStage()
    backend = object.__new__(UsdPhysicalAttachmentBackend)
    backend._stage = stage
    backend._joint_path = "/World/Franka/pick_cube_fixed_joint"
    backend._owns_joint = False
    backend._owned_filter_paths = set()

    with pytest.raises(RuntimeError, match="чужим prim"):
        backend.apply_attached(False)

    assert stage.remove_calls == []
    assert stage.GetPrimAtPath(backend._joint_path).IsValid()


def test_partial_collision_filter_failure_rolls_back_attach_and_detach(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Both transitions restore original targets when SetTargets mutates then fails."""
    pxr_module = _create_filter_only_pxr_module()
    monkeypatch.setitem(sys.modules, "pxr", pxr_module)
    stage = _FilterFailureStage()
    backend = _create_filter_failure_backend(stage)

    attach_relation = _PartialFailureRelation(("/World/Foreign",))
    pxr_module.UsdPhysics.FilteredPairsAPI.relation = attach_relation
    with pytest.raises(RuntimeError, match="SetTargets failed"):
        backend.apply_attached(True)

    assert attach_relation.targets == ["/World/Foreign"]
    assert attach_relation.set_calls == 2
    assert backend._owns_joint is False
    assert backend._owned_filter_paths == set()
    assert stage.remove_calls == []

    owned_filters = {
        "/World/Franka/panda_leftfinger",
        "/World/Franka/panda_rightfinger",
    }
    original_detach_targets = (
        "/World/Foreign",
        *sorted(owned_filters),
    )
    detach_relation = _PartialFailureRelation(original_detach_targets)
    pxr_module.UsdPhysics.FilteredPairsAPI.relation = detach_relation
    backend._owns_joint = True
    backend._owned_filter_paths = owned_filters
    backend._owned_joint_or_none = lambda: object()

    with pytest.raises(RuntimeError, match="SetTargets failed"):
        backend.apply_attached(False)

    assert detach_relation.targets == list(original_detach_targets)
    assert detach_relation.set_calls == 2
    assert backend._owns_joint is True
    assert backend._owned_filter_paths == owned_filters
    assert stage.remove_calls == []


@pytest.mark.parametrize(
    "failed_operation",
    [
        "filtered_pairs",
        "body0",
        "body1",
        "local_pos0",
        "local_rot0",
        "local_pos1",
        "local_rot1",
        "break_force",
        "break_torque",
        "collision_enabled",
        "exclude_from_articulation",
    ],
)
def test_false_critical_usd_write_rolls_back_attach(
    monkeypatch: pytest.MonkeyPatch,
    failed_operation: str,
) -> None:
    """Every critical relation/attribute false result restores owned USD state."""
    stage = _BooleanWriteStage(failed_operation=failed_operation)
    monkeypatch.setitem(sys.modules, "pxr", _create_boolean_write_pxr_module(stage))
    backend = _create_boolean_write_backend(stage)

    with pytest.raises(RuntimeError, match="USD write не подтверждён"):
        backend.apply_attached(True)

    assert stage.joint is None
    assert stage.filtered_relation.targets == ["/World/Foreign"]
    assert backend._owns_joint is False
    assert backend._owned_filter_paths == set()
    assert backend._captured_relative_pose is None
    assert failed_operation in stage.writer.calls


def test_failed_joint_rollback_retains_ownership_for_later_cleanup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An unremoved partial joint remains owned instead of becoming foreign."""
    stage = _BooleanWriteStage(
        failed_operation="local_pos0",
        remove_succeeds=False,
    )
    monkeypatch.setitem(sys.modules, "pxr", _create_boolean_write_pxr_module(stage))
    backend = _create_boolean_write_backend(stage)

    with pytest.raises(RuntimeError) as error:
        backend.apply_attached(True)

    assert any(
        "Attach joint rollback failed" in note
        for note in error.value.__notes__
    )
    assert stage.joint is not None
    assert backend._owns_joint is True
    assert backend._captured_relative_pose is not None

    stage.remove_succeeds = True
    backend.apply_attached(False)
    assert stage.joint is None


def test_set_bool_callback_maps_the_stepped_result() -> None:
    """The ROS adapter must expose the barrier result without extra policy."""
    backend = _Backend()
    barrier = PhysicalAttachmentBarrier(backend)
    logger = _EventLogger()
    barrier._set_event_logger(logger)
    callback = SetBoolAttachmentCallback(barrier)
    response = SimpleNamespace(success=None, message="")
    returned: list[object] = []

    thread = threading.Thread(
        target=lambda: returned.append(
            callback(SimpleNamespace(data=False), response)
        )
    )
    thread.start()
    _wait_for_pending_request(barrier)
    barrier.before_step()
    barrier.after_step()
    thread.join(timeout=1.0)

    assert returned == [response]
    assert response.success is True
    assert "detached" in response.message
    assert logger.messages == [
        "physical_attachment service request desired=detached",
        "physical_attachment simulation apply desired=detached",
        "physical_attachment simulation ACK desired=detached success=true",
    ]


def test_collision_filter_helpers_preserve_foreign_targets() -> None:
    """Detach removes only filters introduced by this service instance."""
    foreign = "/World/foreign_body"
    preexisting_touch = "/World/Franka/panda_hand"
    required = (
        preexisting_touch,
        "/World/Franka/panda_leftfinger",
        "/World/Franka/panda_rightfinger",
    )

    merged, owned = _merge_required_targets(
        (foreign, preexisting_touch),
        required,
    )
    restored = _remove_owned_targets(merged, owned)

    assert merged == (foreign, *required)
    assert owned == set(required[1:])
    assert restored == (foreign, preexisting_touch)


@pytest.mark.parametrize("distance", [float("nan"), float("inf")])
def test_capture_rejects_nonfinite_proximity(distance: float) -> None:
    """Invalid transforms must fail closed instead of bypassing the distance gate."""
    with pytest.raises(RuntimeError, match="конечным"):
        _require_capture_distance(distance, maximum_m=0.03)


def test_simulation_loop_wraps_each_world_step_with_barrier_hooks() -> None:
    """An ACK hook follows the exact physics step that consumed its mutation."""
    calls: list[str] = []
    world = _World(calls)
    app = _App(max_checks=3)

    run_simulation_loop(
        world,
        app,
        headless=True,
        time_steps=2,
        before_step=lambda: calls.append("attachment.before"),
        after_step=lambda: calls.append("attachment.after"),
    )

    assert calls == [
        "world.play",
        "attachment.before",
        "world.step",
        "attachment.after",
        "attachment.before",
        "world.step",
        "attachment.after",
    ]


def _start_request(
    barrier: PhysicalAttachmentBarrier,
    desired_attached: bool,
) -> tuple[threading.Thread, list[object]]:
    results: list[object] = []
    thread = threading.Thread(
        target=lambda: results.append(barrier.request(desired_attached))
    )
    thread.start()
    _wait_for_pending_request(barrier)
    return thread, results


def _wait_for_pending_request(barrier: PhysicalAttachmentBarrier) -> None:
    deadline = time.monotonic() + 1.0
    while barrier.pending_request_count == 0 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert barrier.pending_request_count == 1


class _Backend:
    """Record simulation-thread backend calls and inject typed failures."""

    def __init__(
        self,
        *,
        apply_error: Exception | None = None,
        verify_error: Exception | None = None,
        confirm_detail: str | None = None,
    ) -> None:
        self.apply_error = apply_error
        self.verify_error = verify_error
        self.confirm_detail = confirm_detail
        self.calls: list[tuple[str, bool]] = []
        self.thread_ids: list[int] = []

    def apply_attached(self, desired_attached: bool) -> None:
        self._record("apply", desired_attached)
        if self.apply_error is not None:
            raise self.apply_error

    def verify_attached(self, desired_attached: bool) -> None:
        self._record("verify", desired_attached)
        if self.verify_error is not None:
            raise self.verify_error

    def confirm_attached(self, desired_attached: bool) -> str | None:
        self._record("confirm", desired_attached)
        return self.confirm_detail

    def _record(self, operation: str, desired_attached: bool) -> None:
        self.calls.append((operation, desired_attached))
        self.thread_ids.append(threading.get_ident())


class _ServiceOwner:
    """Injectable ROS owner stand-in for close ordering and aggregation."""

    def __init__(self, *, close_error: Exception | None = None) -> None:
        self.close_error = close_error
        self.closed = False

    def raise_if_failed(self) -> None:
        """Match the production owner API."""

    def close(self) -> None:
        self.closed = True
        if self.close_error is not None:
            raise self.close_error


class _EventLogger:
    """Capture request-scoped handoff evidence without ROS runtime."""

    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)


class _ValidPrim:
    """Minimal valid prim occupying the canonical joint path."""

    def IsValid(self) -> bool:
        return True


class _ForeignJointStage:
    """Preserve a preexisting joint while recording forbidden removals."""

    def __init__(self) -> None:
        self._prim = _ValidPrim()
        self.remove_calls: list[str] = []

    def GetPrimAtPath(self, _path: str):
        return self._prim

    def RemovePrim(self, path: str) -> bool:
        self.remove_calls.append(path)
        return True


class _PartialFailureRelation:
    """Simulate a USD relation that mutates targets before reporting failure."""

    def __init__(self, targets: tuple[str, ...]) -> None:
        self.targets = list(targets)
        self.set_calls = 0

    def GetTargets(self) -> list[str]:
        return list(self.targets)

    def SetTargets(self, targets: list[str]) -> bool:
        self.targets = list(targets)
        self.set_calls += 1
        if self.set_calls == 1:
            raise RuntimeError("SetTargets failed after partial mutation")
        return True


class _FakeFilteredPairsAPI:
    """Route attach and detach relation access to the active fake relation."""

    relation: _PartialFailureRelation

    def __init__(self, _prim) -> None:
        pass

    @classmethod
    def Apply(cls, prim):
        return cls(prim)

    def CreateFilteredPairsRel(self) -> _PartialFailureRelation:
        return self.relation

    def GetFilteredPairsRel(self) -> _PartialFailureRelation:
        return self.relation


class _FilterFailureStage:
    """Expose an absent joint and record any unexpected joint removal."""

    def __init__(self) -> None:
        self.remove_calls: list[str] = []

    def GetPrimAtPath(self, path: str):
        return SimpleNamespace(
            IsValid=lambda: path != "/World/Franka/pick_cube_fixed_joint"
        )

    def RemovePrim(self, path: str) -> bool:
        self.remove_calls.append(path)
        return True


def _create_filter_only_pxr_module() -> ModuleType:
    pxr_module = ModuleType("pxr")
    pxr_module.Gf = SimpleNamespace()
    pxr_module.Sdf = SimpleNamespace(Path=lambda path: path)
    pxr_module.UsdGeom = SimpleNamespace()
    pxr_module.UsdPhysics = SimpleNamespace(
        FilteredPairsAPI=_FakeFilteredPairsAPI
    )
    return pxr_module


def _create_filter_failure_backend(
    stage: _FilterFailureStage,
) -> UsdPhysicalAttachmentBackend:
    backend = object.__new__(UsdPhysicalAttachmentBackend)
    backend._stage = stage
    backend._cube_path = "/World/PickCube"
    backend._hand_path = "/World/Franka/panda_hand"
    backend._tool_center_path = "/World/Franka/panda_hand/tool_center"
    backend._touch_body_paths = (
        backend._hand_path,
        "/World/Franka/panda_leftfinger",
        "/World/Franka/panda_rightfinger",
    )
    backend._joint_path = "/World/Franka/pick_cube_fixed_joint"
    backend._capture_distance_max_m = 0.03
    backend._owns_joint = False
    backend._owned_filter_paths = set()
    backend._captured_relative_pose = None
    backend._grasp_distance_m = lambda: 0.0
    backend._current_relative_transform_and_pose = (
        lambda: (
            _FakeRelativeTransform(),
            _MetricRelativePose(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0),
            ),
        )
    )
    return backend


class _FakeRelativeTransform:
    """Provide joint-local values while metric evidence stays plain Python."""

    def ExtractTranslation(self) -> tuple[float, float, float]:
        return (0.0, 0.0, 0.0)

    def ExtractRotationQuat(self) -> tuple[float, float, float, float]:
        return (1.0, 0.0, 0.0, 0.0)


class _BooleanWriteController:
    """Return false once for one named USD write and record all operations."""

    def __init__(self, failed_operation: str) -> None:
        self.failed_operation = failed_operation
        self.failed = False
        self.calls: list[str] = []

    def result(self, operation: str) -> bool:
        self.calls.append(operation)
        if operation == self.failed_operation and not self.failed:
            self.failed = True
            return False
        return True


class _BooleanRelation:
    """Fake USD relationship whose SetTargets has a meaningful bool result."""

    def __init__(
        self,
        writer: _BooleanWriteController,
        operation: str,
        targets: tuple[str, ...] = (),
    ) -> None:
        self._writer = writer
        self._operation = operation
        self.targets = list(targets)

    def GetTargets(self) -> list[str]:
        return list(self.targets)

    def SetTargets(self, targets: list[str]) -> bool:
        self.targets = list(targets)
        return self._writer.result(self._operation)


class _BooleanAttribute:
    """Fake USD attribute exposing Set's bool authoring result."""

    def __init__(
        self,
        writer: _BooleanWriteController,
        operation: str,
    ) -> None:
        self._writer = writer
        self._operation = operation

    def Set(self, _value) -> bool:
        return self._writer.result(self._operation)


class _BooleanJoint:
    """Fake FixedJoint with independently failing relationships and attributes."""

    def __init__(self, writer: _BooleanWriteController) -> None:
        self._prim = _ValidPrim()
        self._body0 = _BooleanRelation(writer, "body0")
        self._body1 = _BooleanRelation(writer, "body1")
        self._attributes = {
            "local_pos0": _BooleanAttribute(writer, "local_pos0"),
            "local_rot0": _BooleanAttribute(writer, "local_rot0"),
            "local_pos1": _BooleanAttribute(writer, "local_pos1"),
            "local_rot1": _BooleanAttribute(writer, "local_rot1"),
            "break_force": _BooleanAttribute(writer, "break_force"),
            "break_torque": _BooleanAttribute(writer, "break_torque"),
            "collision_enabled": _BooleanAttribute(writer, "collision_enabled"),
            "exclude_from_articulation": _BooleanAttribute(
                writer,
                "exclude_from_articulation",
            ),
        }

    def GetPrim(self) -> _ValidPrim:
        return self._prim

    def CreateBody0Rel(self) -> _BooleanRelation:
        return self._body0

    def CreateBody1Rel(self) -> _BooleanRelation:
        return self._body1

    def GetBody0Rel(self) -> _BooleanRelation:
        return self._body0

    def GetBody1Rel(self) -> _BooleanRelation:
        return self._body1

    def CreateLocalPos0Attr(self) -> _BooleanAttribute:
        return self._attributes["local_pos0"]

    def CreateLocalRot0Attr(self) -> _BooleanAttribute:
        return self._attributes["local_rot0"]

    def CreateLocalPos1Attr(self) -> _BooleanAttribute:
        return self._attributes["local_pos1"]

    def CreateLocalRot1Attr(self) -> _BooleanAttribute:
        return self._attributes["local_rot1"]

    def CreateBreakForceAttr(self) -> _BooleanAttribute:
        return self._attributes["break_force"]

    def CreateBreakTorqueAttr(self) -> _BooleanAttribute:
        return self._attributes["break_torque"]

    def CreateCollisionEnabledAttr(self) -> _BooleanAttribute:
        return self._attributes["collision_enabled"]

    def CreateExcludeFromArticulationAttr(self) -> _BooleanAttribute:
        return self._attributes["exclude_from_articulation"]


class _BooleanWriteStage:
    """Own fake relation/joint state for fail-closed USD authoring tests."""

    def __init__(
        self,
        *,
        failed_operation: str,
        remove_succeeds: bool = True,
    ) -> None:
        self.writer = _BooleanWriteController(failed_operation)
        self.filtered_relation = _BooleanRelation(
            self.writer,
            "filtered_pairs",
            ("/World/Foreign",),
        )
        self.joint: _BooleanJoint | None = None
        self.remove_succeeds = remove_succeeds

    def GetPrimAtPath(self, path: str):
        if path == "/World/Franka/pick_cube_fixed_joint":
            if self.joint is None:
                return SimpleNamespace(IsValid=lambda: False)
            return self.joint.GetPrim()
        return _ValidPrim()

    def define_joint(self) -> _BooleanJoint:
        self.joint = _BooleanJoint(self.writer)
        return self.joint

    def RemovePrim(self, _path: str) -> bool:
        if not self.remove_succeeds:
            return False
        self.joint = None
        return True


def _create_boolean_write_pxr_module(
    stage: _BooleanWriteStage,
) -> ModuleType:
    class _FilteredPairs:
        @classmethod
        def Apply(cls, _prim):
            return cls()

        def CreateFilteredPairsRel(self) -> _BooleanRelation:
            return stage.filtered_relation

        def GetFilteredPairsRel(self) -> _BooleanRelation:
            return stage.filtered_relation

    pxr_module = ModuleType("pxr")
    pxr_module.Gf = SimpleNamespace(
        Vec3f=lambda *values: values,
        Quatf=lambda *values: values,
    )
    pxr_module.Sdf = SimpleNamespace(Path=lambda path: path)
    pxr_module.UsdGeom = SimpleNamespace()
    pxr_module.UsdPhysics = SimpleNamespace(
        FilteredPairsAPI=_FilteredPairs,
        FixedJoint=SimpleNamespace(
            Define=lambda _stage, _path: stage.define_joint(),
            Get=lambda _stage, _path: stage.joint,
        ),
    )
    return pxr_module


def _create_boolean_write_backend(
    stage: _BooleanWriteStage,
) -> UsdPhysicalAttachmentBackend:
    backend = object.__new__(UsdPhysicalAttachmentBackend)
    backend._stage = stage
    backend._cube_path = "/World/PickCube"
    backend._hand_path = "/World/Franka/panda_hand"
    backend._tool_center_path = "/World/Franka/panda_hand/tool_center"
    backend._touch_body_paths = (
        backend._hand_path,
        "/World/Franka/panda_leftfinger",
        "/World/Franka/panda_rightfinger",
    )
    backend._joint_path = "/World/Franka/pick_cube_fixed_joint"
    backend._capture_distance_max_m = 0.03
    backend._owns_joint = False
    backend._owned_filter_paths = set()
    backend._captured_relative_pose = None
    backend._grasp_distance_m = lambda: 0.0
    backend._current_relative_transform_and_pose = (
        lambda: (
            _FakeRelativeTransform(),
            _MetricRelativePose(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0),
            ),
        )
    )
    return backend


class _World:
    """Record simulation-loop ordering."""

    def __init__(self, calls: list[str]) -> None:
        self._calls = calls

    def play(self) -> None:
        self._calls.append("world.play")

    def step(self, *, render: bool) -> None:
        assert render is False
        self._calls.append("world.step")


class _App:
    """Bound the pure loop without owning cleanup."""

    def __init__(self, *, max_checks: int) -> None:
        self._checks_remaining = max_checks

    def is_running(self) -> bool:
        self._checks_remaining -= 1
        return self._checks_remaining >= 0

    def update(self) -> None:
        """Match the headless SimulationApp API."""
