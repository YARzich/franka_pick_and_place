"""
Isaac-side physical attachment barrier for the canonical Panda pick cube.

Responsibility:
- Serve the object-scoped ``SetBool`` attachment endpoint.
- Queue ROS requests without touching USD from the executor thread.
- Apply and verify physical attachment changes on the simulation thread.

Does not own:
- MoveIt PlanningScene attachment state or workflow policy.
- Gripper actions, articulation commands, or task recovery decisions.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
from typing import Callable, Protocol, Sequence


DEFAULT_ATTACHMENT_SERVICE = "/franka_motion/isaac/set_pick_cube_attached"
_HAND_LINK_NAME = "panda_hand"
_TOOL_CENTER_RELATIVE_PATH = "panda_hand/tool_center"
_TOUCH_LINK_NAMES = ("panda_hand", "panda_leftfinger", "panda_rightfinger")
_JOINT_NAME = "pick_cube_fixed_joint"
_MAX_FLOAT = 3.4028234663852886e38
_DEFAULT_CAPTURE_DISTANCE_MAX_M = 0.03
_ATTACHED_TRANSLATION_TOLERANCE_M = 0.001
_ATTACHED_ROTATION_TOLERANCE_RAD = math.radians(1.0)
_SHUTDOWN_WAIT_SEC = 5.0


@dataclass(frozen=True)
class AttachmentResult:
    """Terminal result returned to one object-scoped service request."""

    success: bool
    message: str


@dataclass(frozen=True)
class _MetricRelativePose:
    """Cube pose relative to the hand, independent of USD stage units."""

    translation_m: tuple[float, float, float]
    rotation_wxyz: tuple[float, float, float, float]


@dataclass
class _PendingAttachmentRequest:
    desired_attached: bool
    completed: bool = False
    result: AttachmentResult | None = None
    apply_error: str = ""


class PhysicalAttachmentBackend(Protocol):
    """USD operations invoked only from simulation-loop hooks."""

    def apply_attached(self, desired_attached: bool) -> None:
        """Apply an idempotent desired physical state."""

    def verify_attached(self, desired_attached: bool) -> None:
        """Verify the desired state after at least one physics step."""

    def confirm_attached(self, desired_attached: bool) -> str | None:
        """Commit bookkeeping and optionally return stepped ground-truth detail."""


class PhysicalAttachmentBarrier:
    """Thread-safe handoff between a ROS callback and the simulation thread."""

    def __init__(self, backend: PhysicalAttachmentBackend) -> None:
        self._backend = backend
        self._condition = threading.Condition()
        self._pending: deque[_PendingAttachmentRequest] = deque()
        self._active: _PendingAttachmentRequest | None = None
        self._closed = False
        self._simulation_thread_id: int | None = None
        self._event_logger = None

    def request(self, desired_attached: bool) -> AttachmentResult:
        """Queue a request and wait without a semantic timeout for its stepped result."""
        request = _PendingAttachmentRequest(bool(desired_attached))
        with self._condition:
            if self._closed:
                return _shutdown_result()
            self._pending.append(request)
            self._condition.notify_all()
            while not request.completed and not self._closed:
                self._condition.wait()
            if request.result is not None:
                return request.result
            return _shutdown_result()

    def before_step(self) -> None:
        """Apply at most one queued request immediately before a world step."""
        self._assert_simulation_thread()
        with self._condition:
            if self._closed or self._active is not None or not self._pending:
                return
            request = self._pending.popleft()
            self._active = request

        self._log_info(
            "physical_attachment simulation apply "
            f"desired={_attachment_state(request.desired_attached)}"
        )
        try:
            self._backend.apply_attached(request.desired_attached)
        except Exception as error:  # The failed request is still acknowledged after a step.
            request.apply_error = str(error) or type(error).__name__

    def after_step(self) -> None:
        """Verify and acknowledge the active request after the completed world step."""
        self._assert_simulation_thread()
        with self._condition:
            request = self._active
            if self._closed or request is None:
                return

        result = self._result_after_step(request)
        self._log_info(
            "physical_attachment simulation ACK "
            f"desired={_attachment_state(request.desired_attached)} "
            f"success={str(result.success).lower()}"
        )
        with self._condition:
            if request.completed:
                return
            request.result = result
            request.completed = True
            self._active = None
            self._condition.notify_all()

    def close(self) -> None:
        """Fail all pending requests so executor callbacks can finish during shutdown."""
        with self._condition:
            if self._closed:
                return
            self._closed = True
            result = _shutdown_result()
            requests = list(self._pending)
            self._pending.clear()
            if self._active is not None:
                requests.append(self._active)
                self._active = None
            for request in requests:
                request.result = result
                request.completed = True
            self._condition.notify_all()

    @property
    def pending_request_count(self) -> int:
        """Return queued plus active requests for diagnostics and fast tests."""
        with self._condition:
            return len(self._pending) + int(self._active is not None)

    def _set_event_logger(self, logger) -> None:
        """Install the ROS logger before the service executor starts."""
        self._event_logger = logger

    def _log_service_request(self, desired_attached: bool) -> None:
        self._log_info(
            "physical_attachment service request "
            f"desired={_attachment_state(desired_attached)}"
        )

    def _result_after_step(
        self,
        request: _PendingAttachmentRequest,
    ) -> AttachmentResult:
        if request.apply_error:
            return AttachmentResult(False, request.apply_error)
        try:
            self._backend.verify_attached(request.desired_attached)
            detail = self._backend.confirm_attached(request.desired_attached)
        except Exception as error:
            return AttachmentResult(False, str(error) or type(error).__name__)
        state = "attached" if request.desired_attached else "detached"
        message = f"pick cube physical state confirmed {state}"
        if detail:
            message = f"{message}; {detail}"
        return AttachmentResult(True, message)

    def _assert_simulation_thread(self) -> None:
        current_thread_id = threading.get_ident()
        if self._simulation_thread_id is None:
            self._simulation_thread_id = current_thread_id
            return
        if self._simulation_thread_id != current_thread_id:
            raise RuntimeError(
                "Isaac physical attachment hooks вызваны не из simulation thread"
            )

    def _log_info(self, message: str) -> None:
        if self._event_logger is not None:
            self._event_logger.info(message)


class SetBoolAttachmentCallback:
    """Pure response adapter used by the rclpy service."""

    def __init__(self, barrier: PhysicalAttachmentBarrier) -> None:
        self._barrier = barrier

    def __call__(self, request, response):
        desired_attached = bool(request.data)
        self._barrier._log_service_request(desired_attached)
        result = self._barrier.request(desired_attached)
        response.success = result.success
        response.message = result.message
        return response


class RclpySetBoolAttachmentService:
    """Own a private rclpy context and one single-threaded service executor."""

    def __init__(
        self,
        barrier: PhysicalAttachmentBarrier,
        *,
        service_name: str = DEFAULT_ATTACHMENT_SERVICE,
    ) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from std_srvs.srv import SetBool

        self._barrier = barrier
        self._context = Context()
        self._executor = None
        self._node = None
        self._service = None
        self._thread = None
        self._spin_error: Exception | None = None
        self._state_lock = threading.Lock()
        self._closing = False

        try:
            rclpy.init(
                args=[],
                context=self._context,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            self._node = rclpy.create_node(
                "franka_isaac_pick_cube_attachment",
                context=self._context,
                cli_args=[],
                use_global_arguments=False,
                start_parameter_services=False,
            )
            logger = self._node.get_logger()
            barrier._set_event_logger(logger)
            callback = SetBoolAttachmentCallback(barrier)
            self._service = self._node.create_service(
                SetBool,
                service_name,
                callback,
            )
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._spin,
                name="franka-isaac-physical-attachment",
            )
            self._thread.start()
        except Exception:
            self._close_partial()
            raise

    def raise_if_failed(self) -> None:
        """Surface an unexpected executor failure on the simulation thread."""
        with self._state_lock:
            error = self._spin_error
        if error is not None:
            raise RuntimeError(
                "ROS executor physical attachment service завершился с ошибкой"
            ) from error

    def close(self) -> None:
        """Complete callbacks, stop the executor, and release its private context."""
        with self._state_lock:
            if self._closing:
                return
            self._closing = True
        self._barrier.close()

        errors: list[Exception] = []
        if self._executor is not None:
            try:
                stopped = self._executor.shutdown(timeout_sec=_SHUTDOWN_WAIT_SEC)
                if not stopped:
                    errors.append(RuntimeError("rclpy attachment executor не остановился"))
            except Exception as error:
                errors.append(error)
        if self._thread is not None:
            self._thread.join(timeout=_SHUTDOWN_WAIT_SEC)
            if self._thread.is_alive():
                errors.append(RuntimeError("Поток attachment service не остановился"))
        if self._executor is not None and self._node is not None:
            try:
                self._executor.remove_node(self._node)
            except Exception as error:
                errors.append(error)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception as error:
                errors.append(error)
        try:
            self._context.try_shutdown()
        except Exception as error:
            errors.append(error)
        if errors:
            raise RuntimeError("Не удалось закрыть Isaac attachment service") from errors[0]

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception as error:
            with self._state_lock:
                if not self._closing:
                    self._spin_error = error
            self._barrier.close()

    def _close_partial(self) -> None:
        self._barrier.close()
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=_SHUTDOWN_WAIT_SEC)
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        try:
            self._context.try_shutdown()
        except Exception:
            pass


class IsaacPickCubeAttachmentService:
    """Compose the stepped barrier and the object-scoped ROS service owner."""

    def __init__(
        self,
        backend: PhysicalAttachmentBackend,
        *,
        service_name: str = DEFAULT_ATTACHMENT_SERVICE,
        service_factory: Callable[..., object] | None = None,
    ) -> None:
        self._backend = backend
        self._simulation_thread_id = threading.get_ident()
        self._barrier = PhysicalAttachmentBarrier(backend)
        factory = (
            RclpySetBoolAttachmentService
            if service_factory is None
            else service_factory
        )
        self._service = factory(
            self._barrier,
            service_name=service_name,
        )

    def before_step(self) -> None:
        """Apply a queued request from the simulation thread."""
        self._assert_simulation_thread()
        self._service.raise_if_failed()
        self._barrier.before_step()

    def after_step(self) -> None:
        """Verify and acknowledge the active request after physics."""
        self._assert_simulation_thread()
        self._barrier.after_step()
        self._service.raise_if_failed()

    def close(self) -> None:
        """Stop ROS first, then remove owned USD state before World.stop."""
        self._assert_simulation_thread()
        errors: list[Exception] = []
        try:
            self._service.close()
        except Exception as error:
            errors.append(error)

        cleanup_verified = True
        try:
            self._backend.apply_attached(False)
        except Exception as error:
            errors.append(error)
            cleanup_verified = False
        try:
            self._backend.verify_attached(False)
        except Exception as error:
            errors.append(error)
            cleanup_verified = False
        if cleanup_verified:
            try:
                self._backend.confirm_attached(False)
            except Exception as error:
                errors.append(error)

        if errors:
            aggregate = RuntimeError(
                "Не удалось полностью закрыть Isaac physical attachment"
            )
            for error in errors[1:]:
                aggregate.add_note(str(error))
            raise aggregate from errors[0]

    def _assert_simulation_thread(self) -> None:
        if threading.get_ident() != self._simulation_thread_id:
            raise RuntimeError(
                "Isaac physical attachment lifecycle вызван не из simulation thread"
            )


class UsdPhysicalAttachmentBackend:
    """Author and remove the owned runtime USD fixed joint and collision filters."""

    def __init__(
        self,
        stage,
        *,
        articulation_path: str,
        cube_path: str,
        capture_distance_max_m: float = _DEFAULT_CAPTURE_DISTANCE_MAX_M,
    ) -> None:
        if not math.isfinite(capture_distance_max_m) or capture_distance_max_m < 0.0:
            raise ValueError("Capture distance must be finite and nonnegative")
        root = articulation_path.rstrip("/")
        self._stage = stage
        self._cube_path = cube_path
        self._hand_path = f"{root}/{_HAND_LINK_NAME}"
        self._tool_center_path = f"{root}/{_TOOL_CENTER_RELATIVE_PATH}"
        self._touch_body_paths = tuple(f"{root}/{name}" for name in _TOUCH_LINK_NAMES)
        self._joint_path = f"{root}/{_JOINT_NAME}"
        self._capture_distance_max_m = capture_distance_max_m
        self._owns_joint = False
        self._owned_filter_paths: set[str] = set()
        self._captured_relative_pose: _MetricRelativePose | None = None
        self._validate_paths()

    def apply_attached(self, desired_attached: bool) -> None:
        """Apply an idempotent attach or detach mutation."""
        if desired_attached:
            self._attach()
        else:
            self._detach()

    def verify_attached(self, desired_attached: bool) -> None:
        """Verify joint and owned filters after PhysX consumed the USD mutation."""
        if desired_attached:
            self._require_owned_joint()
            targets = set(self._filtered_targets())
            missing = set(self._touch_body_paths) - targets
            if missing:
                raise RuntimeError(
                    "Isaac не активировал collision filters: " + ", ".join(sorted(missing))
                )
            if self._captured_relative_pose is None:
                raise RuntimeError(
                    "Captured cube/hand relative pose отсутствует для owned joint"
                )
            _require_relative_pose_close(
                self._captured_relative_pose,
                self._current_relative_pose(),
            )
            return

        if self._stage.GetPrimAtPath(self._joint_path).IsValid():
            raise RuntimeError("Isaac не удалил physical cube attachment joint")
        remaining_owned = self._owned_filter_paths.intersection(self._filtered_targets())
        if remaining_owned:
            raise RuntimeError(
                "Isaac не удалил owned collision filters: "
                + ", ".join(sorted(remaining_owned))
            )

    def confirm_attached(self, desired_attached: bool) -> str:
        """Commit ownership and report stepped USD cube ground truth."""
        if not desired_attached:
            self._owns_joint = False
            self._owned_filter_paths.clear()
            self._captured_relative_pose = None
        return self._cube_world_position_detail()

    def _attach(self) -> None:
        from pxr import Gf, Sdf, UsdPhysics

        existing_joint = self._owned_joint_or_none()
        relative_transform = None
        captured_relative_pose = self._captured_relative_pose
        if existing_joint is None:
            distance = self._grasp_distance_m()
            _require_capture_distance(
                distance,
                maximum_m=self._capture_distance_max_m,
            )
            relative_transform, captured_relative_pose = (
                self._current_relative_transform_and_pose()
            )
        elif captured_relative_pose is None:
            raise RuntimeError(
                "Owned physical attachment joint не имеет captured relative pose"
            )

        cube_prim = self._stage.GetPrimAtPath(self._cube_path)
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(cube_prim)
        filtered_relation = filtered_pairs.CreateFilteredPairsRel()
        original_targets = tuple(str(path) for path in filtered_relation.GetTargets())
        merged_targets, newly_owned = _merge_required_targets(
            original_targets,
            self._touch_body_paths,
        )

        joint_authoring_started = False
        previous_owns_joint = self._owns_joint
        previous_owned_filters = set(self._owned_filter_paths)
        previous_relative_pose = self._captured_relative_pose
        try:
            _require_usd_write_succeeded(
                filtered_relation.SetTargets(
                    [Sdf.Path(path) for path in merged_targets]
                ),
                "cube collision-filter targets",
            )
            if existing_joint is None:
                joint_authoring_started = True
                joint = UsdPhysics.FixedJoint.Define(self._stage, self._joint_path)
                if not joint or not joint.GetPrim().IsValid():
                    raise RuntimeError(
                        "USD FixedJoint definition не создала валидный prim"
                    )
                _require_usd_write_succeeded(
                    joint.CreateBody0Rel().SetTargets([Sdf.Path(self._hand_path)]),
                    "FixedJoint body0 relationship",
                )
                _require_usd_write_succeeded(
                    joint.CreateBody1Rel().SetTargets([Sdf.Path(self._cube_path)]),
                    "FixedJoint body1 relationship",
                )
                _require_usd_write_succeeded(
                    joint.CreateLocalPos0Attr().Set(
                        Gf.Vec3f(relative_transform.ExtractTranslation())
                    ),
                    "FixedJoint localPos0 attribute",
                )
                _require_usd_write_succeeded(
                    joint.CreateLocalRot0Attr().Set(
                        Gf.Quatf(relative_transform.ExtractRotationQuat())
                    ),
                    "FixedJoint localRot0 attribute",
                )
                _require_usd_write_succeeded(
                    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0)),
                    "FixedJoint localPos1 attribute",
                )
                _require_usd_write_succeeded(
                    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0)),
                    "FixedJoint localRot1 attribute",
                )
                _require_usd_write_succeeded(
                    joint.CreateBreakForceAttr().Set(_MAX_FLOAT),
                    "FixedJoint breakForce attribute",
                )
                _require_usd_write_succeeded(
                    joint.CreateBreakTorqueAttr().Set(_MAX_FLOAT),
                    "FixedJoint breakTorque attribute",
                )
                _require_usd_write_succeeded(
                    joint.CreateCollisionEnabledAttr().Set(False),
                    "FixedJoint collisionEnabled attribute",
                )
                _require_usd_write_succeeded(
                    joint.CreateExcludeFromArticulationAttr().Set(True),
                    "FixedJoint excludeFromArticulation attribute",
                )
                self._owns_joint = True
                self._captured_relative_pose = captured_relative_pose
            self._owned_filter_paths.update(newly_owned)
        except Exception as error:
            joint_rollback_succeeded = not joint_authoring_started
            if joint_authoring_started:
                try:
                    if not self._stage.RemovePrim(self._joint_path):
                        raise RuntimeError(
                            "Attach joint rollback RemovePrim вернул false"
                        )
                    joint_rollback_succeeded = True
                except Exception as rollback_error:
                    error.add_note(f"Attach joint rollback failed: {rollback_error}")
            filter_rollback_succeeded = True
            try:
                _require_usd_write_succeeded(
                    filtered_relation.SetTargets(
                        [Sdf.Path(path) for path in original_targets]
                    ),
                    "attach collision-filter rollback",
                )
            except Exception as rollback_error:
                filter_rollback_succeeded = False
                error.add_note(
                    f"Attach collision-filter rollback failed: {rollback_error}"
                )
            if joint_rollback_succeeded:
                self._owns_joint = previous_owns_joint
                self._captured_relative_pose = previous_relative_pose
            else:
                self._owns_joint = True
                self._captured_relative_pose = captured_relative_pose
            if filter_rollback_succeeded:
                self._owned_filter_paths = previous_owned_filters
            else:
                self._owned_filter_paths = previous_owned_filters.union(newly_owned)
            raise

    def _detach(self) -> None:
        from pxr import Sdf, UsdPhysics

        joint_prim = self._owned_joint_prim_or_none()
        relation = None
        original_targets: tuple[str, ...] = ()
        if self._owned_filter_paths:
            cube_prim = self._stage.GetPrimAtPath(self._cube_path)
            filtered_pairs = UsdPhysics.FilteredPairsAPI(cube_prim)
            relation = filtered_pairs.GetFilteredPairsRel()
            if not relation:
                raise RuntimeError("Owned physical cube collision filters отсутствуют")
            original_targets = tuple(str(path) for path in relation.GetTargets())
            remaining_targets = _remove_owned_targets(
                original_targets,
                self._owned_filter_paths,
            )

        try:
            if relation is not None:
                _require_usd_write_succeeded(
                    relation.SetTargets(
                        [Sdf.Path(path) for path in remaining_targets]
                    ),
                    "detach collision-filter targets",
                )
            if joint_prim is not None and not self._stage.RemovePrim(self._joint_path):
                raise RuntimeError(
                    "Isaac не удалил physical cube attachment joint"
                )
        except Exception as error:
            if relation is not None:
                try:
                    _require_usd_write_succeeded(
                        relation.SetTargets(
                            [Sdf.Path(path) for path in original_targets]
                        ),
                        "detach collision-filter rollback",
                    )
                except Exception as rollback_error:
                    error.add_note(
                        f"Detach collision-filter rollback failed: {rollback_error}"
                    )
            raise

    def _current_relative_pose(self) -> _MetricRelativePose:
        _, relative_pose = self._current_relative_transform_and_pose()
        return relative_pose

    def _current_relative_transform_and_pose(self):
        from pxr import UsdGeom

        cache = UsdGeom.XformCache()
        cube_pose = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._cube_path)
        )
        hand_pose = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._hand_path)
        )
        relative_transform = (
            cube_pose * hand_pose.GetInverse()
        ).RemoveScaleShear()
        translation = relative_transform.ExtractTranslation()
        rotation = relative_transform.ExtractRotationQuat()
        imaginary = rotation.GetImaginary()
        meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(self._stage))
        relative_pose = _make_metric_relative_pose(
            translation,
            (
                float(rotation.GetReal()),
                float(imaginary[0]),
                float(imaginary[1]),
                float(imaginary[2]),
            ),
            meters_per_unit=meters_per_unit,
        )
        return relative_transform, relative_pose

    def _grasp_distance_m(self) -> float:
        from pxr import UsdGeom

        cache = UsdGeom.XformCache()
        cube_position = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._cube_path)
        ).ExtractTranslation()
        tool_position = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._tool_center_path)
        ).ExtractTranslation()
        stage_distance = float((cube_position - tool_position).GetLength())
        return stage_distance * float(UsdGeom.GetStageMetersPerUnit(self._stage))

    def _cube_world_position_detail(self) -> str:
        from pxr import UsdGeom

        cache = UsdGeom.XformCache()
        cube_position = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._cube_path)
        ).ExtractTranslation()
        meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(self._stage))
        return _format_cube_world_position_m(
            tuple(float(value) * meters_per_unit for value in cube_position)
        )

    def _owned_joint_or_none(self):
        from pxr import UsdPhysics

        prim = self._owned_joint_prim_or_none()
        if prim is None:
            return None
        joint = UsdPhysics.FixedJoint.Get(self._stage, self._joint_path)
        if not joint or not joint.GetPrim().IsValid():
            raise RuntimeError(
                "Owned physical attachment path занят несовместимым prim"
            )
        body0 = [str(path) for path in joint.GetBody0Rel().GetTargets()]
        body1 = [str(path) for path in joint.GetBody1Rel().GetTargets()]
        if body0 != [self._hand_path] or body1 != [self._cube_path]:
            raise RuntimeError(
                "Owned physical attachment joint имеет несовместимые bodies"
            )
        return joint

    def _owned_joint_prim_or_none(self):
        prim = self._stage.GetPrimAtPath(self._joint_path)
        if not prim.IsValid():
            return None
        if not self._owns_joint:
            raise RuntimeError(
                "Physical attachment joint path занят чужим prim; "
                "backend не имеет права его удалять"
            )
        return prim

    def _require_owned_joint(self):
        joint = self._owned_joint_or_none()
        if joint is None:
            raise RuntimeError("Isaac не активировал physical cube attachment joint")
        if joint.GetCollisionEnabledAttr().Get() is not False:
            raise RuntimeError("Physical attachment joint collision должен быть выключен")
        if joint.GetExcludeFromArticulationAttr().Get() is not True:
            raise RuntimeError("Physical attachment joint попал внутрь articulation")
        return joint

    def _filtered_targets(self) -> tuple[str, ...]:
        from pxr import UsdPhysics

        cube_prim = self._stage.GetPrimAtPath(self._cube_path)
        relation = UsdPhysics.FilteredPairsAPI(cube_prim).GetFilteredPairsRel()
        if not relation:
            return ()
        return tuple(str(path) for path in relation.GetTargets())

    def _validate_paths(self) -> None:
        from pxr import UsdPhysics

        required_paths = (
            self._cube_path,
            self._hand_path,
            self._tool_center_path,
            *self._touch_body_paths,
        )
        missing = [
            path
            for path in required_paths
            if not self._stage.GetPrimAtPath(path).IsValid()
        ]
        if missing:
            raise RuntimeError(
                "Isaac physical grasp prims отсутствуют: " + ", ".join(missing)
            )
        rigid_paths = (self._cube_path, self._hand_path, *self._touch_body_paths[1:])
        for body_path in rigid_paths:
            if not self._stage.GetPrimAtPath(body_path).HasAPI(
                UsdPhysics.RigidBodyAPI
            ):
                raise RuntimeError(
                    f"Isaac physical grasp body не является rigid: {body_path}"
                )


def create_pick_cube_attachment_service(
    stage,
    *,
    articulation_path: str,
    cube_path: str,
    service_name: str = DEFAULT_ATTACHMENT_SERVICE,
) -> IsaacPickCubeAttachmentService:
    """Create the canonical physical attachment barrier and its ROS owner."""
    backend = UsdPhysicalAttachmentBackend(
        stage,
        articulation_path=articulation_path,
        cube_path=cube_path,
    )
    return IsaacPickCubeAttachmentService(
        backend,
        service_name=service_name,
    )


def _merge_required_targets(
    existing: Sequence[str],
    required: Sequence[str],
) -> tuple[tuple[str, ...], set[str]]:
    existing_targets = tuple(existing)
    known = set(existing_targets)
    newly_owned = {target for target in required if target not in known}
    merged = existing_targets + tuple(
        target for target in required if target in newly_owned
    )
    return merged, newly_owned


def _remove_owned_targets(
    existing: Sequence[str],
    owned: set[str],
) -> tuple[str, ...]:
    return tuple(target for target in existing if target not in owned)


def _require_capture_distance(distance_m: float, *, maximum_m: float) -> None:
    if not math.isfinite(distance_m):
        raise RuntimeError("Pick cube/tool center distance не является конечным")
    if distance_m > maximum_m:
        raise RuntimeError(
            "Pick cube слишком далеко от tool center: "
            f"{distance_m:.6f} m > {maximum_m:.6f} m"
        )


def _make_metric_relative_pose(
    translation_stage_units: Sequence[float],
    rotation_wxyz: Sequence[float],
    *,
    meters_per_unit: float,
) -> _MetricRelativePose:
    if (
        not math.isfinite(meters_per_unit)
        or meters_per_unit <= 0.0
    ):
        raise RuntimeError("USD metersPerUnit должен быть конечным и положительным")
    if len(translation_stage_units) != 3:
        raise RuntimeError("Relative translation должна содержать три координаты")
    translation_m = tuple(
        float(value) * meters_per_unit
        for value in translation_stage_units
    )
    if not all(math.isfinite(value) for value in translation_m):
        raise RuntimeError("Relative translation не является конечной")
    return _MetricRelativePose(
        translation_m=translation_m,
        rotation_wxyz=_normalize_quaternion(rotation_wxyz),
    )


def _relative_pose_errors(
    expected: _MetricRelativePose,
    actual: _MetricRelativePose,
) -> tuple[float, float]:
    if len(expected.translation_m) != 3 or len(actual.translation_m) != 3:
        raise RuntimeError("Relative translation должна содержать три координаты")
    values = (*expected.translation_m, *actual.translation_m)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Relative translation не является конечной")
    translation_error_m = math.sqrt(
        sum(
            (actual_value - expected_value) ** 2
            for expected_value, actual_value in zip(
                expected.translation_m,
                actual.translation_m,
            )
        )
    )
    expected_rotation = _normalize_quaternion(expected.rotation_wxyz)
    actual_rotation = _normalize_quaternion(actual.rotation_wxyz)
    quaternion_dot = abs(
        sum(
            expected_value * actual_value
            for expected_value, actual_value in zip(
                expected_rotation,
                actual_rotation,
            )
        )
    )
    rotation_error_rad = 2.0 * math.acos(
        min(1.0, max(0.0, quaternion_dot))
    )
    return translation_error_m, rotation_error_rad


def _require_relative_pose_close(
    expected: _MetricRelativePose,
    actual: _MetricRelativePose,
) -> None:
    translation_error_m, rotation_error_rad = _relative_pose_errors(
        expected,
        actual,
    )
    if (
        translation_error_m > _ATTACHED_TRANSLATION_TOLERANCE_M
        or rotation_error_rad > _ATTACHED_ROTATION_TOLERANCE_RAD
    ):
        raise RuntimeError(
            "Isaac FixedJoint не сохранил captured cube/hand relative pose: "
            f"translation_error={translation_error_m:.6f} m "
            f"(limit={_ATTACHED_TRANSLATION_TOLERANCE_M:.6f} m), "
            f"rotation_error={math.degrees(rotation_error_rad):.3f} deg "
            f"(limit={math.degrees(_ATTACHED_ROTATION_TOLERANCE_RAD):.3f} deg)"
        )


def _normalize_quaternion(
    rotation_wxyz: Sequence[float],
) -> tuple[float, float, float, float]:
    if len(rotation_wxyz) != 4:
        raise RuntimeError("Relative rotation quaternion должен содержать четыре числа")
    values = tuple(float(value) for value in rotation_wxyz)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Relative rotation quaternion не является конечным")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise RuntimeError("Relative rotation quaternion имеет нулевую норму")
    return tuple(value / norm for value in values)


def _require_usd_write_succeeded(result: object, operation: str) -> None:
    if result is not True:
        raise RuntimeError(f"USD write не подтверждён: {operation}")


def _format_cube_world_position_m(position_m: Sequence[float]) -> str:
    if len(position_m) != 3:
        raise RuntimeError("Cube world position должна содержать три координаты")
    values = tuple(float(value) for value in position_m)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Cube world position не является конечной")
    normalized = tuple(0.0 if abs(value) < 0.5e-6 else value for value in values)
    return "cube_world_m=({:.6f},{:.6f},{:.6f})".format(*normalized)


def _attachment_state(desired_attached: bool) -> str:
    return "attached" if desired_attached else "detached"


def _shutdown_result() -> AttachmentResult:
    return AttachmentResult(
        False,
        "Isaac physical attachment service is shutting down",
    )


__all__ = [
    "AttachmentResult",
    "DEFAULT_ATTACHMENT_SERVICE",
    "IsaacPickCubeAttachmentService",
    "PhysicalAttachmentBarrier",
    "RclpySetBoolAttachmentService",
    "SetBoolAttachmentCallback",
    "UsdPhysicalAttachmentBackend",
    "create_pick_cube_attachment_service",
]
