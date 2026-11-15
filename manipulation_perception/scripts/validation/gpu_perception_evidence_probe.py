#!/usr/bin/env python3

"""
Read-only ROS adapter for bounded GPU perception runtime evidence.

Responsibility:
    Subscribe to scenario-declared stamped topics and exact diagnostics, observe
    publisher types, collect explicit GPU/model provenance, and write one artifact.

Does not own:
    Pipeline startup, inference, topic/frame defaults, recovery, accuracy, or replay.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import sys
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.utilities import get_message


def _add_source_package_root_to_path() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "manipulation_perception").is_dir():
            parent_text = str(parent)
            if parent_text not in sys.path:
                sys.path.insert(0, parent_text)
            return


_add_source_package_root_to_path()

from manipulation_perception.validation.artifact import (  # noqa: E402,I100,I101,I201
    build_artifact,
    utc_now,
    validate_evidence_identity,
    write_artifact,
)
from manipulation_perception.validation.contracts import (  # noqa: E402,I100,I101,I201
    ContractError,
    GpuPerceptionContract,
    load_contract,
)
from manipulation_perception.validation.diagnostics import (  # noqa: E402,I100,I101,I201
    DiagnosticContractError,
    key_values_to_mapping,
    require_unique_status_names,
)
from manipulation_perception.validation.evidence import (  # noqa: E402,I100,I101,I201
    EvidenceSession,
)
from manipulation_perception.validation.provenance import (  # noqa: E402,I100,I101,I201
    collect_gpu_identity,
    hash_assets,
    ProvenanceError,
)


class GpuPerceptionEvidenceProbeNode(Node):
    """Adapt ROS graph observations into one pure bounded evidence session."""

    def __init__(self, session: EvidenceSession) -> None:
        super().__init__("gpu_perception_evidence_probe")
        self._session = session
        self._topic_subscriptions = []
        for topic in session.contract.topics:
            message_class = get_message(topic.message_type)
            subscription = self.create_subscription(
                message_class,
                topic.topic,
                lambda message, item=topic: self._on_topic(item, message),
                qos_profile_sensor_data,
            )
            self._topic_subscriptions.append(subscription)
        self._status_names = {
            item.status_name for item in session.contract.diagnostics
        }
        self._diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._on_diagnostics,
            20,
        )
        self._graph_timer = self.create_timer(0.5, self._observe_graph_types)
        self._observe_graph_types()

    def _on_topic(self, contract: Any, message: Any) -> None:
        """Extract header/content metadata and hash serialized bytes once."""
        try:
            header = message.header
            stamp = header.stamp
            payload = None
            payload_sha256 = None
            if contract.capture_payload_digest:
                payload = bytes(serialize_message(message))
                payload_sha256 = hashlib.sha256(payload).hexdigest()
            content_count = _content_count(message, contract.nonempty_field)
            if contract.role == self._session.contract.terminal_pose_role:
                _require_finite_pose(message)
            self._session.record_topic_sample(
                role=contract.role,
                message_type=contract.message_type,
                frame_id=str(header.frame_id),
                stamp_sec=int(stamp.sec),
                stamp_nanosec=int(stamp.nanosec),
                received_monotonic=time.monotonic(),
                serialized_size_bytes=None if payload is None else len(payload),
                serialized_sha256=payload_sha256,
                content_count=content_count,
            )
        except (AttributeError, TypeError, ValueError) as error:
            self._session.record_error(
                f"topic role {contract.role} message validation failed: {error}"
            )

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        """Forward only exact scenario-owned status names to the pure parser."""
        received = time.monotonic()
        matching = [
            status for status in message.status if status.name in self._status_names
        ]
        try:
            require_unique_status_names(status.name for status in matching)
        except DiagnosticContractError as error:
            self._session.record_error(str(error))
            return
        for status in matching:
            try:
                values = key_values_to_mapping(
                    (item.key, item.value) for item in status.values
                )
            except DiagnosticContractError as error:
                self._session.record_error(f"diagnostic {status.name}: {error}")
                continue
            self._session.record_diagnostic(
                status_name=status.name,
                level=_diagnostic_level_to_int(status.level),
                message=status.message,
                values=values,
                received_monotonic=received,
            )

    def _observe_graph_types(self) -> None:
        """Record exact advertised types instead of inferring them from callbacks."""
        for topic in self._session.contract.topics:
            infos = self.get_publishers_info_by_topic(topic.topic)
            self._session.record_advertised_types(
                topic.role,
                {str(info.topic_type) for info in infos},
            )

    def close(self) -> None:
        """Detach read-only subscriptions and timer before node destruction."""
        if self._graph_timer is not None:
            self.destroy_timer(self._graph_timer)
            self._graph_timer = None
        if self._diagnostics_subscription is not None:
            self.destroy_subscription(self._diagnostics_subscription)
            self._diagnostics_subscription = None
        for subscription in self._topic_subscriptions:
            self.destroy_subscription(subscription)
        self._topic_subscriptions.clear()


def _diagnostic_level_to_int(level: Any) -> int:
    """Convert the DiagnosticStatus ``byte`` wire level (bytes) to int."""
    if isinstance(level, (bytes, bytearray)):
        return int.from_bytes(level, "big")
    return int(level)


def _content_count(message: Any, field_name: str | None) -> int | None:
    if field_name is None:
        return None
    value = getattr(message, field_name, None)
    if value is None:
        return None
    try:
        return len(value)
    except TypeError as error:
        raise ValueError(f"field {field_name!r} is not a sequence") from error


def _require_finite_pose(message: Any) -> None:
    pose = message.pose
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("terminal PoseStamped contains non-finite values")
    quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
    if quaternion_norm <= 1.0e-9:
        raise ValueError("terminal PoseStamped has a zero quaternion")


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect bounded, read-only GPU perception runtime evidence."
    )
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--artifact-dir", required=True, type=Path)
    parser.add_argument("--run-token", required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument(
        "--asset",
        action="append",
        nargs=2,
        required=True,
        metavar=("NAME", "ABSOLUTE_PATH"),
        help="Explicit logical asset name and existing absolute file path.",
    )
    return parser.parse_args(remove_ros_args(argv)[1:])


def _asset_paths(values: list[list[str]]) -> dict[str, Path]:
    result = {}
    for name, path_text in values:
        if name in result:
            raise ContractError(f"duplicate --asset name: {name}")
        result[name] = Path(path_text)
    return result


def _require_absolute_cli_paths(
    contract_path: Path,
    artifact_directory: Path,
) -> None:
    """Reject working-directory-dependent contract and artifact locations."""
    if not contract_path.is_absolute():
        raise ContractError("--contract must be an absolute path")
    if not artifact_directory.is_absolute():
        raise ContractError("--artifact-dir must be an absolute path")


def _run(
    contract: GpuPerceptionContract,
    artifact_directory: Path,
    run_token: str,
    revision: str,
    asset_paths: dict[str, Path],
    ros_args: list[str],
) -> int:
    started_at = utc_now()
    provenance_errors = []
    gpu_identity = None
    asset_digests = ()
    try:
        gpu_identity = collect_gpu_identity(contract.gpu_identity)
    except ProvenanceError as error:
        provenance_errors.append(str(error))
    try:
        asset_digests = hash_assets(contract.assets, asset_paths)
    except ProvenanceError as error:
        provenance_errors.append(str(error))

    session = EvidenceSession(contract)
    node = None
    executor = None
    ros_initialized = False
    evaluation = None
    try:
        rclpy.init(args=ros_args)
        ros_initialized = True
        node = GpuPerceptionEvidenceProbeNode(session)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        started_monotonic = time.monotonic()
        session.begin_observation(started_monotonic)
        deadline = started_monotonic + contract.observation_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            evaluation = session.evaluate(now_monotonic=time.monotonic())
            if evaluation.evidence_status == "pass":
                break
        else:
            if not rclpy.ok():
                session.record_error("ROS context stopped before evidence completed")
        evaluation = session.evaluate(now_monotonic=time.monotonic())
    except (ExternalShutdownException, KeyboardInterrupt):
        if not session.observation_started:
            session.begin_observation(time.monotonic())
        session.record_error("evidence probe was interrupted")
        evaluation = session.evaluate(now_monotonic=time.monotonic())
    except Exception as error:  # noqa: B902 - persist bounded failure evidence
        if not session.observation_started:
            session.begin_observation(time.monotonic())
        session.record_error(f"ROS evidence adapter failed: {error}")
        evaluation = session.evaluate(now_monotonic=time.monotonic())
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.close()
            node.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()

    assert evaluation is not None

    artifact = build_artifact(
        contract=contract,
        run_token=run_token,
        revision=revision,
        started_at=started_at,
        finished_at=utc_now(),
        evaluation=evaluation,
        gpu_identity=gpu_identity,
        asset_digests=asset_digests,
        provenance_errors=provenance_errors,
    )
    path = write_artifact(artifact_directory, artifact)
    return _report_result(artifact.evidence_status, path)


def _report_result(evidence_status: str, artifact_path: Path) -> int:
    """Print a stable post-write marker and map artifact status to a process code."""
    if evidence_status not in {"pass", "fail"}:
        raise ValueError(f"unsupported evidence status: {evidence_status!r}")
    marker = (
        "GPU_PERCEPTION_PASS"
        if evidence_status == "pass"
        else "GPU_PERCEPTION_FAIL"
    )
    print(f"{marker} artifact={artifact_path}", flush=True)
    return 0 if evidence_status == "pass" else 1


def main() -> None:
    """Parse explicit inputs, run one bounded session, and return stable status."""
    argv = list(sys.argv)
    arguments = _arguments(argv)
    try:
        _require_absolute_cli_paths(arguments.contract, arguments.artifact_dir)
        validate_evidence_identity(arguments.run_token, arguments.revision)
        contract = load_contract(arguments.contract)
        asset_paths = _asset_paths(arguments.asset)
    except (ContractError, ValueError) as error:
        raise SystemExit(f"invalid GPU perception evidence input: {error}") from error
    raise SystemExit(
        _run(
            contract,
            arguments.artifact_dir,
            arguments.run_token,
            arguments.revision,
            asset_paths,
            argv,
        )
    )


if __name__ == "__main__":
    main()
