"""
Fixtures for pure GPU perception evidence tests.

Responsibility: Build scenario-neutral contracts, diagnostics, and passing sessions.
"""

from __future__ import annotations

from manipulation_perception.common.pipeline_diagnostic_contract import (
    PIPELINE_DIAGNOSTIC_KEYS,
)
from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.evidence import EvidenceSession


def contract_value() -> dict[str, object]:
    """Return a minimal complete contract with optical-to-world frame transition."""
    return {
        "schema_version": 1,
        "contract_id": "test_gpu_perception_v1",
        "observation_timeout_sec": 10.0,
        "minimum_observation_sec": 2.0,
        "terminal_freshness_sec": 2.0,
        "diagnostic_freshness_sec": 2.0,
        "minimum_input_delta": 1,
        "minimum_output_delta": 1,
        "minimum_latency_samples": 1,
        "topics": [
            {
                "role": "image",
                "topic": "/camera/image",
                "message_type": "sensor_msgs/msg/Image",
                "frame_id": "camera_optical_frame",
                "nonempty_field": None,
                "capture_payload_digest": False,
                "maximum_rate_hz": 30.0,
            },
            {
                "role": "backend_detection",
                "topic": "/pose_backend/output",
                "message_type": "vision_msgs/msg/Detection3DArray",
                "frame_id": "camera_optical_frame",
                "nonempty_field": "detections",
                "capture_payload_digest": True,
                "maximum_rate_hz": 10.0,
            },
            {
                "role": "terminal_pose",
                "topic": "/object_pose",
                "message_type": "geometry_msgs/msg/PoseStamped",
                "frame_id": "world",
                "nonempty_field": None,
                "capture_payload_digest": True,
                "maximum_rate_hz": 10.0,
            },
        ],
        "correlation_groups": [
            {
                "name": "end_to_end_pose",
                "roles": ["image", "backend_detection", "terminal_pose"],
            }
        ],
        "terminal_pose_role": "terminal_pose",
        "diagnostics": [
            {
                "role": "stereo",
                "status_name": "stereo: perception/stereo_depth_pointcloud",
                "source_topic_role": "image",
            },
            {
                "role": "detector",
                "status_name": "detector: perception/yolo_detection",
                "source_topic_role": "image",
            },
            {
                "role": "pose_backend",
                "status_name": "backend: perception/megapose_detection",
                "source_topic_role": "backend_detection",
            },
            {
                "role": "pose_adapter",
                "status_name": "adapter: perception/pose_estimation",
                "source_topic_role": "terminal_pose",
            },
        ],
        "assets": [{"name": "engine"}, {"name": "model"}],
        "gpu_identity": {
            "argv": [
                "/usr/bin/nvidia-smi",
                "--query-gpu=name,uuid,driver_version,memory.total",
                "--format=csv,noheader,nounits",
            ],
            "fields": ["name", "uuid", "driver_version", "memory_total"],
            "timeout_sec": 5.0,
        },
    }


def diagnostic_values(
    *,
    input_count: int,
    processed_count: int,
    output_count: int,
    error_count: int = 0,
    drop_count: int = 0,
    worker_timeout_count: int = 0,
    worker_restart_count: int = 0,
    latency_sample_count: int = 10,
    last_success_age_sec: float = 0.1,
    last_success_source: str = "5.000000001@camera_optical_frame",
) -> dict[str, str]:
    """Return all 15 machine keys using producer-compatible wire formats."""
    keys = PIPELINE_DIAGNOSTIC_KEYS
    ratio = 0.0 if input_count == 0 else drop_count / input_count
    return {
        keys.input_count: str(input_count),
        keys.processed_count: str(processed_count),
        keys.output_count: str(output_count),
        keys.error_count: str(error_count),
        keys.drop_or_skip_count: str(drop_count),
        keys.drop_or_skip_ratio: f"{ratio:.6f}",
        keys.worker_timeout_count: str(worker_timeout_count),
        keys.worker_restart_count: str(worker_restart_count),
        keys.latency_sample_count: str(latency_sample_count),
        keys.processing_latency_p50_sec: "0.010000",
        keys.processing_latency_p95_sec: "0.020000",
        keys.processing_latency_max_sec: "0.030000",
        keys.last_success_age_sec: f"{last_success_age_sec:.6f}",
        keys.last_success_source: last_success_source,
        keys.steady_uptime_sec: "20.000",
    }


def passing_session(
    contract: GpuPerceptionContract | None = None,
) -> EvidenceSession:
    """Populate one session that passes at steady time 103.1 seconds."""
    policy = contract or GpuPerceptionContract.from_mapping(contract_value())
    session = EvidenceSession(policy, started_monotonic=100.0)
    for topic in policy.topics:
        session.record_advertised_types(topic.role, {topic.message_type})
        session.record_topic_sample(
            role=topic.role,
            message_type=topic.message_type,
            frame_id=topic.frame_id,
            stamp_sec=5,
            stamp_nanosec=1,
            received_monotonic=102.8,
            serialized_size_bytes=16 if topic.capture_payload_digest else None,
            serialized_sha256="a" * 64 if topic.capture_payload_digest else None,
            content_count=1 if topic.nonempty_field is not None else None,
        )
    for diagnostic in policy.diagnostics:
        source_topic = next(
            topic
            for topic in policy.topics
            if topic.role == diagnostic.source_topic_role
        )
        source_label = f"5.000000001@{source_topic.frame_id}"
        session.record_diagnostic(
            status_name=diagnostic.status_name,
            level=0,
            message="pipeline observed",
            values=diagnostic_values(
                input_count=10,
                processed_count=10,
                output_count=5,
                last_success_source=source_label,
            ),
            received_monotonic=100.5,
        )
        session.record_diagnostic(
            status_name=diagnostic.status_name,
            level=0,
            message="pipeline observed",
            values=diagnostic_values(
                input_count=12,
                processed_count=12,
                output_count=6,
                last_success_source=source_label,
            ),
            received_monotonic=103.0,
        )
    return session


__all__ = ["contract_value", "diagnostic_values", "passing_session"]
