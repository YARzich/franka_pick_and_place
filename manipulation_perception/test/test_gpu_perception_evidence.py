"""
Test exact stamped topic correlation and freshness for GPU perception evidence.

Responsibility: Verify type/frame/stamp joins, nonempty outputs, and terminal pose age.
"""

from gpu_perception_evidence_test_support import (
    contract_value,
    diagnostic_values,
    passing_session,
)
import pytest

from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.error_log import DEFAULT_ERROR_CAPACITY
from manipulation_perception.validation.evidence import EvidenceSession


def _record_diagnostics(session: EvidenceSession, *, stamp_sec: int = 5) -> None:
    for item in session.contract.diagnostics:
        source_topic = next(
            topic
            for topic in session.contract.topics
            if topic.role == item.source_topic_role
        )
        source_label = f"{stamp_sec}.000000001@{source_topic.frame_id}"
        session.record_diagnostic(
            status_name=item.status_name,
            level=0,
            message="baseline",
            values=diagnostic_values(
                input_count=10,
                processed_count=10,
                output_count=5,
                last_success_source=source_label,
            ),
            received_monotonic=100.5,
        )
        session.record_diagnostic(
            status_name=item.status_name,
            level=0,
            message="latest",
            values=diagnostic_values(
                input_count=12,
                processed_count=12,
                output_count=6,
                last_success_source=source_label,
            ),
            received_monotonic=103.0,
        )


def _record_sample(
    session: EvidenceSession,
    role: str,
    *,
    stamp_sec: int = 5,
    frame_id: str | None = None,
    message_type: str | None = None,
    content_count: int | None = None,
) -> None:
    topic = next(item for item in session.contract.topics if item.role == role)
    session.record_topic_sample(
        role=role,
        message_type=message_type or topic.message_type,
        frame_id=frame_id or topic.frame_id,
        stamp_sec=stamp_sec,
        stamp_nanosec=1,
        received_monotonic=102.8,
        serialized_size_bytes=16 if topic.capture_payload_digest else None,
        serialized_sha256="a" * 64 if topic.capture_payload_digest else None,
        content_count=(
            content_count
            if content_count is not None
            else (1 if topic.nonempty_field is not None else None)
        ),
    )


def test_observation_clock_starts_explicitly_after_adapter_setup() -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())
    session = EvidenceSession(contract)
    for topic in contract.topics:
        session.record_advertised_types(topic.role, {topic.message_type})

    with pytest.raises(RuntimeError, match="has not started"):
        session.evaluate(now_monotonic=100.0)

    session.begin_observation(100.0)

    assert session.elapsed(100.25) == pytest.approx(0.25)
    with pytest.raises(RuntimeError, match="already started"):
        session.begin_observation(101.0)


def test_passing_evidence_keeps_exact_stamp_across_optical_and_world_frames() -> None:
    evaluation = passing_session().evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "pass"
    assert evaluation.errors == ()
    assert evaluation.terminal_pose is not None
    assert evaluation.terminal_pose.frame_id == "world"
    sample = evaluation.correlated_samples[0]
    assert {(topic.stamp_sec, topic.stamp_nanosec) for topic in sample.topics} == {(5, 1)}
    assert {topic.frame_id for topic in sample.topics} == {
        "camera_optical_frame",
        "world",
    }


def test_different_stamps_do_not_form_end_to_end_sample() -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())
    session = EvidenceSession(contract, started_monotonic=100.0)
    for topic in contract.topics:
        session.record_advertised_types(topic.role, {topic.message_type})
    _record_sample(session, "image", stamp_sec=5)
    _record_sample(session, "backend_detection", stamp_sec=5)
    _record_sample(session, "terminal_pose", stamp_sec=6)
    _record_diagnostics(session)

    evaluation = session.evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "fail"
    assert any("no exact common ROS stamp" in error for error in evaluation.errors)
    assert evaluation.terminal_pose is None


def test_wrong_type_frame_and_graph_advertisement_fail_closed() -> None:
    session = passing_session()
    session.record_advertised_types("image", {"sensor_msgs/msg/CompressedImage"})
    _record_sample(session, "image", stamp_sec=6, frame_id="wrong_frame")
    _record_sample(
        session,
        "terminal_pose",
        stamp_sec=6,
        message_type="geometry_msgs/msg/TransformStamped",
    )

    evaluation = session.evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "fail"
    assert any("advertised types" in error for error in evaluation.errors)
    assert any("received frame" in error for error in evaluation.errors)
    assert any("received type" in error for error in evaluation.errors)


def test_empty_detection_and_missing_required_digest_are_rejected() -> None:
    session = passing_session()
    _record_sample(session, "backend_detection", stamp_sec=6, content_count=0)
    session.record_topic_sample(
        role="terminal_pose",
        message_type="geometry_msgs/msg/PoseStamped",
        frame_id="world",
        stamp_sec=6,
        stamp_nanosec=1,
        received_monotonic=102.9,
        serialized_size_bytes=None,
        serialized_sha256=None,
        content_count=None,
    )

    evaluation = session.evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "fail"
    assert any("empty field" in error for error in evaluation.errors)
    assert any("no serialized payload" in error for error in evaluation.errors)


def test_high_rate_topic_can_skip_payload_digest_without_losing_header_proof() -> None:
    evaluation = passing_session().evaluate(now_monotonic=103.1)
    image = next(
        topic
        for topic in evaluation.correlated_samples[0].topics
        if topic.role == "image"
    )

    assert image.serialized_size_bytes is None
    assert image.serialized_sha256 is None


def test_terminal_pose_and_diagnostics_must_remain_fresh() -> None:
    evaluation = passing_session().evaluate(now_monotonic=106.0)

    assert evaluation.evidence_status == "fail"
    assert any("terminal pose age" in error for error in evaluation.errors)
    assert any("diagnostic role stereo age" in error for error in evaluation.errors)


def test_rate_derived_retention_keeps_delayed_backend_source_beyond_512_samples() -> None:
    value = contract_value()
    value["topics"][0]["maximum_rate_hz"] = 100.0
    contract = GpuPerceptionContract.from_mapping(value)
    session = EvidenceSession(contract, started_monotonic=100.0)
    for topic in contract.topics:
        session.record_advertised_types(topic.role, {topic.message_type})
    _record_sample(session, "image", stamp_sec=1)
    image = next(topic for topic in contract.topics if topic.role == "image")
    for stamp_sec in range(2, 602):
        session.record_topic_sample(
            role="image",
            message_type=image.message_type,
            frame_id=image.frame_id,
            stamp_sec=stamp_sec,
            stamp_nanosec=1,
            received_monotonic=102.0 + stamp_sec * 0.001,
            serialized_size_bytes=None,
            serialized_sha256=None,
            content_count=None,
        )
    _record_sample(session, "backend_detection", stamp_sec=1)
    _record_sample(session, "terminal_pose", stamp_sec=1)
    _record_diagnostics(session, stamp_sec=1)

    evaluation = session.evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "pass"
    assert evaluation.correlated_samples[0].stamp_sec == 1


def test_diagnostic_source_must_match_an_observed_stamped_topic() -> None:
    session = passing_session()
    diagnostic = session.contract.diagnostics[0]
    session.record_diagnostic(
        status_name=diagnostic.status_name,
        level=0,
        message="newer unrelated source",
        values=diagnostic_values(
            input_count=13,
            processed_count=13,
            output_count=7,
            last_success_source="999.000000001@camera_optical_frame",
        ),
        received_monotonic=103.05,
    )

    evaluation = session.evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "fail"
    assert any("does not match observed topic" in error for error in evaluation.errors)
    stereo = next(item for item in evaluation.diagnostics if item.role == "stereo")
    assert stereo.latest_source_sample is None


def test_adapter_errors_are_deduplicated_and_bounded() -> None:
    session = passing_session()
    for index in range(DEFAULT_ERROR_CAPACITY + 20):
        session.record_error(f"failure {index}")
    session.record_error("failure 0")

    evaluation = session.evaluate(now_monotonic=103.1)

    assert len(evaluation.errors) == DEFAULT_ERROR_CAPACITY + 1
    assert evaluation.errors[-1] == "20 additional errors omitted"
