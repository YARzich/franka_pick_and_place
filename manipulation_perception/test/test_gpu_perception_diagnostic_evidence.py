"""
Test machine-readable perception diagnostics used by GPU runtime evidence.

Responsibility: Pin exact keys, strict parsing, counter deltas, and failure gates.
"""

from copy import deepcopy

from gpu_perception_evidence_test_support import (
    contract_value,
    diagnostic_values,
    passing_session,
)
import pytest

from manipulation_perception.common.pipeline_diagnostic_contract import (
    PIPELINE_DIAGNOSTIC_KEYS,
)
from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.diagnostics import (
    calculate_pipeline_delta,
    DiagnosticContractError,
    key_values_to_mapping,
    parse_pipeline_snapshot,
    require_unique_status_names,
)


def test_parser_accepts_exact_contract_and_ignores_forward_compatible_extra_keys() -> None:
    values = diagnostic_values(input_count=12, processed_count=8, output_count=4)
    values["future_metric"] = "supported"

    snapshot = parse_pipeline_snapshot(values)

    assert snapshot.input_count == 12
    assert snapshot.processing_latency_p95_sec == pytest.approx(0.02)
    assert snapshot.last_success_source is not None
    assert snapshot.last_success_source.frame_id == "camera_optical_frame"


def test_parser_rejects_duplicate_missing_and_malformed_values() -> None:
    with pytest.raises(DiagnosticContractError, match="duplicate"):
        key_values_to_mapping([("input_count", "1"), ("input_count", "2")])

    missing = diagnostic_values(input_count=1, processed_count=1, output_count=1)
    del missing[PIPELINE_DIAGNOSTIC_KEYS.worker_timeout_count]
    with pytest.raises(DiagnosticContractError, match="worker_timeout_count"):
        parse_pipeline_snapshot(missing)

    malformed = diagnostic_values(input_count=1, processed_count=1, output_count=1)
    malformed[PIPELINE_DIAGNOSTIC_KEYS.input_count] = "01"
    with pytest.raises(DiagnosticContractError, match="canonical"):
        parse_pipeline_snapshot(malformed)

    with pytest.raises(DiagnosticContractError, match="duplicate DiagnosticStatus"):
        require_unique_status_names(["backend: task", "backend: task"])


def test_delta_uses_processed_count_as_new_latency_observation_count() -> None:
    baseline = parse_pipeline_snapshot(
        diagnostic_values(
            input_count=100,
            processed_count=100,
            output_count=20,
            latency_sample_count=120,
        )
    )
    latest = parse_pipeline_snapshot(
        diagnostic_values(
            input_count=105,
            processed_count=104,
            output_count=21,
            latency_sample_count=120,
        )
    )

    delta = calculate_pipeline_delta(baseline, latest)

    assert delta.latency_observation_count == 4
    assert delta.input_count == 5
    assert latest.latency_sample_count - baseline.latency_sample_count == 0


@pytest.mark.parametrize(
    ("latest_level", "value_overrides", "error_text"),
    [
        (2, {}, "ERROR/STALE"),
        (0, {"worker_timeout_count": 1}, "worker timeouts"),
        (0, {"worker_restart_count": 1}, "worker restarts"),
        (0, {"error_count": 1}, "processing errors"),
    ],
)
def test_session_rejects_error_timeout_restart_and_new_processing_error(
    latest_level: int,
    value_overrides: dict[str, int],
    error_text: str,
) -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())
    session = passing_session(contract)
    diagnostic = contract.diagnostics[0]
    session.record_diagnostic(
        status_name=diagnostic.status_name,
        level=latest_level,
        message="latest",
        values=diagnostic_values(
            input_count=14,
            processed_count=14,
            output_count=7,
            **value_overrides,
        ),
        received_monotonic=103.05,
    )

    evaluation = session.evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "fail"
    assert any(error_text in error for error in evaluation.errors)


def test_session_requires_new_input_output_and_latency_observations() -> None:
    value = deepcopy(contract_value())
    value["minimum_input_delta"] = 3
    value["minimum_output_delta"] = 2
    value["minimum_latency_samples"] = 3
    session = passing_session(GpuPerceptionContract.from_mapping(value))

    evaluation = session.evaluate(now_monotonic=103.1)

    assert evaluation.evidence_status == "fail"
    assert any("input delta" in error for error in evaluation.errors)
    assert any("output delta" in error for error in evaluation.errors)
    assert any("latency observations" in error for error in evaluation.errors)


def test_cumulative_counter_regression_is_rejected() -> None:
    baseline = parse_pipeline_snapshot(
        diagnostic_values(input_count=10, processed_count=10, output_count=5)
    )
    latest = parse_pipeline_snapshot(
        diagnostic_values(input_count=9, processed_count=10, output_count=5)
    )

    with pytest.raises(DiagnosticContractError, match="moved backwards"):
        calculate_pipeline_delta(baseline, latest)
