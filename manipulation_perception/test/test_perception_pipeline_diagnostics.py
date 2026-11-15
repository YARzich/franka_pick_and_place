"""
Test perception diagnostics parameters, wiring, and emitted names.

Responsibility: Verify typed thresholds, fail-closed startup, task registration, and snapshots.
"""

import time
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pytest
import rclpy

from manipulation_perception.common.pipeline_diagnostic_contract import (
    PIPELINE_DIAGNOSTIC_KEYS,
)
from manipulation_perception.common.pipeline_diagnostics import PipelineDiagnostics
from manipulation_perception.common.pipeline_diagnostics_config import (
    declare_pipeline_diagnostics_parameters,
    PipelineDiagnosticsConfig,
    read_pipeline_diagnostics_config,
)
from manipulation_perception.common.pipeline_metrics import PipelineMetrics


class _FakeNode:
    def __init__(self, overrides=None) -> None:
        self._values = dict(overrides or {})

    def declare_parameter(self, name, default):
        self._values.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self._values[name])


class _FakeTimer:
    def __init__(self) -> None:
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class _FakeUpdater:
    def __init__(self, node, period) -> None:
        self.node = node
        self.period = period
        self.timer = _FakeTimer()
        self.hardware_id = None
        self.tasks = []

    def setHardwareID(self, hardware_id) -> None:  # noqa: N802 - upstream API
        self.hardware_id = hardware_id

    def add(self, name, callback) -> None:
        self.tasks.append((name, callback))


class _FakeStatus:
    def __init__(self) -> None:
        self.level = None
        self.message = None
        self.values = {}

    def summary(self, level, message) -> None:
        self.level = level
        self.message = message

    def add(self, key, value) -> None:
        self.values[key] = value


def _config(**overrides) -> PipelineDiagnosticsConfig:
    values = {
        "enabled": True,
        "require_success": False,
        "window_size": 10,
        "update_period_sec": 1.0,
        "minimum_samples": 2,
        "warn_processing_latency_p95_sec": 0.0,
        "error_processing_latency_p95_sec": 0.0,
        "warn_last_success_age_sec": 0.0,
        "error_last_success_age_sec": 0.0,
        "warn_drop_or_skip_ratio": 0.0,
        "error_drop_or_skip_ratio": 0.0,
        "warn_error_count": 0,
        "error_error_count": 0,
    }
    values.update(overrides)
    return PipelineDiagnosticsConfig(**values)


def test_diagnostics_parameters_are_typed() -> None:
    node = _FakeNode(
        {
            "diagnostics.enabled": False,
            "diagnostics.window_size": 32,
            "diagnostics.update_period_sec": 0.5,
            "diagnostics.warn_drop_or_skip_ratio": 0.2,
            "diagnostics.error_drop_or_skip_ratio": 0.4,
        }
    )
    declare_pipeline_diagnostics_parameters(node)

    config = read_pipeline_diagnostics_config(node)

    assert config.enabled is False
    assert config.window_size == 32
    assert config.update_period_sec == 0.5
    assert config.warn_drop_or_skip_ratio == 0.2
    assert config.error_drop_or_skip_ratio == 0.4


def test_error_threshold_lower_than_warn_fails_fast() -> None:
    node = _FakeNode(
        {
            "diagnostics.warn_drop_or_skip_ratio": 0.4,
            "diagnostics.error_drop_or_skip_ratio": 0.2,
        }
    )
    declare_pipeline_diagnostics_parameters(node)

    with pytest.raises(ValueError, match="ERROR threshold"):
        read_pipeline_diagnostics_config(node)


def test_required_success_fails_closed_before_first_input() -> None:
    metrics = PipelineMetrics()
    created = []

    def updater_factory(node, period):
        updater = _FakeUpdater(node, period)
        created.append(updater)
        return updater

    adapter = PipelineDiagnostics(
        node=object(),
        pipeline_name="required_pipeline",
        metrics=metrics,
        config=_config(require_success=True),
        updater_factory=updater_factory,
    )

    status = created[0].tasks[0][1](_FakeStatus())
    assert status.level == DiagnosticStatus.WARN
    assert status.message == "waiting for first successful output"

    adapter.close()


def test_adapter_registers_standard_task_and_publishes_snapshot() -> None:
    metrics = PipelineMetrics()
    metrics.record_input(2)
    metrics.record_drop_or_skip()
    created = []

    def updater_factory(node, period):
        updater = _FakeUpdater(node, period)
        created.append(updater)
        return updater

    adapter = PipelineDiagnostics(
        node=object(),
        pipeline_name="test_pipeline",
        metrics=metrics,
        config=_config(warn_drop_or_skip_ratio=0.4),
        updater_factory=updater_factory,
    )

    updater = created[0]
    assert updater.period == 1.0
    assert updater.hardware_id == "none"
    assert updater.tasks[0][0] == "perception/test_pipeline"
    status = updater.tasks[0][1](_FakeStatus())
    assert status.level == DiagnosticStatus.WARN
    assert "drop/skip ratio" in status.message
    assert status.values["input_count"] == "2"
    assert status.values["last_success_source"] == "n/a"
    assert tuple(status.values) == tuple(PIPELINE_DIAGNOSTIC_KEYS)

    adapter.close()
    assert updater.timer.cancelled is True


def test_real_updater_prefixes_task_with_node_name() -> None:
    """Pin the Jazzy wire-name contract consumed by the health adapter."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    source_node = None
    observer_node = None
    subscription = None
    adapter = None
    executor = None
    try:
        source_node = rclpy.create_node(
            "diagnostic_name_contract", context=context
        )
        observer_node = rclpy.create_node(
            "diagnostic_name_contract_observer", context=context
        )
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(source_node)
        executor.add_node(observer_node)
        received = []
        subscription = observer_node.create_subscription(
            DiagnosticArray, "/diagnostics", received.append, 10
        )
        adapter = PipelineDiagnostics(
            node=source_node,
            pipeline_name="name_contract",
            metrics=PipelineMetrics(),
            config=_config(update_period_sec=0.1),
        )

        expected = "diagnostic_name_contract: perception/name_contract"
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if any(
                status.name == expected
                for message in received
                for status in message.status
            ):
                break
        else:
            pytest.fail(f"diagnostic_updater did not publish {expected!r}")
    finally:
        if adapter is not None:
            adapter.close()
        if executor is not None:
            if observer_node is not None:
                executor.remove_node(observer_node)
            if source_node is not None:
                executor.remove_node(source_node)
            executor.shutdown(timeout_sec=1.0)
        if observer_node is not None:
            if subscription is not None:
                observer_node.destroy_subscription(subscription)
            observer_node.destroy_node()
        if source_node is not None:
            source_node.destroy_node()
        rclpy.shutdown(context=context)


def test_disabled_adapter_has_no_updater_side_effect() -> None:
    called = []
    adapter = PipelineDiagnostics(
        node=object(),
        pipeline_name="disabled",
        metrics=PipelineMetrics(),
        config=_config(enabled=False),
        updater_factory=lambda *_args, **_kwargs: called.append(True),
    )

    adapter.close()
    assert called == []
