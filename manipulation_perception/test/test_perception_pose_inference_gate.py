"""
Test scheduling and mutual exclusion of pose inference.

Responsibility: Enforce frame periods, minimum intervals, non-overlap, and explicit completion.
"""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_inference_gate import (  # noqa: E402,I100,I101,I201
    PoseInferenceGate,
)


def test_gate_skips_frames_by_configured_period():
    gate = PoseInferenceGate(run_every_n_frames=3, min_interval_sec=0.0)

    assert gate.accepts_frame() is False
    assert gate.accepts_frame() is False
    assert gate.accepts_frame() is True
    assert gate.accepts_frame() is False


def test_gate_prevents_overlap_and_respects_min_interval():
    gate = PoseInferenceGate(run_every_n_frames=1, min_interval_sec=2.0)

    assert gate.is_running() is False
    assert gate.try_start(10.0) is True
    assert gate.is_running() is True
    assert gate.try_start(12.5) is False

    gate.mark_done()
    assert gate.is_running() is False

    assert gate.try_start(11.0) is False
    assert gate.try_start(12.0) is True


def test_gate_can_restart_after_mark_done_when_interval_elapsed():
    gate = PoseInferenceGate(run_every_n_frames=1, min_interval_sec=0.5)

    assert gate.try_start(1.0) is True
    gate.mark_done()

    assert gate.try_start(1.6) is True
