"""
Test loading and normalization of GigaPose coarse predictions.

Responsibility: Verify hypothesis limits, batched arrays, translation scale, and missing output.
"""

from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_gigapose_worker_outputs import (  # noqa: E402
    load_gigapose_prediction,
)


def test_load_prediction_limits_hypotheses_and_scales_translation(tmp_path: Path):
    prediction_path = tmp_path / 'prediction.npz'
    poses = np.repeat(np.eye(4, dtype=np.float32)[None, ...], 3, axis=0)
    poses[:, 0, 3] = [1.0, 2.0, 3.0]
    scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)
    np.savez(prediction_path, poses=poses, scores=scores)

    result = load_gigapose_prediction(
        prediction_path,
        num_hypotheses=2,
        translation_scale=0.001,
    )

    assert len(result['poses']) == 2
    assert result['poses'][0][0][3] == pytest.approx(0.001)
    assert result['poses'][1][0][3] == pytest.approx(0.002)
    assert result['scores'] == pytest.approx([0.9, 0.8])


def test_load_prediction_accepts_batched_prediction_arrays(tmp_path: Path):
    prediction_path = tmp_path / 'prediction.npz'
    poses = np.repeat(np.eye(4, dtype=np.float32)[None, None, ...], 2, axis=1)
    scores = np.array([[0.6, 0.5]], dtype=np.float32)
    np.savez(prediction_path, poses=poses, scores=scores)

    result = load_gigapose_prediction(prediction_path, num_hypotheses=5, translation_scale=1.0)

    assert len(result['poses']) == 2
    assert result['scores'] == pytest.approx([0.6, 0.5])


def test_load_prediction_reports_missing_file(tmp_path: Path):
    with pytest.raises(FileNotFoundError, match='prediction file'):
        load_gigapose_prediction(tmp_path / 'missing.npz', num_hypotheses=1, translation_scale=1.0)
