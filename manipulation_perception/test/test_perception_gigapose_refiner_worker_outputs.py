"""
Test serialization of GigaPose refiner predictions.

Responsibility: Preserve label, pose, and score fields while rejecting empty estimates.
"""

from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_gigapose_refiner_worker_outputs import (  # noqa: E402
    serialize_gigapose_refiner_prediction,
)


class FakePoseTensor:
    def __init__(self, pose) -> None:
        self._pose = np.asarray(pose)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._pose


class FakePoses:
    def __init__(self, pose) -> None:
        self.pose = pose

    def __getitem__(self, index):
        """Return the fake first pose tensor."""
        assert index == 0
        return FakePoseTensor(self.pose)


class FakeScoreSeries:
    def __init__(self, value) -> None:
        self.iloc = [value]


class FakeInfos(dict):
    def __init__(self, score=None) -> None:
        super().__init__()
        if score is not None:
            self['pose_score'] = True
            self.pose_score = FakeScoreSeries(score)


class FakePoseEstimates:
    def __init__(self, pose, score=None, length=1) -> None:
        self.poses = FakePoses(pose)
        self.infos = FakeInfos(score)
        self._length = length

    def __len__(self):
        """Return the configured fake collection length."""
        return self._length


class FakeTransform:
    def __init__(self, pose) -> None:
        self.pose = pose.tolist()


class FakeObjectData:
    def __init__(self, *, label, TWO) -> None:
        self.label = label
        self.TWO = TWO

    def to_json(self):
        return {'label': self.label, 'TWO': self.TWO.pose}


def test_serialize_refiner_prediction_keeps_label_pose_and_score():
    pose = np.eye(4)

    object_data = serialize_gigapose_refiner_prediction(
        FakePoseEstimates(pose, score=0.42),
        output_label='cube_5cm',
        object_data_type=FakeObjectData,
        transform_type=FakeTransform,
    )

    assert object_data['label'] == 'cube_5cm'
    assert object_data['TWO'] == pose.tolist()
    assert object_data['score'] == 0.42


def test_serialize_refiner_prediction_rejects_empty_estimates():
    with pytest.raises(ValueError, match='не вернул pose'):
        serialize_gigapose_refiner_prediction(
            FakePoseEstimates(np.eye(4), length=0),
            output_label='cube_5cm',
            object_data_type=FakeObjectData,
            transform_type=FakeTransform,
        )
