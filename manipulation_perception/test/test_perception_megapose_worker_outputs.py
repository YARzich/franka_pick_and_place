"""
Test serialization and persistence of MegaPose predictions.

Responsibility: Preserve labels and pose payloads and write valid empty or populated output JSON.
"""

import json
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_megapose_worker_outputs import (  # noqa: E402
    serialize_and_write_megapose_predictions,
    serialize_megapose_predictions,
    write_megapose_object_data,
)


class FakePoseTensor:
    def __init__(self, poses) -> None:
        self._poses = poses

    def cpu(self):
        return self

    def numpy(self):
        return self._poses


class FakePoseEstimates:
    def __init__(self, labels, poses) -> None:
        self.infos = {'label': labels}
        self.poses = FakePoseTensor(poses)


class FakeTransform:
    def __init__(self, pose) -> None:
        self.pose = pose


class FakeObjectData:
    def __init__(self, *, label, TWO) -> None:
        self.label = label
        self.TWO = TWO

    def to_json(self):
        return {'label': self.label, 'TWO': self.TWO.pose}


def test_serialize_predictions_keeps_labels_and_pose_payloads():
    pose_estimates = FakePoseEstimates(['cube_a', 'cube_b'], ['pose-a', 'pose-b'])

    object_data = serialize_megapose_predictions(
        pose_estimates,
        object_data_type=FakeObjectData,
        transform_type=FakeTransform,
    )

    assert object_data == [
        {'label': 'cube_a', 'TWO': 'pose-a'},
        {'label': 'cube_b', 'TWO': 'pose-b'},
    ]


def test_write_object_data_creates_outputs_json(tmp_path: Path):
    object_data = [{'label': 'cube_5cm', 'TWO': [1, 2, 3]}]

    write_megapose_object_data(tmp_path / 'example', object_data)

    output_path = tmp_path / 'example' / 'outputs' / 'object_data.json'
    assert json.loads(output_path.read_text()) == object_data


def test_serialize_and_write_predictions_handles_empty_results(tmp_path: Path):
    pose_estimates = FakePoseEstimates([], [])

    object_data = serialize_and_write_megapose_predictions(
        tmp_path / 'example',
        pose_estimates,
        object_data_type=FakeObjectData,
        transform_type=FakeTransform,
    )

    assert object_data == []
    output_path = tmp_path / 'example' / 'outputs' / 'object_data.json'
    assert json.loads(output_path.read_text()) == []
