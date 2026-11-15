"""
Test pose selection and fallback loading from estimator output files.

Responsibility: Prefer matching object labels and preserve deterministic candidate order.
"""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_estimation_io import (  # noqa: E402,I100,I101,I201
    load_pose_from_output_files,
    select_pose_from_object_data,
)


def test_select_pose_from_object_data_prefers_matching_label_with_two():
    first = {'label': 'other', 'TWO': [[0, 0, 0, 1], [0, 0, 0]]}
    second = {'label': 'cube_5cm', 'TWO': [[0, 0, 0, 1], [1, 2, 3]]}

    assert select_pose_from_object_data([first, second], 'cube_5cm') is second


def test_select_pose_from_object_data_keeps_existing_first_pose_fallback():
    first = {'label': 'other', 'TWO': [[0, 0, 0, 1], [0, 0, 0]]}

    assert select_pose_from_object_data([first], 'cube_5cm') is first
    assert select_pose_from_object_data([{'label': 'cube_5cm'}], 'cube_5cm') is None
    assert select_pose_from_object_data([], 'cube_5cm') is None


def test_load_pose_from_output_files_checks_candidates_in_order(tmp_path: Path):
    missing = tmp_path / 'missing.json'
    empty = tmp_path / 'empty.json'
    output = tmp_path / 'object_data.json'
    empty.write_text('[]')
    output.write_text('[{"label": "cube_5cm", "TWO": [[0, 0, 0, 1], [1, 2, 3]]}]')

    pose = load_pose_from_output_files([missing, empty, output], 'cube_5cm')

    assert pose == {'label': 'cube_5cm', 'TWO': [[0, 0, 0, 1], [1, 2, 3]]}
