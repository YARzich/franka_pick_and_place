"""
CPU-only tests for the private PlanningScene conversion contract.

Responsibility: Verify fail-closed world, attachment, ACM, and geometry mapping.
"""

from copy import deepcopy
import math

import pytest

from manipulation_curobo_runtime.scene_conversion import (
    CONE_SEGMENTS,
    SceneContractError,
    UnsupportedSceneError,
    conservative_cone_mesh,
    globally_allowed_world_objects,
    load_scene_snapshot,
)


IDENTITY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def _part(name, owner, kind, **values):
    return {
        'name': name,
        'owner_id': owner,
        'kind': kind,
        'pose': IDENTITY,
        **values,
    }


def _request():
    compound_parts = [
        _part('compound__primitive_000', 'compound', 'cuboid', dims=[1, 2, 3]),
        _part('compound__primitive_001', 'compound', 'sphere', radius=0.2),
        _part(
            'compound__primitive_002',
            'compound',
            'cylinder',
            radius=0.1,
            height=0.4,
        ),
        _part(
            'compound__primitive_003',
            'compound',
            'cone',
            radius=0.3,
            height=0.7,
        ),
        _part(
            'compound__mesh_000',
            'compound',
            'mesh',
            vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
            faces=[[0, 1, 2]],
        ),
    ]
    return {
        'schema_version': 2,
        'workspace_frame': 'base_link',
        'scene': {
            'is_diff': False,
            'planning_frame': 'base_link',
            'robot_state': {
                'is_diff': False,
                'joint_names': ['joint_a'],
                'joint_positions': [0.1],
            },
            'frame_transforms': [
                {
                    'parent_frame': 'base_link',
                    'child_frame': 'base_link',
                    'pose': IDENTITY,
                },
                {
                    'parent_frame': 'base_link',
                    'child_frame': 'tool0',
                    'pose': [0.4, 0, 0.5, 0, 0, 0, 1],
                },
            ],
            'world_objects': [
                {
                    'id': 'compound',
                    'source_frame': 'base_link',
                    'parts': compound_parts,
                }
            ],
            'attached_objects': [
                {
                    'link_name': 'tool0',
                    'touch_links': ['left_finger', 'right_finger'],
                    'object': {
                        'id': 'payload',
                        'source_frame': 'tool0',
                        'parts': [
                            _part(
                                'payload__primitive_000',
                                'payload',
                                'cuboid',
                                dims=[0.1, 0.1, 0.2],
                            )
                        ],
                    },
                }
            ],
            'allowed_collision_matrix': {
                'entry_names': ['compound', 'link_a', 'link_b'],
                'entries': [
                    [False, False, False],
                    [False, False, False],
                    [False, False, False],
                ],
                'default_entry_names': [],
                'default_entries': [],
            },
        },
    }


def test_loads_primitive_mesh_compound_attached_and_transforms():
    snapshot = load_scene_snapshot(_request())

    assert snapshot.planning_frame == 'base_link'
    assert [part.kind for part in snapshot.world] == [
        'cuboid',
        'sphere',
        'cylinder',
        'cone',
        'mesh',
    ]
    assert len(snapshot.world[3].values['vertices']) == CONE_SEGMENTS + 2
    assert len(snapshot.world[3].values['faces']) == CONE_SEGMENTS * 2
    assert snapshot.attached[0].link_name == 'tool0'
    assert snapshot.attached[0].touch_links == ('left_finger', 'right_finger')


def test_cone_envelope_is_conservative_and_bounded():
    radius = 0.4
    vertices, _ = conservative_cone_mesh(radius, 0.8)
    base_radii = [math.hypot(x, y) for x, y, _ in vertices[2:]]

    assert min(base_radii) > radius
    assert max(base_radii) / radius < 1.005


def test_acm_disables_only_a_whole_world_object():
    request = _request()
    request['scene']['attached_objects'] = []
    request['scene']['allowed_collision_matrix']['entries'] = [
        [False, True, True],
        [True, False, False],
        [True, False, False],
    ]
    snapshot = load_scene_snapshot(request)

    assert globally_allowed_world_objects(snapshot, ['link_a', 'link_b']) == {
        'compound'
    }


def test_world_allowance_cannot_silently_drop_attached_object_collision():
    request = _request()
    request['scene']['allowed_collision_matrix']['entries'] = [
        [False, True, True],
        [True, False, False],
        [True, False, False],
    ]
    snapshot = load_scene_snapshot(request)

    with pytest.raises(UnsupportedSceneError, match='partial ACM'):
        globally_allowed_world_objects(snapshot, ['link_a', 'link_b'])


def test_partial_world_acm_fails_closed():
    request = _request()
    request['scene']['allowed_collision_matrix']['entries'][0][1] = True
    request['scene']['allowed_collision_matrix']['entries'][1][0] = True
    snapshot = load_scene_snapshot(request)

    with pytest.raises(UnsupportedSceneError, match='partial ACM'):
        globally_allowed_world_objects(snapshot, ['link_a', 'link_b'])


def test_robot_self_acm_requires_profile_confirmation():
    request = _request()
    request['scene']['allowed_collision_matrix']['entries'][1][2] = True
    request['scene']['allowed_collision_matrix']['entries'][2][1] = True
    snapshot = load_scene_snapshot(request)

    with pytest.raises(UnsupportedSceneError, match='не подтверждён'):
        globally_allowed_world_objects(snapshot, ['link_a', 'link_b'])
    assert globally_allowed_world_objects(
        snapshot,
        ['link_a', 'link_b'],
        supported_self_collision_pairs=[('link_b', 'link_a')],
    ) == set()


def test_rejects_missing_frame_snapshot_and_diff_state():
    request = deepcopy(_request())
    request['scene']['frame_transforms'] = request['scene']['frame_transforms'][:1]
    with pytest.raises(SceneContractError, match='source_frame|link_name'):
        load_scene_snapshot(request)

    request = deepcopy(_request())
    request['scene']['robot_state']['is_diff'] = True
    with pytest.raises(SceneContractError, match='RobotState'):
        load_scene_snapshot(request)


def test_requires_workspace_frame_for_every_goal_type():
    request = deepcopy(_request())
    del request['workspace_frame']
    with pytest.raises(SceneContractError, match='workspace_frame'):
        load_scene_snapshot(request)

    request = deepcopy(_request())
    request['workspace_frame'] = 'other_frame'
    with pytest.raises(UnsupportedSceneError, match='workspace_frame'):
        load_scene_snapshot(request)
