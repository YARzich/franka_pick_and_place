"""
CPU fake tests for request-owned world update and MotionGen conversion.

Responsibility: Verify bridge conversion without requiring CUDA or cuRobo.
"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


class FakeTensor:
    def __init__(self, values):
        self._values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def dim(self):
        return 2

    def tolist(self):
        return self._values


class FakeScalar:
    def __init__(self, value):
        self._value = value

    def item(self):
        return self._value


class FakeObstacle:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class FakeWorldConfig:
    def __init__(self, **kwargs):
        self.cuboid = kwargs.get('cuboid', [])
        self.sphere = kwargs.get('sphere', [])
        self.cylinder = kwargs.get('cylinder', [])
        self.mesh = kwargs.get('mesh', [])


def _install_fake_curobo_modules():
    class FakeCollisionCheckerType:
        MESH = 'mesh'
        VOXEL = 'voxel'
        SIGNED_DISTANCE_FIELD = 'signed_distance_field'

    class FakeTensorDeviceType:
        def __init__(self, device):
            self.device = device

    class FakePose:
        @staticmethod
        def from_list(values):
            return values

    class FakeJointState:
        @staticmethod
        def from_position(position, joint_names):
            return SimpleNamespace(position=position, joint_names=joint_names)

    modules = {
        'torch': ModuleType('torch'),
        'curobo': ModuleType('curobo'),
        'curobo.geom': ModuleType('curobo.geom'),
        'curobo.geom.sdf': ModuleType('curobo.geom.sdf'),
        'curobo.geom.sdf.world': ModuleType('curobo.geom.sdf.world'),
        'curobo.geom.types': ModuleType('curobo.geom.types'),
        'curobo.types': ModuleType('curobo.types'),
        'curobo.types.base': ModuleType('curobo.types.base'),
        'curobo.types.math': ModuleType('curobo.types.math'),
        'curobo.types.robot': ModuleType('curobo.types.robot'),
        'curobo.wrap': ModuleType('curobo.wrap'),
        'curobo.wrap.reacher': ModuleType('curobo.wrap.reacher'),
        'curobo.wrap.reacher.motion_gen': ModuleType('curobo.wrap.reacher.motion_gen'),
    }
    modules['curobo.geom.sdf.world'].CollisionCheckerType = FakeCollisionCheckerType
    for name in ('Cuboid', 'Cylinder', 'Mesh', 'Sphere'):
        setattr(modules['curobo.geom.types'], name, FakeObstacle)
    modules['curobo.geom.types'].WorldConfig = FakeWorldConfig
    modules['curobo.types.base'].TensorDeviceType = FakeTensorDeviceType
    modules['curobo.types.math'].Pose = FakePose
    modules['curobo.types.robot'].JointState = FakeJointState
    modules['curobo.wrap.reacher.motion_gen'].MotionGen = object
    modules['curobo.wrap.reacher.motion_gen'].MotionGenConfig = object
    modules['curobo.wrap.reacher.motion_gen'].MotionGenPlanConfig = object
    modules['torch'].Tensor = FakeTensor
    modules['torch'].cuda = SimpleNamespace(
        is_available=lambda: False, device_count=lambda: 0
    )
    sys.modules.update(modules)


_install_fake_curobo_modules()

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_PATH = PACKAGE_ROOT / 'manipulation_curobo_runtime' / 'motion_gen_bridge.py'
CONFIG_PATH = PACKAGE_ROOT / 'test' / 'fixtures' / 'motion_gen.yaml'
SPEC = importlib.util.spec_from_file_location('motion_gen_bridge_for_test', BRIDGE_PATH)
motion_gen_bridge = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = motion_gen_bridge
SPEC.loader.exec_module(motion_gen_bridge)


def _scene(attached=False):
    attached_objects = []
    if attached:
        attached_objects.append(
            {
                'link_name': 'tool0',
                'touch_links': ['left_finger', 'right_finger'],
                'object': {
                    'id': 'payload',
                    'source_frame': 'tool0',
                    'parts': [
                        {
                            'name': 'payload__primitive_000',
                            'owner_id': 'payload',
                            'kind': 'cuboid',
                            'pose': [0.4, 0, 0.3, 0, 0, 0, 1],
                            'dims': [0.1, 0.1, 0.2],
                        }
                    ],
                },
            }
        )
    return {
        'is_diff': False,
        'planning_frame': 'base_link',
        'robot_state': {
            'is_diff': False,
            'joint_names': ['joint_a', 'joint_b'],
            'joint_positions': [0.1, 0.2],
        },
        'frame_transforms': [
            {
                'parent_frame': 'base_link',
                'child_frame': 'base_link',
                'pose': [0, 0, 0, 0, 0, 0, 1],
            },
            {
                'parent_frame': 'base_link',
                'child_frame': 'tool0',
                'pose': [0.4, 0, 0.3, 0, 0, 0, 1],
            },
        ],
        'world_objects': [
            {
                'id': 'compound',
                'source_frame': 'base_link',
                'parts': [
                    {
                        'name': 'compound__primitive_000',
                        'owner_id': 'compound',
                        'kind': 'sphere',
                        'pose': [0.6, 0, 0.2, 0, 0, 0, 1],
                        'radius': 0.1,
                    },
                    {
                        'name': 'compound__mesh_000',
                        'owner_id': 'compound',
                        'kind': 'mesh',
                        'pose': [0.6, 0, 0.2, 0, 0, 0, 1],
                        'vertices': [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                        'faces': [[0, 1, 2]],
                    },
                ],
            }
        ],
        'attached_objects': attached_objects,
        'allowed_collision_matrix': {
            'entry_names': [],
            'entries': [],
            'default_entry_names': [],
            'default_entries': [],
        },
    }


def _pose_request(frame_id='base_link', attached=False):
    return {
        'schema_version': 2,
        'workspace_frame': 'base_link',
        'joint_names': ['joint_a', 'joint_b'],
        'start_positions': [0.1, 0.2],
        'max_attempts': 3,
        'plan_timeout': 2.5,
        'velocity_scale': 0.5,
        'acceleration_scale': 0.4,
        'goal': {
            'type': 'pose',
            'frame_id': frame_id,
            'link_name': 'tool0',
            'position': [0.4, 0.0, 0.3],
            'orientation': [0.0, 0.0, 0.0, 1.0],
        },
        'scene': _scene(attached),
    }


def _config():
    return motion_gen_bridge.MotionGenBridgeConfig(
        robot_cfg='example_robot.yml',
        tensor_device='cpu',
        motion_gen_kwargs={},
        default_max_attempts=2,
        workspace_frame='base_link',
        ee_link_name='tool0',
        attached_link_map={'tool0': 'attached_object'},
        attached_touch_links={
            'tool0': frozenset({'tool0', 'left_finger', 'right_finger'})
        },
    )


def test_load_bridge_config_has_no_static_world(monkeypatch):
    monkeypatch.setattr(
        motion_gen_bridge,
        'get_package_share_directory',
        lambda package_name: (
            str(PACKAGE_ROOT) if package_name == 'manipulation_curobo' else None
        ),
    )

    config = motion_gen_bridge.load_bridge_config(CONFIG_PATH, _pose_request())

    assert not hasattr(config, 'world_cfg')
    assert config.attached_link_map == {'tool0': 'attached_object'}
    assert config.motion_gen_kwargs['velocity_scale'] == pytest.approx(0.5)
    assert config.motion_gen_kwargs['acceleration_scale'] == pytest.approx(0.4)


def test_mesh_checker_does_not_require_unselected_sdf_enum(monkeypatch):
    """Older cuRobo releases need only expose the enum selected by the profile."""
    checker_type = SimpleNamespace(MESH='mesh', VOXEL='voxel')
    monkeypatch.setattr(motion_gen_bridge, 'CollisionCheckerType', checker_type)

    assert motion_gen_bridge._resolve_collision_checker('mesh') == 'mesh'
    with pytest.raises(ValueError, match='SIGNED_DISTANCE_FIELD'):
        motion_gen_bridge._resolve_collision_checker('signed_distance_field')


@pytest.mark.parametrize('name', ['voxelx', 'unknown', '   '])
def test_unknown_collision_checker_is_rejected_during_config_load(monkeypatch, name):
    document = motion_gen_bridge._load_yaml(CONFIG_PATH)
    document['motion_gen']['collision_checker'] = name
    monkeypatch.setattr(motion_gen_bridge, '_load_yaml', lambda _path: document)

    with pytest.raises(ValueError, match='collision checker'):
        motion_gen_bridge.load_bridge_config(CONFIG_PATH, {})


def test_absent_collision_checker_keeps_backend_default(monkeypatch):
    document = motion_gen_bridge._load_yaml(CONFIG_PATH)
    document['motion_gen'].pop('collision_checker', None)
    monkeypatch.setattr(motion_gen_bridge, '_load_yaml', lambda _path: document)

    config = motion_gen_bridge.load_bridge_config(CONFIG_PATH, {})

    assert 'collision_checker_type' not in config.motion_gen_kwargs


def test_collision_links_use_complete_index_map_not_ee_only_link_names():
    """Released cuRobo may expose only requested FK links through link_names."""
    kinematics = SimpleNamespace(
        link_names=['panda_hand'],
        link_name_to_idx_map={
            'panda_link0': 0,
            'panda_link1': 1,
            'panda_hand': 2,
            'attached_object': 3,
        },
        get_number_of_spheres=lambda name: 0 if name == 'panda_link1' else 1,
    )
    motion_gen = SimpleNamespace(
        robot_cfg=SimpleNamespace(
            kinematics=SimpleNamespace(kinematics_config=kinematics)
        )
    )

    assert motion_gen_bridge._collision_links(
        motion_gen, {'attached_object'}
    ) == ['panda_link0', 'panda_hand']


def test_runtime_robot_config_replaces_self_collision_policy(monkeypatch):
    config = _config()
    monkeypatch.setattr(
        motion_gen_bridge,
        '_load_robot_config_document',
        lambda _robot_cfg: {
            'robot_cfg': {
                'kinematics': {
                    'collision_link_names': [
                        'attached_object',
                        'left_finger',
                        'link_a',
                        'link_b',
                        'right_finger',
                        'tool0',
                    ],
                    'self_collision_ignore': {'stale': ['policy']},
                }
            }
        },
    )

    request = _pose_request()
    request['scene']['allowed_collision_matrix'] = {
        'entry_names': ['link_a', 'link_b'],
        'entries': [[False, True], [True, False]],
        'default_entry_names': [],
        'default_entries': [],
    }
    snapshot = motion_gen_bridge.load_scene_snapshot(request)

    document = motion_gen_bridge._request_robot_config(config, snapshot)

    assert document['robot_cfg']['kinematics']['self_collision_ignore'] == {
        'attached_object': ['left_finger', 'right_finger', 'tool0'],
        'link_a': ['link_b'],
    }


def test_static_world_key_is_rejected(monkeypatch):
    monkeypatch.setattr(
        motion_gen_bridge,
        '_load_yaml',
        lambda _path: {
            'motion_gen': {
                'robot_cfg': 'robot.yml',
                'world_cfg': 'stale.yml',
                'ee_link_name': 'tool0',
            }
        },
    )

    with pytest.raises(ValueError, match='world_cfg удалён'):
        motion_gen_bridge.load_bridge_config(CONFIG_PATH, _pose_request())


def test_workspace_frame_is_required_in_backend_config(monkeypatch):
    monkeypatch.setattr(
        motion_gen_bridge,
        '_load_yaml',
        lambda _path: {
            'motion_gen': {
                'robot_cfg': 'robot.yml',
                'workspace_frame': '',
                'ee_link_name': 'tool0',
            }
        },
    )

    with pytest.raises(ValueError, match='workspace_frame'):
        motion_gen_bridge.load_bridge_config(CONFIG_PATH, _pose_request())


def test_readiness_probe_checks_config_and_cuda_without_warmup(monkeypatch):
    monkeypatch.setattr(
        motion_gen_bridge,
        '_load_robot_config_document',
        lambda _robot_cfg: {
            'robot_cfg': {
                'kinematics': {'collision_link_names': ['tool0']}
            }
        },
    )
    monkeypatch.setattr(
        motion_gen_bridge,
        'load_bridge_config',
        lambda _path, _request: motion_gen_bridge.MotionGenBridgeConfig(
            **{**_config().__dict__, 'tensor_device': 'cuda:0'}
        ),
    )

    with pytest.raises(RuntimeError, match='CUDA недоступна'):
        motion_gen_bridge.probe_backend(CONFIG_PATH)


def test_readiness_probe_success_does_not_construct_motion_gen(monkeypatch):
    monkeypatch.setattr(
        motion_gen_bridge,
        '_load_robot_config_document',
        lambda _robot_cfg: {
            'robot_cfg': {
                'kinematics': {'collision_link_names': ['tool0']}
            }
        },
    )
    monkeypatch.setattr(
        motion_gen_bridge,
        'load_bridge_config',
        lambda _path, _request: _config(),
    )

    assert motion_gen_bridge.probe_backend(CONFIG_PATH) == {
        'workspace_frame': 'base_link',
        'ee_link': 'tool0',
        'tensor_device': 'cpu',
    }


def test_joint_goal_rejects_workspace_scene_mismatch_before_planning():
    request = _pose_request()
    request['goal'] = {'type': 'joint', 'joint_positions': [0.2, 0.3]}
    request['workspace_frame'] = 'different_frame'

    with pytest.raises(
        motion_gen_bridge.UnsupportedSceneError, match='workspace_frame'
    ):
        motion_gen_bridge.load_scene_snapshot(request)


def test_plan_updates_request_world_and_attaches_via_explicit_proxy(monkeypatch):
    captured = {}

    class FakePlanConfig:
        def __init__(self, **kwargs):
            captured['plan_config'] = kwargs

    plan = SimpleNamespace(
        joint_names=['joint_b', 'joint_a'],
        position=FakeTensor([[0.2, 0.1], [0.3, 0.2]]),
        velocity=FakeTensor([[0.0, 0.0], [0.1, 0.1]]),
        acceleration=FakeTensor([[0.0, 0.0], [0.0, 0.0]]),
    )
    result = SimpleNamespace(
        success=FakeScalar(True),
        interpolation_dt=0.02,
        get_interpolated_plan=lambda: plan,
    )

    class FakeKinematics:
        link_names = ['link_a', 'attached_object']

        @staticmethod
        def get_number_of_spheres(_link):
            return 4

    class FakeMotionGen:
        tensor_args = SimpleNamespace(device='cpu')
        robot_cfg = SimpleNamespace(
            kinematics=SimpleNamespace(kinematics_config=FakeKinematics())
        )

        @staticmethod
        def update_world(world):
            captured['world'] = world

        @staticmethod
        def attach_external_objects_to_robot(
            _start, objects, *, link_name
        ):
            captured['attached'] = (objects, link_name)
            return True

        @staticmethod
        def plan_single(_start, goal_pose, _plan_config):
            captured['goal_pose'] = goal_pose
            return result

    monkeypatch.setattr(motion_gen_bridge, 'MotionGenPlanConfig', FakePlanConfig)
    monkeypatch.setattr(
        motion_gen_bridge,
        '_joint_state',
        lambda names, positions, device: (list(names), list(positions), device),
    )
    request = _pose_request(attached=True)
    snapshot = motion_gen_bridge.load_scene_snapshot(request)

    response = motion_gen_bridge.plan_motion(
        FakeMotionGen(), request, _config(), snapshot
    )

    assert len(captured['world'].sphere) == 1
    assert len(captured['world'].mesh) == 1
    assert captured['attached'][1] == 'attached_object'
    assert captured['attached'][0][0].name == 'payload__primitive_000'
    assert captured['plan_config'] == {'max_attempts': 3, 'timeout': 2.5}
    assert captured['goal_pose'] == [0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0]
    assert response['trajectory']['joint_names'] == ['joint_b', 'joint_a']


def test_attached_proxy_and_touch_mapping_are_mandatory(monkeypatch):
    request = _pose_request(attached=True)
    snapshot = motion_gen_bridge.load_scene_snapshot(request)
    motion_gen = SimpleNamespace(
        tensor_args=SimpleNamespace(device='cpu'),
        robot_cfg=SimpleNamespace(
            kinematics=SimpleNamespace(
                kinematics_config=SimpleNamespace(
                    link_names=[], get_number_of_spheres=lambda _name: 0
                )
            )
        ),
        update_world=lambda _world: None,
    )
    monkeypatch.setattr(
        motion_gen_bridge,
        '_joint_state',
        lambda names, positions, device: (list(names), list(positions), device),
    )
    config = _config()
    config = motion_gen_bridge.MotionGenBridgeConfig(
        **{**config.__dict__, 'attached_link_map': {}}
    )

    with pytest.raises(motion_gen_bridge.UnsupportedSceneError, match='attached_link_map'):
        motion_gen_bridge.plan_motion(motion_gen, request, config, snapshot)


def test_attachment_link_must_match_curobo_end_effector(monkeypatch):
    request = _pose_request(attached=True)
    request['goal'] = {
        'type': 'joint',
        'joint_positions': [0.2, 0.3],
    }
    request['scene']['frame_transforms'].append(
        {
            'parent_frame': 'base_link',
            'child_frame': 'wrist',
            'pose': [0.4, 0, 0.3, 0, 0, 0, 1],
        }
    )
    attachment = request['scene']['attached_objects'][0]
    attachment['link_name'] = 'wrist'
    attachment['object']['source_frame'] = 'wrist'
    snapshot = motion_gen_bridge.load_scene_snapshot(request)
    motion_gen = SimpleNamespace(
        tensor_args=SimpleNamespace(device='cpu'),
        robot_cfg=SimpleNamespace(
            kinematics=SimpleNamespace(
                kinematics_config=SimpleNamespace(
                    link_names=[], get_number_of_spheres=lambda _name: 0
                )
            )
        ),
        update_world=lambda _world: None,
    )
    monkeypatch.setattr(
        motion_gen_bridge,
        '_joint_state',
        lambda names, positions, device: (list(names), list(positions), device),
    )
    config = _config()
    config = motion_gen_bridge.MotionGenBridgeConfig(
        **{
            **config.__dict__,
            'attached_link_map': {'wrist': 'attached_object'},
            'attached_touch_links': {
                'wrist': frozenset({'wrist', 'left_finger', 'right_finger'})
            },
        }
    )

    with pytest.raises(
        motion_gen_bridge.UnsupportedSceneError, match='не совпадает'
    ):
        motion_gen_bridge.plan_motion(motion_gen, request, config, snapshot)


def test_failed_motion_gen_budget_is_typed_timeout(monkeypatch):
    request = _pose_request()
    snapshot = motion_gen_bridge.load_scene_snapshot(request)
    result = SimpleNamespace(success=FakeScalar(False))
    motion_gen = SimpleNamespace(
        tensor_args=SimpleNamespace(device='cpu'),
        robot_cfg=SimpleNamespace(
            kinematics=SimpleNamespace(
                kinematics_config=SimpleNamespace(
                    link_names=[], get_number_of_spheres=lambda _name: 0
                )
            )
        ),
        update_world=lambda _world: None,
        plan_single=lambda _start, _goal, _config: result,
    )
    monkeypatch.setattr(
        motion_gen_bridge, 'MotionGenPlanConfig', lambda **_kwargs: object()
    )
    monkeypatch.setattr(
        motion_gen_bridge,
        '_joint_state',
        lambda names, positions, device: (list(names), list(positions), device),
    )
    clock = iter((10.0, 12.6))
    monkeypatch.setattr(motion_gen_bridge.time, 'monotonic', lambda: next(clock))

    response = motion_gen_bridge.plan_motion(
        motion_gen, request, _config(), snapshot
    )

    assert response['error_code'] == 'timed_out'


@pytest.mark.parametrize('scale', [0.0, -0.1, 1.1, float('nan')])
def test_request_scale_rejects_invalid_values(scale):
    with pytest.raises(ValueError, match='диапазоне'):
        motion_gen_bridge._request_scale({'velocity_scale': scale}, 'velocity_scale')
