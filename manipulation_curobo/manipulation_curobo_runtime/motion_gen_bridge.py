"""
Validated one-shot MotionGen planning over a request-owned PlanningScene.

Responsibility:
    Build cuRobo collision geometry from the immutable private scene contract,
    update one MotionGen instance, attach request geometry, and plan once.

Does not own:
    ROS actions, persistent planner state, static world truth, or execution.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
import re
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, Cylinder, Mesh, Sphere, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as CuPose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from manipulation_curobo_runtime.scene_conversion import (
    AttachedGeometry,
    GeometrySpec,
    SceneSnapshot,
    UnsupportedSceneError,
    enabled_robot_self_collision_pairs,
    globally_allowed_world_objects,
    load_scene_snapshot,
)


@dataclass(frozen=True)
class MotionGenBridgeConfig:
    """Validated builder settings injected by a robot-specific backend profile."""

    robot_cfg: Any
    tensor_device: str
    motion_gen_kwargs: Dict[str, Any]
    default_max_attempts: int
    workspace_frame: str
    ee_link_name: str
    attached_link_map: Mapping[str, str]
    attached_touch_links: Mapping[str, frozenset[str]]


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _resolve_collision_checker(value: Optional[str]) -> Optional[CollisionCheckerType]:
    if not value:
        return None
    normalized = value.strip().upper()
    mapping = {
        'MESH': 'MESH',
        'VOXEL': 'VOXEL',
        'SIGNED_DISTANCE_FIELD': 'SIGNED_DISTANCE_FIELD',
    }
    if normalized not in mapping:
        raise ValueError(f'Неизвестный collision checker: {value!r}')
    attribute = mapping[normalized]
    resolved = getattr(CollisionCheckerType, attribute, None)
    if resolved is None:
        raise ValueError(
            f'Текущая версия cuRobo не поддерживает collision checker {normalized!r}'
        )
    return resolved


def _resolve_package_uri(value: Any) -> Any:
    if not isinstance(value, str) or not value.startswith('package://'):
        return value
    package_resource = value.removeprefix('package://')
    package_name, separator, relative_path = package_resource.partition('/')
    if not package_name or not separator or not relative_path:
        raise ValueError(f'Некорректный package URI: {value}')
    relative_resource = Path(relative_path)
    if relative_resource.is_absolute() or '..' in relative_resource.parts:
        raise ValueError(f'package URI выходит за границы package: {value}')
    resolved = Path(get_package_share_directory(package_name)) / relative_resource
    if not resolved.is_file():
        raise FileNotFoundError(f'Ресурс package URI не найден: {value} -> {resolved}')
    return str(resolved)


def _request_scale(request: Dict[str, Any], name: str) -> float | None:
    if name not in request:
        return None
    value = float(request[name])
    if not math.isfinite(value) or value <= 0.0 or value > 1.0:
        raise ValueError(f'{name} должен быть в диапазоне (0, 1]')
    return value


def _string_map(value: Any, name: str) -> Dict[str, str]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f'motion_gen.{name} должен быть mapping')
    result = {}
    for key, mapped in value.items():
        source = str(key).strip()
        target = str(mapped).strip()
        if not source or not target:
            raise ValueError(f'motion_gen.{name} содержит пустое имя')
        result[source] = target
    return result


def _touch_link_map(value: Any) -> Dict[str, frozenset[str]]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError('motion_gen.attached_touch_links должен быть mapping')
    result = {}
    for key, links in value.items():
        source = str(key).strip()
        if not source or not isinstance(links, Sequence) or isinstance(links, str):
            raise ValueError('motion_gen.attached_touch_links содержит invalid entry')
        normalized = frozenset(str(link).strip() for link in links)
        if any(not link for link in normalized):
            raise ValueError('motion_gen.attached_touch_links содержит пустой link')
        result[source] = normalized
    return result


def load_bridge_config(
    config_path: Path, request: Dict[str, Any]
) -> MotionGenBridgeConfig:
    """Load builder settings without accepting a parallel static world source."""
    config_dict = _load_yaml(config_path)
    if not isinstance(config_dict, dict) or 'motion_gen' not in config_dict:
        raise RuntimeError(f'В {config_path} отсутствует секция motion_gen')
    mg_cfg = dict(config_dict['motion_gen'])
    if 'world_cfg' in mg_cfg:
        raise ValueError(
            'motion_gen.world_cfg удалён: world берётся только из PlanningScene request'
        )
    robot_cfg = _resolve_package_uri(mg_cfg.pop('robot_cfg'))
    tensor_device = str(mg_cfg.pop('tensor_device', 'cuda:0'))
    default_max_attempts = max(1, int(mg_cfg.pop('max_attempts', 1)))
    workspace_frame = str(mg_cfg.pop('workspace_frame', '')).strip()
    ee_link_name = str(mg_cfg.get('ee_link_name', '')).strip()
    if not tensor_device.strip():
        raise ValueError('motion_gen.tensor_device не может быть пустым')
    if not workspace_frame:
        raise ValueError('motion_gen.workspace_frame не может быть пустым')
    if not ee_link_name:
        raise ValueError('motion_gen.ee_link_name не может быть пустым')
    attached_link_map = _string_map(
        mg_cfg.pop('attached_link_map', None), 'attached_link_map'
    )
    attached_touch_links = _touch_link_map(
        mg_cfg.pop('attached_touch_links', None)
    )

    velocity_scale = _request_scale(request, 'velocity_scale')
    acceleration_scale = _request_scale(request, 'acceleration_scale')
    if velocity_scale is not None:
        mg_cfg['velocity_scale'] = velocity_scale
    if acceleration_scale is not None:
        mg_cfg['acceleration_scale'] = acceleration_scale
    collision_checker = mg_cfg.pop('collision_checker', None)
    if collision_checker:
        mg_cfg['collision_checker_type'] = _resolve_collision_checker(collision_checker)
    return MotionGenBridgeConfig(
        robot_cfg=robot_cfg,
        tensor_device=tensor_device,
        motion_gen_kwargs=mg_cfg,
        default_max_attempts=default_max_attempts,
        workspace_frame=workspace_frame,
        ee_link_name=ee_link_name,
        attached_link_map=attached_link_map,
        attached_touch_links=attached_touch_links,
    )


def _validate_cuda_readiness(tensor_device: str) -> None:
    """Check the configured CUDA device without allocating tensors or warming kernels."""
    if tensor_device == 'cpu':
        return
    if not tensor_device.startswith('cuda'):
        raise ValueError(f'Неподдерживаемое имя tensor_device: {tensor_device!r}')
    match = re.fullmatch(r'cuda(?::([0-9]+))?', tensor_device)
    if match is None:
        raise ValueError(f'Некорректное имя CUDA device: {tensor_device!r}')
    if not torch.cuda.is_available():
        raise RuntimeError(
            f'Запрошен {tensor_device}, но CUDA недоступна; cuRobo backend не готов'
        )
    device_index = int(match.group(1) or 0)
    device_count = int(torch.cuda.device_count())
    if device_index >= device_count:
        raise RuntimeError(
            f'Запрошен {tensor_device}, но доступно CUDA devices: {device_count}'
        )


def probe_backend(config_path: Path) -> Dict[str, str]:
    """Validate imports, robot config and CUDA readiness without MotionGen warmup."""
    config = load_bridge_config(config_path, {})
    document = _load_robot_config_document(config.robot_cfg)
    robot = document.get('robot_cfg', document)
    if not isinstance(robot, Mapping) or not isinstance(robot.get('kinematics'), Mapping):
        raise ValueError('cuRobo robot_cfg не содержит robot_cfg.kinematics')
    collision_links = tuple(robot['kinematics'].get('collision_link_names', ()))
    if not collision_links or any(not str(link).strip() for link in collision_links):
        raise ValueError('cuRobo robot_cfg не содержит collision_link_names')
    _validate_cuda_readiness(config.tensor_device)
    return {
        'workspace_frame': config.workspace_frame,
        'ee_link': config.ee_link_name,
        'tensor_device': config.tensor_device,
    }


def _curobo_pose(pose_xyzw: Sequence[float]) -> List[float]:
    x, y, z, qx, qy, qz, qw = pose_xyzw
    return [x, y, z, qw, qx, qy, qz]


def _obstacle(spec: GeometrySpec) -> Any:
    common = {'name': spec.name, 'pose': _curobo_pose(spec.pose)}
    if spec.kind == 'cuboid':
        return Cuboid(**common, dims=list(spec.values['dims']))
    if spec.kind == 'sphere':
        return Sphere(**common, radius=spec.values['radius'])
    if spec.kind == 'cylinder':
        return Cylinder(
            **common,
            radius=spec.values['radius'],
            height=spec.values['height'],
        )
    if spec.kind in {'mesh', 'cone'}:
        return Mesh(
            **common,
            vertices=[list(vertex) for vertex in spec.values['vertices']],
            faces=[list(face) for face in spec.values['faces']],
        )
    raise UnsupportedSceneError(f'Неподдерживаемый geometry kind: {spec.kind!r}')


def build_world(
    snapshot: SceneSnapshot, excluded_owner_ids: Iterable[str] = ()
) -> WorldConfig:
    """Create a request-owned WorldConfig, preserving primitive fidelity."""
    excluded = frozenset(excluded_owner_ids)
    cuboids = []
    spheres = []
    cylinders = []
    meshes = []
    for spec in snapshot.world:
        if spec.owner_id in excluded:
            continue
        obstacle = _obstacle(spec)
        if spec.kind == 'cuboid':
            cuboids.append(obstacle)
        elif spec.kind == 'sphere':
            spheres.append(obstacle)
        elif spec.kind == 'cylinder':
            cylinders.append(obstacle)
        else:
            meshes.append(obstacle)
    return WorldConfig(
        cuboid=cuboids,
        sphere=spheres,
        cylinder=cylinders,
        mesh=meshes,
    )


def _load_robot_config_document(robot_cfg: Any) -> Dict[str, Any]:
    """Load a fresh cuRobo robot document without mutating shared config state."""
    if isinstance(robot_cfg, Mapping):
        document = deepcopy(dict(robot_cfg))
    else:
        source = Path(str(robot_cfg))
        if source.is_file():
            document = _load_yaml(source)
        else:
            from curobo.util_file import (  # Imported only in the selected backend.
                get_robot_configs_path,
                join_path,
                load_yaml as load_curobo_yaml,
            )

            document = load_curobo_yaml(
                join_path(get_robot_configs_path(), str(robot_cfg))
            )
    if not isinstance(document, Mapping):
        raise ValueError('cuRobo robot_cfg должен содержать mapping')
    return deepcopy(dict(document))


def _request_robot_config(
    config: MotionGenBridgeConfig, snapshot: SceneSnapshot
) -> Dict[str, Any]:
    """Apply the reviewed MoveIt ACM and attachment touch pairs to cuRobo."""
    document = _load_robot_config_document(config.robot_cfg)
    robot = document.get('robot_cfg', document)
    if not isinstance(robot, dict) or not isinstance(robot.get('kinematics'), dict):
        raise ValueError('cuRobo robot_cfg не содержит robot_cfg.kinematics')
    kinematics = robot['kinematics']
    collision_links = frozenset(
        str(link).strip() for link in kinematics.get('collision_link_names', ())
    )
    if not collision_links or any(not link for link in collision_links):
        raise ValueError('cuRobo robot_cfg не содержит collision_link_names')

    ignored_pairs = set(
        enabled_robot_self_collision_pairs(snapshot, collision_links)
    )
    for moveit_link, proxy_link in config.attached_link_map.items():
        touch_links = config.attached_touch_links.get(moveit_link)
        if touch_links is None:
            raise ValueError(
                f'Нет attached_touch_links для MoveIt link {moveit_link!r}'
            )
        ignored_pairs.update(
            tuple(sorted((proxy_link, touch_link))) for touch_link in touch_links
        )
    invalid_pairs = sorted(
        pair for pair in ignored_pairs
        if pair[0] not in collision_links or pair[1] not in collision_links
    )
    if invalid_pairs:
        raise ValueError(
            f'ACM/touch pairs отсутствуют в cuRobo collision links: {invalid_pairs}'
        )

    self_collision_ignore: Dict[str, List[str]] = {}
    for left, right in sorted(ignored_pairs):
        self_collision_ignore.setdefault(left, []).append(right)
    kinematics['self_collision_ignore'] = self_collision_ignore
    return document


def build_motion_gen(
    config: MotionGenBridgeConfig,
    world: WorldConfig,
    snapshot: SceneSnapshot,
) -> MotionGen:
    """Construct one MotionGen whose cache is sized from the request scene."""
    tensor_device = config.tensor_device
    if tensor_device.startswith('cuda') and not torch.cuda.is_available():
        raise RuntimeError(
            f'Запрошен {tensor_device}, но CUDA недоступна; текущий cuRobo требует CUDA'
        )
    tensor_args = TensorDeviceType(device=tensor_device)
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        _request_robot_config(config, snapshot),
        world,
        tensor_args,
        **config.motion_gen_kwargs,
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen


def _as_pose_list(
    position: Sequence[float], orientation_xyzw: Sequence[float]
) -> List[float]:
    if len(position) != 3 or len(orientation_xyzw) != 4:
        raise ValueError('Некорректный формат позы')
    values = [float(value) for value in (*position, *orientation_xyzw)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Pose goal содержит NaN или Inf')
    x, y, z, ox, oy, oz, ow = values
    quaternion_norm = math.sqrt(ox * ox + oy * oy + oz * oz + ow * ow)
    if quaternion_norm <= 1.0e-12:
        raise ValueError('Pose goal содержит нулевой quaternion')
    return [
        x,
        y,
        z,
        ow / quaternion_norm,
        ox / quaternion_norm,
        oy / quaternion_norm,
        oz / quaternion_norm,
    ]


def _joint_state(
    joint_names: Sequence[str], positions: Sequence[float], device: torch.device
) -> JointState:
    tensor = torch.tensor([positions], dtype=torch.float32, device=device)
    return JointState.from_position(tensor, joint_names=joint_names)


def _tensor_to_list(tensor: torch.Tensor) -> List[List[float]]:
    if tensor is None:
        return []
    data = tensor.detach().cpu()
    if data.dim() > 2:
        data = data[0]
    return data.tolist()


def _robot_link_names(motion_gen: MotionGen) -> List[str]:
    """Return the complete kinematic link inventory across supported cuRobo APIs."""
    kinematics = motion_gen.robot_cfg.kinematics.kinematics_config
    indexed_links = getattr(kinematics, 'link_name_to_idx_map', None)
    if isinstance(indexed_links, Mapping) and indexed_links:
        return list(indexed_links)
    return list(getattr(kinematics, 'link_names', ()))


def _collision_links(
    motion_gen: MotionGen, excluded_links: Iterable[str] = ()
) -> List[str]:
    kinematics = motion_gen.robot_cfg.kinematics.kinematics_config
    excluded = frozenset(excluded_links)
    return [
        link
        for link in _robot_link_names(motion_gen)
        if link not in excluded and kinematics.get_number_of_spheres(link) > 0
    ]


def _attach_scene_geometry(
    motion_gen: MotionGen,
    start_state: JointState,
    attached: Sequence[AttachedGeometry],
    config: MotionGenBridgeConfig,
) -> None:
    grouped: Dict[str, List[Any]] = {}
    for item in attached:
        if item.link_name != config.ee_link_name:
            raise UnsupportedSceneError(
                f"MoveIt attachment link {item.link_name!r} не совпадает с "
                f"cuRobo ee link {config.ee_link_name!r}"
            )
        placeholder = config.attached_link_map.get(item.link_name)
        if not placeholder:
            raise UnsupportedSceneError(
                f'Нет attached_link_map для MoveIt link {item.link_name!r}'
            )
        configured_touch_links = config.attached_touch_links.get(item.link_name)
        required_touch_links = frozenset((*item.touch_links, item.link_name))
        if configured_touch_links is None or required_touch_links != configured_touch_links:
            raise UnsupportedSceneError(
                f'attached touch_links для {item.object_id!r} не подтверждены robot profile'
            )
        grouped.setdefault(placeholder, []).extend(_obstacle(part) for part in item.parts)
    for placeholder, obstacles in grouped.items():
        attached_ok = motion_gen.attach_external_objects_to_robot(
            start_state,
            obstacles,
            link_name=placeholder,
        )
        if not attached_ok:
            raise UnsupportedSceneError(
                f'cuRobo не смог представить attached geometry на {placeholder!r}'
            )


def plan_motion(
    motion_gen: MotionGen,
    request: Dict[str, Any],
    config: MotionGenBridgeConfig,
    snapshot: SceneSnapshot | None = None,
) -> Dict[str, Any]:
    """Update request-owned world/attachments and execute exactly one plan."""
    snapshot = snapshot or load_scene_snapshot(request)
    request_workspace_frame = str(request.get('workspace_frame', '')).strip()
    if not request_workspace_frame:
        raise ValueError('Request workspace_frame не может быть пустым')
    if request_workspace_frame != snapshot.planning_frame:
        raise UnsupportedSceneError(
            f"Request workspace frame '{request_workspace_frame}' не совпадает с "
            f"PlanningScene frame '{snapshot.planning_frame}'"
        )
    if request_workspace_frame != config.workspace_frame:
        raise ValueError(
            f"Request workspace frame '{request_workspace_frame}' не совпадает с "
            f"cuRobo workspace frame '{config.workspace_frame}'"
        )
    joint_names = request['joint_names']
    start_positions = request['start_positions']
    goal = request['goal']
    goal_type = str(goal.get('type', '')).strip()
    if goal_type not in {'joint', 'pose'}:
        raise ValueError(f'Неподдерживаемый goal.type: {goal_type!r}')
    if goal_type == 'pose':
        goal_frame = str(goal.get('frame_id', '')).strip()
        if not goal_frame:
            raise ValueError('Pose goal должен содержать frame_id')
        if goal_frame != snapshot.planning_frame:
            raise UnsupportedSceneError(
                f"Pose goal frame '{goal_frame}' не совпадает с PlanningScene frame "
                f"'{snapshot.planning_frame}'"
            )
        if goal_frame != config.workspace_frame:
            raise ValueError(
                f"Pose goal frame '{goal_frame}' не совпадает с "
                f"cuRobo workspace frame '{config.workspace_frame}'"
            )
        goal_link = str(goal.get('link_name', '')).strip()
        if goal_link != config.ee_link_name:
            raise ValueError(
                f"Pose goal link '{goal_link}' не совпадает с cuRobo ee link "
                f"'{config.ee_link_name}'"
            )

    robot_link_names = _robot_link_names(motion_gen)
    collision_links = _collision_links(
        motion_gen, config.attached_link_map.values()
    )
    disabled_owners = globally_allowed_world_objects(
        snapshot,
        collision_links,
        robot_link_names=robot_link_names,
        supported_self_collision_pairs=enabled_robot_self_collision_pairs(
            snapshot, robot_link_names
        ),
    )
    motion_gen.update_world(build_world(snapshot, disabled_owners))
    start_state = _joint_state(
        joint_names, start_positions, motion_gen.tensor_args.device
    )
    _attach_scene_geometry(motion_gen, start_state, snapshot.attached, config)

    plan_timeout = max(0.001, float(request.get('plan_timeout', 5.0)))
    plan_config = MotionGenPlanConfig(
        max_attempts=max(
            1, int(request.get('max_attempts', config.default_max_attempts))
        ),
        timeout=plan_timeout,
    )
    start_time = time.monotonic()
    if goal_type == 'joint':
        joint_goal = _joint_state(
            joint_names,
            goal['joint_positions'],
            motion_gen.tensor_args.device,
        )
        result = motion_gen.plan_single_js(start_state, joint_goal, plan_config)
    else:
        pose_vec = _as_pose_list(goal['position'], goal['orientation'])
        result = motion_gen.plan_single(
            start_state, CuPose.from_list(pose_vec), plan_config
        )
    planning_time = time.monotonic() - start_time
    if not bool(result.success.item()):
        timed_out = planning_time >= plan_timeout
        return {
            'success': False,
            'error_code': 'timed_out' if timed_out else 'planning_failed',
            'message': (
                'MotionGen исчерпал planning budget.'
                if timed_out
                else 'MotionGen не смог построить план.'
            ),
            'planning_time': planning_time,
        }

    plan = result.get_interpolated_plan()
    plan_joint_names = list(getattr(plan, 'joint_names', []) or [])
    if not plan_joint_names:
        raise RuntimeError('cuRobo не вернул joint_names интерполированной траектории')
    trajectory = {
        'dt': float(result.interpolation_dt),
        'joint_names': plan_joint_names,
        'points': [],
    }
    positions = _tensor_to_list(plan.position)
    velocities = _tensor_to_list(plan.velocity)
    accelerations = _tensor_to_list(plan.acceleration)
    for index, position in enumerate(positions):
        point = {'positions': position}
        if index < len(velocities):
            point['velocities'] = velocities[index]
        if index < len(accelerations):
            point['accelerations'] = accelerations[index]
        trajectory['points'].append(point)
    return {
        'success': True,
        'message': '',
        'planning_time': planning_time,
        'trajectory': trajectory,
    }
