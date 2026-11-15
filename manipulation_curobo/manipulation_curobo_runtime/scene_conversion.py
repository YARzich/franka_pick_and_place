"""
Pure conversion of the private PlanningScene snapshot into cuRobo-neutral geometry.

Responsibility:
    Validate the complete request-owned scene, preserve deterministic object
    ownership, and express the subset that cuRobo can represent exactly.

Does not own:
    cuRobo imports, GPU state, ROS communication, or trajectory construction.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple


PRIVATE_SCHEMA_VERSION = 2
CONE_SEGMENTS = 32


class SceneContractError(ValueError):
    """The private request is malformed and cannot describe a planning scene."""


class UnsupportedSceneError(SceneContractError):
    """The request is valid MoveIt data that cuRobo cannot represent exactly."""


@dataclass(frozen=True)
class GeometrySpec:
    """One deterministic cuRobo-neutral collision geometry."""

    name: str
    owner_id: str
    kind: str
    pose: Tuple[float, ...]
    values: Mapping[str, Any]


@dataclass(frozen=True)
class AttachedGeometry:
    """All geometry attached to one MoveIt robot link."""

    object_id: str
    link_name: str
    touch_links: Tuple[str, ...]
    parts: Tuple[GeometrySpec, ...]


@dataclass(frozen=True)
class SceneSnapshot:
    """Validated immutable scene data, independent of the cuRobo Python API."""

    planning_frame: str
    world: Tuple[GeometrySpec, ...]
    attached: Tuple[AttachedGeometry, ...]
    acm: Mapping[str, Any]


def _finite_vector(
    value: Any, size: int, field: str, *, positive: bool = False
) -> Tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise SceneContractError(f'{field} должен быть последовательностью')
    values = tuple(float(item) for item in value)
    if len(values) != size or not all(math.isfinite(item) for item in values):
        raise SceneContractError(
            f'{field} должен содержать {size} конечных значений'
        )
    if positive and not all(item > 0.0 for item in values):
        raise SceneContractError(f'{field} должен содержать положительные значения')
    return values


def _pose(value: Any, field: str) -> Tuple[float, ...]:
    pose = _finite_vector(value, 7, field)
    norm = math.sqrt(sum(component * component for component in pose[3:]))
    if norm <= 1.0e-12:
        raise SceneContractError(f'{field} содержит нулевой quaternion')
    return (*pose[:3], *(component / norm for component in pose[3:]))


def conservative_cone_mesh(
    radius: float, height: float, segments: int = CONE_SEGMENTS
) -> Tuple[Tuple[Tuple[float, float, float], ...], Tuple[Tuple[int, int, int], ...]]:
    """
    Return a closed polygonal envelope that conservatively contains a cone.

    The base is a regular circumscribed polygon. With 32 sides its maximum
    radial over-approximation is below 0.5%, while the height and apex are exact.
    """
    radius = float(radius)
    height = float(height)
    if (
        not math.isfinite(radius)
        or not math.isfinite(height)
        or radius <= 0.0
        or height <= 0.0
        or segments < 8
    ):
        raise SceneContractError('cone radius/height/segments недопустимы')
    envelope_radius = radius / math.cos(math.pi / segments)
    vertices = [(0.0, 0.0, height / 2.0), (0.0, 0.0, -height / 2.0)]
    for index in range(segments):
        angle = (2.0 * math.pi * index) / segments + math.pi / segments
        vertices.append(
            (
                envelope_radius * math.cos(angle),
                envelope_radius * math.sin(angle),
                -height / 2.0,
            )
        )
    faces = []
    for index in range(segments):
        current = 2 + index
        following = 2 + ((index + 1) % segments)
        faces.append((0, current, following))
        faces.append((1, following, current))
    return tuple(vertices), tuple(faces)


def _geometry(part: Mapping[str, Any], owner_id: str) -> GeometrySpec:
    name = str(part.get('name', '')).strip()
    part_owner = str(part.get('owner_id', '')).strip()
    kind = str(part.get('kind', '')).strip()
    if not name or part_owner != owner_id:
        raise SceneContractError('geometry name/owner_id не согласованы с object id')
    pose = _pose(part.get('pose'), f'{name}.pose')
    values: Dict[str, Any] = {}
    if kind == 'cuboid':
        values['dims'] = _finite_vector(part.get('dims'), 3, f'{name}.dims', positive=True)
    elif kind == 'sphere':
        values['radius'] = _finite_vector(
            [part.get('radius')], 1, f'{name}.radius', positive=True
        )[0]
    elif kind in {'cylinder', 'cone'}:
        values['radius'] = _finite_vector(
            [part.get('radius')], 1, f'{name}.radius', positive=True
        )[0]
        values['height'] = _finite_vector(
            [part.get('height')], 1, f'{name}.height', positive=True
        )[0]
        if kind == 'cone':
            vertices, faces = conservative_cone_mesh(
                values['radius'], values['height']
            )
            values['vertices'] = vertices
            values['faces'] = faces
    elif kind == 'mesh':
        raw_vertices = part.get('vertices')
        raw_faces = part.get('faces')
        if not isinstance(raw_vertices, Sequence) or not raw_vertices:
            raise SceneContractError(f'{name}.vertices пуст')
        vertices = tuple(
            _finite_vector(vertex, 3, f'{name}.vertices') for vertex in raw_vertices
        )
        if not isinstance(raw_faces, Sequence) or not raw_faces:
            raise SceneContractError(f'{name}.faces пуст')
        faces = []
        for face in raw_faces:
            if not isinstance(face, Sequence) or len(face) != 3:
                raise SceneContractError(f'{name}.faces должен содержать triangles')
            indices = tuple(int(index) for index in face)
            if any(index < 0 or index >= len(vertices) for index in indices):
                raise SceneContractError(f'{name}.faces ссылается на отсутствующую vertex')
            faces.append(indices)
        values['vertices'] = vertices
        values['faces'] = tuple(faces)
    else:
        raise UnsupportedSceneError(f'Неподдерживаемый geometry kind: {kind!r}')
    return GeometrySpec(name, owner_id, kind, pose, values)


def _object_geometry(value: Mapping[str, Any]) -> Tuple[str, Tuple[GeometrySpec, ...]]:
    object_id = str(value.get('id', '')).strip()
    if not object_id:
        raise SceneContractError('collision object id пуст')
    parts = value.get('parts')
    if not isinstance(parts, Sequence) or not parts:
        raise SceneContractError(f'collision object {object_id!r} не содержит geometry')
    geometry = tuple(_geometry(part, object_id) for part in parts)
    if len({part.name for part in geometry}) != len(geometry):
        raise SceneContractError(f'collision object {object_id!r} содержит duplicate parts')
    return object_id, geometry


def _validate_acm(value: Any) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SceneContractError('allowed_collision_matrix отсутствует')
    names = tuple(str(name).strip() for name in value.get('entry_names', ()))
    rows = value.get('entries', ())
    if (
        any(not name for name in names)
        or len(set(names)) != len(names)
        or not isinstance(rows, Sequence)
        or len(rows) != len(names)
    ):
        raise SceneContractError('AllowedCollisionMatrix должен быть square и unique')
    normalized_rows = []
    for row in rows:
        if not isinstance(row, Sequence) or len(row) != len(names):
            raise SceneContractError('AllowedCollisionMatrix row имеет неверную длину')
        normalized_rows.append(tuple(bool(enabled) for enabled in row))
    for row in range(len(names)):
        for column in range(len(names)):
            if normalized_rows[row][column] != normalized_rows[column][row]:
                raise SceneContractError('AllowedCollisionMatrix должен быть симметричным')
    default_names = tuple(
        str(name).strip() for name in value.get('default_entry_names', ())
    )
    defaults = tuple(bool(item) for item in value.get('default_entries', ()))
    if (
        len(default_names) != len(defaults)
        or any(not name for name in default_names)
        or len(set(default_names)) != len(default_names)
    ):
        raise SceneContractError('AllowedCollisionMatrix defaults недопустимы')
    return {
        'entry_names': names,
        'entries': tuple(normalized_rows),
        'default_entry_names': default_names,
        'default_entries': defaults,
    }


def load_scene_snapshot(request: Mapping[str, Any]) -> SceneSnapshot:
    """Validate schema v2 and return deterministic pure scene geometry."""
    if int(request.get('schema_version', 0)) != PRIVATE_SCHEMA_VERSION:
        raise SceneContractError('Неподдерживаемая версия private bridge schema')
    scene = request.get('scene')
    if not isinstance(scene, Mapping) or bool(scene.get('is_diff', True)):
        raise SceneContractError('Требуется полный non-diff PlanningScene')
    robot_state = scene.get('robot_state')
    if not isinstance(robot_state, Mapping) or bool(robot_state.get('is_diff', True)):
        raise SceneContractError('Требуется полный non-diff RobotState')
    state_names = tuple(robot_state.get('joint_names', ()))
    state_positions = tuple(robot_state.get('joint_positions', ()))
    if len(state_names) != len(state_positions):
        raise SceneContractError('RobotState joint names/positions имеют разную длину')

    planning_frame = str(scene.get('planning_frame', '')).strip()
    if not planning_frame:
        raise SceneContractError('PlanningScene planning_frame пуст')
    workspace_frame = str(request.get('workspace_frame', '')).strip()
    if not workspace_frame:
        raise SceneContractError('Request workspace_frame пуст')
    if workspace_frame != planning_frame:
        raise UnsupportedSceneError(
            'Request workspace_frame не совпадает с PlanningScene planning_frame'
        )
    frame_names = set()
    for transform in scene.get('frame_transforms', ()):
        if not isinstance(transform, Mapping):
            raise SceneContractError('frame transform должен быть mapping')
        parent = str(transform.get('parent_frame', '')).strip()
        child = str(transform.get('child_frame', '')).strip()
        if parent != planning_frame or not child or child in frame_names:
            raise SceneContractError('frame transforms не образуют unique planning-frame snapshot')
        _pose(transform.get('pose'), f'frame_transforms[{child}]')
        frame_names.add(child)
    if planning_frame not in frame_names:
        raise SceneContractError('frame transforms не содержат planning frame')

    world_geometry = []
    object_ids = set()
    part_names = set()
    for value in scene.get('world_objects', ()):
        if not isinstance(value, Mapping):
            raise SceneContractError('world object должен быть mapping')
        source_frame = str(value.get('source_frame', '')).strip()
        if source_frame not in frame_names:
            raise SceneContractError('world object ссылается на неизвестный source_frame')
        object_id, geometry = _object_geometry(value)
        if object_id in object_ids:
            raise SceneContractError('collision object ids должны быть unique')
        object_ids.add(object_id)
        world_geometry.extend(geometry)
        part_names.update(part.name for part in geometry)

    attached_geometry = []
    for value in scene.get('attached_objects', ()):
        if not isinstance(value, Mapping) or not isinstance(value.get('object'), Mapping):
            raise SceneContractError('attached object должен быть mapping')
        link_name = str(value.get('link_name', '')).strip()
        if not link_name or link_name not in frame_names:
            raise SceneContractError('attached object link_name отсутствует в frame snapshot')
        object_id, geometry = _object_geometry(value['object'])
        if object_id in object_ids:
            raise SceneContractError('world/attached object ids должны быть unique')
        object_ids.add(object_id)
        touch_links = tuple(str(link).strip() for link in value.get('touch_links', ()))
        if any(not link for link in touch_links):
            raise SceneContractError('attached touch_links содержит пустое имя')
        attached_geometry.append(
            AttachedGeometry(object_id, link_name, touch_links, geometry)
        )
        part_names.update(part.name for part in geometry)

    expected_part_count = len(world_geometry) + sum(
        len(item.parts) for item in attached_geometry
    )
    if len(part_names) != expected_part_count:
        raise SceneContractError('geometry part names должны быть globally unique')
    return SceneSnapshot(
        planning_frame=planning_frame,
        world=tuple(world_geometry),
        attached=tuple(attached_geometry),
        acm=_validate_acm(scene.get('allowed_collision_matrix')),
    )


def _acm_entry(acm: Mapping[str, Any], left: str, right: str) -> bool:
    names = acm['entry_names']
    if left in names and right in names:
        return bool(acm['entries'][names.index(left)][names.index(right)])
    defaults = dict(zip(acm['default_entry_names'], acm['default_entries']))
    return bool(defaults.get(left, False) or defaults.get(right, False))


def enabled_robot_self_collision_pairs(
    snapshot: SceneSnapshot, robot_link_names: Iterable[str]
) -> frozenset[Tuple[str, str]]:
    """Return the exact enabled robot-link pairs from the authoritative ACM."""
    robot_names = frozenset(str(link) for link in robot_link_names if str(link))
    names = snapshot.acm['entry_names']
    rows = snapshot.acm['entries']
    return frozenset(
        tuple(sorted((left, names[column])))
        for row, left in enumerate(names)
        for column in range(row + 1, len(names))
        if rows[row][column]
        and left in robot_names
        and names[column] in robot_names
    )


def globally_allowed_world_objects(
    snapshot: SceneSnapshot,
    robot_collision_links: Iterable[str],
    *,
    robot_link_names: Iterable[str] | None = None,
    supported_self_collision_pairs: Iterable[Tuple[str, str]] = (),
) -> frozenset[str]:
    """
    Return objects safely removable from world collision checking.

    cuRobo exposes whole-obstacle and whole-link toggles, not MoveIt's
    arbitrary link/object pair matrix. A world object is therefore disabled
    only when every collision link may touch it. A partial allowance is
    rejected instead of weakening or silently strengthening the scene.
    """
    links = tuple(dict.fromkeys(str(link) for link in robot_collision_links if link))
    robot_names = frozenset(
        str(link) for link in (robot_link_names or links) if str(link)
    )
    supported_self = frozenset(
        tuple(sorted((str(left), str(right))))
        for left, right in supported_self_collision_pairs
    )
    owners = tuple(dict.fromkeys(part.owner_id for part in snapshot.world))
    attached = {item.object_id: item for item in snapshot.attached}
    disabled = set()
    collision_entities = (*links, *attached)
    if collision_entities:
        for owner in owners:
            allowed = tuple(
                _acm_entry(snapshot.acm, owner, entity)
                for entity in collision_entities
            )
            if all(allowed):
                disabled.add(owner)
            elif any(allowed):
                raise UnsupportedSceneError(
                    f'cuRobo не поддерживает partial ACM для world object {owner!r}'
                )
    disabled_owners = frozenset(disabled)

    names = snapshot.acm['entry_names']
    rows = snapshot.acm['entries']
    world_owners = frozenset(owners)
    enabled_robot_self = set()
    for row, left in enumerate(names):
        for column in range(row + 1, len(names)):
            if not rows[row][column]:
                continue
            right = names[column]
            if left in world_owners or right in world_owners:
                owner = left if left in world_owners else right
                other = right if owner == left else left
                if owner in disabled_owners or other in world_owners:
                    continue
                raise UnsupportedSceneError(
                    f'cuRobo не поддерживает enabled ACM pair {left!r}/{right!r}'
                )
            if left in attached or right in attached:
                owner = left if left in attached else right
                other = right if owner == left else left
                attachment = attached[owner]
                if other in frozenset((*attachment.touch_links, attachment.link_name)):
                    continue
                raise UnsupportedSceneError(
                    f'ACM pair для attached object {owner!r} выходит за touch_links'
                )
            if left in robot_names and right in robot_names:
                pair = tuple(sorted((left, right)))
                enabled_robot_self.add(pair)
                if pair in supported_self:
                    continue
                raise UnsupportedSceneError(
                    f'Robot self-collision ACM pair {left!r}/{right!r} '
                    'не подтверждён cuRobo profile'
                )
            raise UnsupportedSceneError(
                f'ACM содержит enabled pair с неизвестной семантикой: {left!r}/{right!r}'
            )

    declared_robot_self = {
        pair for pair in supported_self
        if pair[0] in robot_names and pair[1] in robot_names
    }
    if declared_robot_self != enabled_robot_self:
        raise UnsupportedSceneError(
            'runtime cuRobo self-collision policy не совпадает с MoveIt ACM snapshot'
        )

    for name, enabled in zip(
        snapshot.acm['default_entry_names'], snapshot.acm['default_entries']
    ):
        if enabled and name not in disabled_owners:
            raise UnsupportedSceneError(
                f'cuRobo не поддерживает enabled ACM default для {name!r}'
            )
    return disabled_owners
