"""
Protect the current wire contract and preserve the immutable V1 archive.

Responsibility:
- compare every current source definition with the reviewed V2 snapshot;
- verify that generated Python classes preserve ordered fields and constants;
- keep the pre-migration V1 snapshot byte-exact and explicitly archival.
"""

import difflib
import hashlib
from importlib import import_module
from importlib.util import find_spec
import json
import math
from pathlib import Path
from typing import Any

from rosidl_adapter.msg import MSG_TYPE_TO_IDL
from rosidl_adapter.parser import parse_action_file
from rosidl_adapter.parser import parse_message_file
from rosidl_adapter.parser import parse_service_file

from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import BoundedString
from rosidl_parser.definition import BoundedWString
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import UnboundedSequence
from rosidl_parser.definition import UnboundedString
from rosidl_parser.definition import UnboundedWString

from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.utilities import get_service


PACKAGE_NAME = 'manipulation_msgs'
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
HISTORICAL_SNAPSHOT_PATH = (
    PACKAGE_ROOT / 'docs' / 'public_wire_contract.v1.json'
)
CURRENT_SNAPSHOT_PATH = PACKAGE_ROOT / 'docs' / 'public_wire_contract.v2.json'
HISTORICAL_SNAPSHOT_SHA256 = (
    '3409d8201adf5f778f9293910b14404e988663a7cba6a113df91ea3aa120f8e9'
)
REMOVED_HISTORICAL_TYPES = frozenset(
    {
        f'{PACKAGE_NAME}/action/CuRoboPlanPose',
        f'{PACKAGE_NAME}/action/PerceptionResolvedTransferSequence',
        f'{PACKAGE_NAME}/action/TransferObject',
        f'{PACKAGE_NAME}/action/TransferObjectSequence',
        f'{PACKAGE_NAME}/msg/PerceptionResolvedTransferStep',
        f'{PACKAGE_NAME}/msg/StaticBoxTransferStep',
    }
)
REMOVED_GENERATED_MODULES = frozenset(
    {
        f'{PACKAGE_NAME}.action._cu_robo_plan_pose',
        f'{PACKAGE_NAME}.action._perception_resolved_transfer_sequence',
        f'{PACKAGE_NAME}.action._transfer_object',
        f'{PACKAGE_NAME}.action._transfer_object_sequence',
        f'{PACKAGE_NAME}.msg._perception_resolved_transfer_step',
        f'{PACKAGE_NAME}.msg._static_box_transfer_step',
    }
)


def _normalize_value(value: Any) -> Any:
    """Return a portable JSON value, including explicit non-finite floats."""
    if isinstance(value, float) and not math.isfinite(value):
        if math.isnan(value):
            label = 'nan'
        elif value > 0.0:
            label = 'positive_infinity'
        else:
            label = 'negative_infinity'
        return {'kind': 'non_finite_float', 'value': label}
    if isinstance(value, (list, tuple)):
        return [_normalize_value(item) for item in value]
    if hasattr(value, 'tolist'):
        return _normalize_value(value.tolist())
    return value


def _source_value_type(type_spec: Any) -> dict[str, Any]:
    """Convert a Jazzy source-parser type to the neutral snapshot shape."""
    if type_spec.pkg_name is not None:
        value_type = {
            'kind': 'namespaced',
            'namespaces': [type_spec.pkg_name, 'msg'],
            'name': type_spec.type,
        }
    elif type_spec.type in ('string', 'wstring'):
        value_type = {
            'kind': 'string',
            'wide': type_spec.type == 'wstring',
            'maximum_size': type_spec.string_upper_bound,
        }
    else:
        try:
            idl_name = MSG_TYPE_TO_IDL[type_spec.type]
        except KeyError as error:
            raise AssertionError(
                f'Unsupported source primitive type: {type_spec.type}'
            ) from error
        value_type = {'kind': 'basic', 'name': idl_name}

    if not type_spec.is_array:
        return value_type
    if type_spec.array_size is None:
        return {
            'kind': 'sequence',
            'maximum_size': None,
            'value_type': value_type,
        }
    if type_spec.is_upper_bound:
        return {
            'kind': 'sequence',
            'maximum_size': type_spec.array_size,
            'value_type': value_type,
        }
    return {
        'kind': 'array',
        'size': type_spec.array_size,
        'value_type': value_type,
    }


def _generated_value_type(type_spec: Any) -> dict[str, Any]:
    """Convert generated ``SLOT_TYPES`` metadata to the snapshot shape."""
    if isinstance(type_spec, BasicType):
        return {'kind': 'basic', 'name': type_spec.typename}
    if isinstance(type_spec, NamespacedType):
        return {
            'kind': 'namespaced',
            'namespaces': list(type_spec.namespaces),
            'name': type_spec.name,
        }
    if isinstance(type_spec, (BoundedString, UnboundedString)):
        maximum_size = (
            type_spec.maximum_size
            if isinstance(type_spec, BoundedString)
            else None
        )
        return {'kind': 'string', 'wide': False, 'maximum_size': maximum_size}
    if isinstance(type_spec, (BoundedWString, UnboundedWString)):
        maximum_size = (
            type_spec.maximum_size
            if isinstance(type_spec, BoundedWString)
            else None
        )
        return {'kind': 'string', 'wide': True, 'maximum_size': maximum_size}
    if isinstance(type_spec, Array):
        return {
            'kind': 'array',
            'size': type_spec.size,
            'value_type': _generated_value_type(type_spec.value_type),
        }
    if isinstance(type_spec, (BoundedSequence, UnboundedSequence)):
        maximum_size = (
            type_spec.maximum_size
            if isinstance(type_spec, BoundedSequence)
            else None
        )
        return {
            'kind': 'sequence',
            'maximum_size': maximum_size,
            'value_type': _generated_value_type(type_spec.value_type),
        }
    raise AssertionError(
        f'Unsupported generated type: {type(type_spec).__name__}'
    )


def _source_section(name: str, specification: Any) -> dict[str, Any]:
    """Serialize a message-like section while preserving declaration order."""
    constants = [
        {
            'name': constant.name,
            'type': {
                'kind': 'basic',
                'name': MSG_TYPE_TO_IDL[constant.type],
            },
            'value': _normalize_value(constant.value),
        }
        for constant in specification.constants
    ]
    fields = []
    for field in specification.fields:
        field_contract = {
            'name': field.name,
            'type': _source_value_type(field.type),
        }
        if field.default_value is not None:
            field_contract['default'] = _normalize_value(field.default_value)
        fields.append(field_contract)
    return {'name': name, 'constants': constants, 'fields': fields}


def _source_interfaces() -> list[dict[str, Any]]:
    """Build the complete canonical contract from authored interface files."""
    interfaces = []

    for path in sorted((PACKAGE_ROOT / 'msg').glob('*.msg')):
        specification = parse_message_file(PACKAGE_NAME, path)
        interfaces.append(
            {
                'type': f'{PACKAGE_NAME}/msg/{path.stem}',
                'kind': 'msg',
                'sections': [_source_section('message', specification)],
            }
        )

    for path in sorted((PACKAGE_ROOT / 'srv').glob('*.srv')):
        specification = parse_service_file(PACKAGE_NAME, path)
        interfaces.append(
            {
                'type': f'{PACKAGE_NAME}/srv/{path.stem}',
                'kind': 'srv',
                'sections': [
                    _source_section('request', specification.request),
                    _source_section('response', specification.response),
                ],
            }
        )

    for path in sorted((PACKAGE_ROOT / 'action').glob('*.action')):
        specification = parse_action_file(PACKAGE_NAME, path)
        interfaces.append(
            {
                'type': f'{PACKAGE_NAME}/action/{path.stem}',
                'kind': 'action',
                'sections': [
                    _source_section('goal', specification.goal),
                    _source_section('result', specification.result),
                    _source_section('feedback', specification.feedback),
                ],
            }
        )

    return sorted(interfaces, key=lambda interface: interface['type'])


def source_contract() -> dict[str, Any]:
    """Return the deterministic current source contract document."""
    return {
        'snapshot_schema_version': 1,
        'package': PACKAGE_NAME,
        'interfaces': _source_interfaces(),
    }


def load_snapshot(path: Path) -> dict[str, Any]:
    """Load one reviewed snapshot committed next to the package sources."""
    with path.open(encoding='utf-8') as stream:
        return json.load(stream)


def json_text(value: Any) -> str:
    """Format contract values deterministically for snapshots and diffs."""
    return json.dumps(value, indent=2, ensure_ascii=True) + '\n'


def _assert_contract_equal(expected: Any, actual: Any, context: str) -> None:
    """Raise one readable unified diff for a contract mismatch."""
    if expected == actual:
        return
    difference = ''.join(
        difflib.unified_diff(
            json_text(expected).splitlines(keepends=True),
            json_text(actual).splitlines(keepends=True),
            fromfile='reviewed_snapshot',
            tofile='current_contract',
        )
    )
    raise AssertionError(f'{context}\n{difference}')


def _generated_section(name: str, message_type: Any) -> dict[str, Any]:
    """Read one generated message class without relying on display strings."""
    field_names = list(message_type.get_fields_and_field_types())
    slot_types = list(message_type.SLOT_TYPES)
    if len(field_names) != len(slot_types):
        raise AssertionError(
            f'{message_type.__name__}: field names and SLOT_TYPES '
            'have different lengths'
        )

    constants = [
        {'name': constant_name, 'value': _normalize_value(value)}
        for constant_name, value in vars(message_type).items()
        if constant_name.isupper() and constant_name != 'SLOT_TYPES'
    ]
    fields = [
        {'name': field_name, 'type': _generated_value_type(slot_type)}
        for field_name, slot_type in zip(field_names, slot_types)
    ]
    return {'name': name, 'constants': constants, 'fields': fields}


def _generated_sections(interface: dict[str, Any]) -> list[dict[str, Any]]:
    """Resolve generated message classes for one current public interface."""
    type_name = interface['type']
    if interface['kind'] == 'msg':
        message_type = get_message(type_name)
        return [_generated_section('message', message_type)]
    if interface['kind'] == 'srv':
        service_type = get_service(type_name)
        return [
            _generated_section('request', service_type.Request),
            _generated_section('response', service_type.Response),
        ]
    if interface['kind'] == 'action':
        action_type = get_action(type_name)
        return [
            _generated_section('goal', action_type.Goal),
            _generated_section('result', action_type.Result),
            _generated_section('feedback', action_type.Feedback),
        ]
    raise AssertionError(f"Unsupported interface kind: {interface['kind']}")


def _generated_expectation(section: dict[str, Any]) -> dict[str, Any]:
    """Select snapshot data that generated Python metadata can represent."""
    return {
        'name': section['name'],
        'constants': [
            {'name': constant['name'], 'value': constant['value']}
            for constant in section['constants']
        ],
        'fields': [
            {'name': field['name'], 'type': field['type']}
            for field in section['fields']
        ],
    }


def test_historical_v1_snapshot_is_byte_exact_archive() -> None:
    """Keep pre-migration evidence immutable after source types are removed."""
    digest = hashlib.sha256(HISTORICAL_SNAPSHOT_PATH.read_bytes()).hexdigest()
    assert digest == HISTORICAL_SNAPSHOT_SHA256

    historical = load_snapshot(HISTORICAL_SNAPSHOT_PATH)
    assert historical['snapshot_schema_version'] == 1
    assert historical['package'] == PACKAGE_NAME
    type_names = [interface['type'] for interface in historical['interfaces']]
    assert len(type_names) == len(set(type_names))
    assert REMOVED_HISTORICAL_TYPES <= set(type_names)


def test_current_source_contract_matches_v2_snapshot() -> None:
    """Protect exact current inventory, fields, types, and defaults."""
    _assert_contract_equal(
        load_snapshot(CURRENT_SNAPSHOT_PATH),
        source_contract(),
        'Current public source wire contract changed',
    )


def test_removed_historical_types_are_archive_only() -> None:
    """Prevent removed contracts and stale modules from surviving upgrades."""
    current_snapshot = load_snapshot(CURRENT_SNAPSHOT_PATH)
    current_types = {
        interface['type'] for interface in current_snapshot['interfaces']
    }
    assert REMOVED_HISTORICAL_TYPES.isdisjoint(current_types)
    for type_name in REMOVED_HISTORICAL_TYPES:
        _, kind, generated_name = type_name.split('/')
        namespace = import_module(f'{PACKAGE_NAME}.{kind}')
        assert not hasattr(namespace, generated_name)
    for module_name in REMOVED_GENERATED_MODULES:
        assert find_spec(module_name) is None


def test_preserved_contracts_match_the_v1_archive() -> None:
    """Prove retained contracts kept their exact shape across migration."""
    historical = load_snapshot(HISTORICAL_SNAPSHOT_PATH)
    current = load_snapshot(CURRENT_SNAPSHOT_PATH)
    historical_by_type = {
        interface['type']: interface for interface in historical['interfaces']
    }
    current_by_type = {
        interface['type']: interface for interface in current['interfaces']
    }
    preserved_types = sorted(
        set(historical_by_type) - REMOVED_HISTORICAL_TYPES
    )
    assert preserved_types
    for type_name in preserved_types:
        assert type_name in current_by_type
        _assert_contract_equal(
            historical_by_type[type_name],
            current_by_type[type_name],
            f'Preserved contract changed across migration: {type_name}',
        )


def test_generated_python_matches_current_v2_snapshot() -> None:
    """Verify code generation preserves every current authored interface."""
    for interface in load_snapshot(CURRENT_SNAPSHOT_PATH)['interfaces']:
        expected = [
            _generated_expectation(section)
            for section in interface['sections']
        ]
        actual = _generated_sections(interface)
        _assert_contract_equal(
            expected,
            actual,
            f"Generated contract changed for {interface['type']}",
        )
