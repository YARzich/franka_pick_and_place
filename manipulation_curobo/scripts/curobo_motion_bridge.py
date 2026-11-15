#!/usr/bin/env python3
"""
CLI transport for one MoveIt plugin request and one cuRobo response.

Responsibility:
    Parse one file-bridge invocation, delegate planning to MotionGen, and return
    a deterministic response file and process exit status to the plugin.

Does not own:
    Persistent ROS transport or controller execution.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any, Dict

import yaml

from manipulation_curobo_runtime.scene_conversion import (
    SceneContractError,
    UnsupportedSceneError,
    load_scene_snapshot,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='cuRobo MotionGen CLI bridge for MoveIt.')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        '--input', type=Path, help='YAML-файл с запросом.'
    )
    source.add_argument(
        '--probe-config', type=Path,
        help='Validate imports/config/CUDA without MotionGen construction.',
    )
    parser.add_argument(
        '--output', required=True, type=Path, help='YAML-файл для результата.'
    )
    return parser.parse_args()


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _dump_yaml(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream)


def main() -> None:
    """Execute exactly one file-based planning request."""
    args = _parse_args()
    if args.probe_config is not None:
        try:
            # Lazy import lets the probe persist a useful diagnostic even when
            # torch or cuRobo itself is absent from the selected interpreter.
            from manipulation_curobo_runtime.motion_gen_bridge import probe_backend

            details = probe_backend(args.probe_config)
            _dump_yaml(
                args.output,
                {'success': True, 'message': '', **details},
            )
        except Exception as exception:  # noqa: BLE001 - startup probe boundary
            _dump_yaml(
                args.output,
                {
                    'success': False,
                    'message': f'cuRobo readiness probe failed: {exception}',
                },
            )
        return

    try:
        from manipulation_curobo_runtime.motion_gen_bridge import (
            build_motion_gen,
            build_world,
            load_bridge_config,
            plan_motion,
        )
        request_data = _load_yaml(args.input)
        motion_gen_config = Path(request_data['motion_gen_config_file'])
        bridge_config = load_bridge_config(motion_gen_config, request_data)
        scene_snapshot = load_scene_snapshot(request_data)
        motion_gen = build_motion_gen(
            bridge_config, build_world(scene_snapshot), scene_snapshot
        )
        result = plan_motion(
            motion_gen, request_data, bridge_config, scene_snapshot
        )
    except UnsupportedSceneError as exception:
        _dump_yaml(
            args.output,
            {
                'success': False,
                'error_code': 'unsupported_scene',
                'message': str(exception),
                'planning_time': 0.0,
            },
        )
        return
    except (SceneContractError, KeyError, TypeError, ValueError) as exception:
        _dump_yaml(
            args.output,
            {
                'success': False,
                'error_code': 'invalid_request',
                'message': str(exception),
                'planning_time': 0.0,
            },
        )
        return
    except Exception as exception:  # noqa: BLE001 - persist boundary errors for C++
        _dump_yaml(
            args.output,
            {
                'success': False,
                'error_code': 'internal_error',
                'message': f'cuRobo bridge error: {exception}',
                'planning_time': 0.0,
            },
        )
        print(f'[curobo_bridge] {exception}', file=sys.stderr)
        return

    _dump_yaml(args.output, result)


if __name__ == '__main__':
    main()
