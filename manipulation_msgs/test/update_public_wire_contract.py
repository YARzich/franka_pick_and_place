"""
Explicitly check or update the reviewed current wire-contract snapshot.

Responsibility: Guard explicit V2 updates while preserving the V1 archive.
"""

import argparse
import hashlib
from pathlib import Path

from test_public_wire_contract import CURRENT_SNAPSHOT_PATH
from test_public_wire_contract import HISTORICAL_SNAPSHOT_PATH
from test_public_wire_contract import HISTORICAL_SNAPSHOT_SHA256
from test_public_wire_contract import json_text
from test_public_wire_contract import REMOVED_HISTORICAL_TYPES
from test_public_wire_contract import source_contract


def _parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Check the V2 snapshot, or update it after explicit review.'
        )
    )
    parser.add_argument(
        '--write',
        action='store_true',
        help=(
            'atomically replace the V2 snapshot with the current source '
            'contract'
        ),
    )
    parser.add_argument(
        '--acknowledge-reviewed-public-change',
        action='store_true',
        help='required together with --write; confirms change-control review',
    )
    arguments = parser.parse_args()
    if arguments.write != arguments.acknowledge_reviewed_public_change:
        parser.error(
            '--write and --acknowledge-reviewed-public-change must be used '
            'together'
        )
    return arguments


def _validate_historical_archive() -> None:
    digest = hashlib.sha256(HISTORICAL_SNAPSHOT_PATH.read_bytes()).hexdigest()
    if digest != HISTORICAL_SNAPSHOT_SHA256:
        raise SystemExit(
            'Refusing update: the immutable V1 archive hash changed'
        )


def _candidate_bytes() -> bytes:
    candidate = source_contract()
    type_names = {interface['type'] for interface in candidate['interfaces']}
    restored_types = sorted(type_names & REMOVED_HISTORICAL_TYPES)
    if restored_types:
        raise SystemExit(
            'Refusing update: removed historical types are present: '
            + ', '.join(restored_types)
        )
    return json_text(candidate).encode('utf-8')


def _write_atomically(path: Path, content: bytes) -> None:
    temporary_path = path.with_suffix(path.suffix + '.tmp')
    temporary_path.write_bytes(content)
    temporary_path.replace(path)


def main() -> int:
    """Check by default or update after explicit acknowledgement."""
    arguments = _parse_arguments()
    _validate_historical_archive()
    candidate = _candidate_bytes()
    existing = (
        CURRENT_SNAPSHOT_PATH.read_bytes()
        if CURRENT_SNAPSHOT_PATH.exists()
        else b''
    )
    if existing == candidate:
        print(f'Current snapshot is up to date: {CURRENT_SNAPSHOT_PATH}')
        return 0
    if not arguments.write:
        print(
            'Current snapshot differs. Review the public change, then run:\n'
            '  python3 test/update_public_wire_contract.py --write '
            '--acknowledge-reviewed-public-change'
        )
        return 1
    _write_atomically(CURRENT_SNAPSHOT_PATH, candidate)
    print(f'Updated reviewed snapshot: {CURRENT_SNAPSHOT_PATH}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
