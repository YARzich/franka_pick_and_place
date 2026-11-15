#!/usr/bin/env python3
"""Responsibility: Bind saved training recipes to local paths without running training."""

from __future__ import annotations

import argparse
from pathlib import Path
import shlex
from string import Template

import yaml


BUNDLE_ROOT = Path(__file__).resolve().parents[1]


def bind_paths(value, roots: dict[str, str]):
    """Expand only explicit path placeholders and preserve every other setting."""
    if isinstance(value, str):
        return Template(value).substitute(roots)
    if isinstance(value, list):
        return [bind_paths(item, roots) for item in value]
    if isinstance(value, dict):
        return {key: bind_paths(item, roots) for key, item in value.items()}
    return value


def _check_inputs(recipe: dict, dataset: dict) -> None:
    """Require local input weights and every referenced train/validation source."""
    missing = []
    weights = Path(recipe['model'])
    if not weights.is_file():
        missing.append(weights)
    for split in ('train', 'val'):
        entries = dataset[split]
        if isinstance(entries, str):
            entries = [entries]
        for entry in entries:
            path = Path(entry)
            if not path.is_absolute():
                path = Path(dataset['path']) / path
            if not path.exists():
                missing.append(path)
    if missing:
        raise FileNotFoundError('Missing training inputs:\n' + '\n'.join(map(str, missing)))


def prepare_recipe(
    checkpoint: str,
    *,
    datasets_root: Path,
    weights_root: Path,
    runs_root: Path,
    output_dir: Path,
    check_inputs: bool = False,
    bundle_root: Path = BUNDLE_ROOT,
) -> Path:
    """Write a new local dataset/config pair; never overwrite recipes or run a tool."""
    inventory = yaml.safe_load((bundle_root / 'provenance.yaml').read_text(encoding='utf-8'))
    selected = next((run for run in inventory['runs'] if run['checkpoint'] == checkpoint), None)
    if selected is None:
        raise ValueError(f'Unknown checkpoint: {checkpoint}')
    roots = {
        'DATASETS_ROOT': str(datasets_root.expanduser().resolve()),
        'WEIGHTS_ROOT': str(weights_root.expanduser().resolve()),
        'RUNS_ROOT': str(runs_root.expanduser().resolve()),
    }
    recipe = bind_paths(
        yaml.safe_load((bundle_root / selected['recipe']).read_text(encoding='utf-8')), roots
    )
    dataset = bind_paths(
        yaml.safe_load((bundle_root / selected['dataset_manifest']).read_text(encoding='utf-8')),
        roots,
    )
    if recipe['mode'] != 'train' or recipe['resume'] is not False:
        raise ValueError('This helper prepares a fresh training run, not a resume or another mode')
    if check_inputs:
        _check_inputs(recipe, dataset)

    output_dir = output_dir.expanduser().resolve()
    dataset_path = output_dir / 'data.yaml'
    train_path = output_dir / 'train.yaml'
    recipe['data'] = str(dataset_path)
    # Do not reuse the historical derived output path for the newly selected run.
    # Keep save_dir in the archived recipe; project/name specify the new run here.
    recipe.pop('save_dir')
    output_dir.mkdir(parents=True, exist_ok=False)
    for path, data in ((dataset_path, dataset), (train_path, recipe)):
        path.write_text(
            '# Responsibility: Bind the saved recipe to explicitly selected local paths.\n'
            + yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
            encoding='utf-8',
        )
    return train_path


def main() -> int:
    """Prepare files and print the command for the separately chosen YOLO environment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', required=True, help='Bundled checkpoint basename, including .pt')
    parser.add_argument('--datasets-root', type=Path, required=True)
    parser.add_argument('--weights-root', type=Path, required=True)
    parser.add_argument('--runs-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True, help='New directory; existing paths are refused')
    parser.add_argument('--check-inputs', action='store_true', help='Require every input on this machine')
    args = parser.parse_args()
    try:
        config = prepare_recipe(**vars(args))
    except (OSError, KeyError, ValueError) as exc:
        parser.exit(1, f'{exc}\n')
    print(f'Prepared only; inspect {config} before training.')
    print(shlex.join(['yolo', 'train', f'cfg={config}']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
