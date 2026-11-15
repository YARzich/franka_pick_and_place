#!/usr/bin/env python3
"""Responsibility: Run the family's source-only architecture checks without a build.

Installed interfaces, installed package surfaces and runtime evidence remain
part of the package and scenario checks. This command never builds or runs ROS.
"""

import argparse
import os
from pathlib import Path
import sys


FAMILY_ROOT = Path(__file__).resolve().parents[1]
CHECKS = (
    'franka_motion/test/architecture/test_active_cancel_architecture.py',
    'franka_motion/test/architecture/test_file_responsibility_contract.py',
    'franka_motion/test/architecture/test_frame_validation_architecture.py',
    'franka_motion/test/architecture/test_package_dependency_architecture.py',
    'franka_motion/test/architecture/test_pose_header_architecture.py',
    'manipulation/test/test_architecture_boundaries.py',
    'manipulation_moveit/test/test_generic_boundary.py',
    'manipulation_curobo/test/test_generic_boundary.py',
    'manipulation_perception/test/test_generic_boundary.py',
    'manipulation_perception_isaac_ros/test/test_dependency_contract.py',
)
INSTALLED_CHECKS = (
    'manipulation_moveit/test/test_generic_boundary.py::'
    'test_incremental_install_has_no_v1_backend_artifacts',
    'manipulation_curobo/test/test_generic_boundary.py::'
    'test_installed_surface_has_no_stale_package_action',
)


class RequiredModules:
    """Require a nonempty collection from every explicitly selected test file."""

    def __init__(self, paths):
        self.paths = set(paths)

    def pytest_collection_finish(self, session):
        import pytest

        collected = {Path(item.path).resolve() for item in session.items}
        empty = self.paths - collected
        if empty:
            raise pytest.UsageError(
                'No tests collected from required architecture modules:\n'
                + '\n'.join(str(path) for path in sorted(empty))
            )


def run_checks(family_root, checks=CHECKS, installed_checks=INSTALLED_CHECKS):
    """Run the selected source checks and preserve pytest failures as exit codes."""
    family_root = Path(family_root).resolve()
    paths = [(family_root / check).resolve() for check in checks]
    missing = [path for path in paths if not path.is_file()]
    if not paths or missing:
        print('Required architecture test files are missing or the list is empty:',
              file=sys.stderr)
        for path in missing:
            print(path, file=sys.stderr)
        return 4

    # Ambient selection flags or plugins must not silently narrow this gate.
    os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
    os.environ.pop('PYTEST_ADDOPTS', None)
    os.environ.pop('PYTEST_PLUGINS', None)
    sys.dont_write_bytecode = True
    import pytest

    print('Source-only architecture checks; installed/wire and runtime checks '
          'remain in the package/scenario suites.', flush=True)
    args = [
        '-q', '-c', os.devnull, '--rootdir', str(family_root),
        '--noconftest', '--import-mode=importlib', '-p', 'no:cacheprovider',
        *(f'--deselect={nodeid}' for nodeid in installed_checks),
        *(str(path) for path in paths),
    ]
    return int(pytest.main(args, plugins=[RequiredModules(paths)]))


if __name__ == '__main__':
    argparse.ArgumentParser(description=__doc__).parse_args()
    raise SystemExit(run_checks(FAMILY_ROOT))
