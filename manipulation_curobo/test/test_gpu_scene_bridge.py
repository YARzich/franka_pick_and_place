"""
Opt-in real-cuRobo smoke test; never downloads models or SDKs.

Responsibility: Exercise representative scene planning with caller-owned assets.
"""

import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


pytestmark = pytest.mark.gpu
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_TIMEOUT_SEC = 300


@pytest.mark.skipif(
    os.environ.get('MANIPULATION_CUROBO_RUN_GPU') != '1',
    reason='set MANIPULATION_CUROBO_RUN_GPU=1 explicitly',
)
def test_real_bridge_plans_request_with_world_and_attachment(tmp_path):
    """Run only against a caller-supplied representative immutable scene."""
    request_path = Path(
        os.environ.get('MANIPULATION_CUROBO_GPU_REQUEST', '')
    ).expanduser()
    if not request_path.is_file():
        pytest.fail('MANIPULATION_CUROBO_GPU_REQUEST must name an existing YAML file')
    request = yaml.safe_load(request_path.read_text(encoding='utf-8'))
    assert request['scene']['world_objects']
    assert request['scene']['attached_objects']
    response_path = tmp_path / 'response.yaml'

    subprocess.run(
        [
            sys.executable,
            str(PACKAGE_ROOT / 'scripts' / 'curobo_motion_bridge.py'),
            '--input',
            str(request_path),
            '--output',
            str(response_path),
        ],
        check=True,
        # A clean machine may JIT-build cuRobo CUDA extensions on the first run.
        timeout=BRIDGE_TIMEOUT_SEC,
    )

    response = yaml.safe_load(response_path.read_text(encoding='utf-8'))
    assert response['success'], response
    assert response['trajectory']['points']
