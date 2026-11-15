"""Responsibility: Verify the architecture command cannot silently skip checks."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT_DIRECTORY = Path(__file__).resolve().parents[1]


def _run(tmp_path, sources, *, missing=(), environment=None):
    """Exercise the real runner and pytest in a fresh child interpreter."""
    for relative_path, source in sources.items():
        path = tmp_path / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding='utf-8')
    checks = [*sources, *missing]
    program = (
        'import sys; '
        f'sys.path.insert(0, {str(SCRIPT_DIRECTORY)!r}); '
        'from check_architecture import run_checks; '
        f'sys.exit(run_checks({str(tmp_path)!r}, {checks!r}, ()))'
    )
    env = dict(os.environ)
    env.update(environment or {})
    return subprocess.run(
        [sys.executable, '-B', '-c', program],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )


def test_same_named_modules_from_multiple_packages_are_both_run(tmp_path):
    result = _run(tmp_path, {
        'first/test_boundary.py': 'def test_first(): assert True\n',
        'second/test_boundary.py': 'def test_second(): assert True\n',
    })
    assert result.returncode == 0, result.stdout + result.stderr
    assert '2 passed' in result.stdout


def test_missing_required_module_fails_even_with_another_valid_module(tmp_path):
    result = _run(
        tmp_path, {'test_ok.py': 'def test_ok(): assert True\n'},
        missing=('test_missing.py',),
    )
    assert result.returncode != 0
    assert 'test_missing.py' in result.stderr


@pytest.mark.parametrize('sources', [
    {},
    {'test_empty.py': '# no checks\n'},
    {'test_ok.py': 'def test_ok(): assert True\n', 'test_empty.py': '# no checks\n'},
])
def test_empty_selection_or_required_module_fails(tmp_path, sources):
    result = _run(tmp_path, sources)
    assert result.returncode != 0


@pytest.mark.parametrize('source', [
    'def test_broken(): assert False\n',
    'import nonexistent_architecture_test_dependency\n',
])
def test_assertion_and_collection_import_errors_fail(tmp_path, source):
    result = _run(tmp_path, {'test_broken.py': source})
    assert result.returncode != 0


def test_ambient_selection_plugins_and_conftest_cannot_narrow_checks(tmp_path):
    (tmp_path / 'conftest.py').write_text('raise RuntimeError("ambient")\n')
    result = _run(
        tmp_path, {'test_ok.py': 'def test_ok(): assert True\n'},
        environment={
            'PYTEST_ADDOPTS': '-k never_selected',
            'PYTEST_PLUGINS': 'nonexistent_ambient_plugin',
        },
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert '1 passed' in result.stdout
