"""
Repository-wide contract for explicit file-level responsibilities.

Responsibility:
- Enumerate project-owned source, test, interface, and commentable config files.
- Require a visible responsibility contract before implementation or data.
- Keep binary assets, generated state, licenses, and prose documentation out of scope.
"""

from __future__ import annotations

import ast
from pathlib import Path


_PACKAGE_FAMILY_ROOT = Path(__file__).resolve().parents[3]
_REPOSITORY_ROOT = (
    _PACKAGE_FAMILY_ROOT
    if (_PACKAGE_FAMILY_ROOT / 'SECURITY.md').is_file()
    else _PACKAGE_FAMILY_ROOT.parent
)
_EXCLUDED_PARTS = {
    ".git",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
}
_COMMENTABLE_SUFFIXES = {
    ".action",
    ".cfg",
    ".cmake",
    ".cpp",
    ".h",
    ".hpp",
    ".kit",
    ".msg",
    ".py",
    ".repos",
    ".rviz",
    ".sdf",
    ".srdf",
    ".srv",
    ".xacro",
    ".xml",
    ".yaml",
    ".yml",
}
_SPECIAL_FILENAMES = {
    ".setup_assistant",
    "CMakeLists.txt",
    "franka_runtime_harness",
}
_SPECIAL_RELATIVE_PATHS = {
    Path("CPPLINT.cfg"),
}


def _is_project_file(path: Path) -> bool:
    """Return whether path is an owned, commentable Franka project file."""
    if not path.is_file() or any(part in _EXCLUDED_PARTS for part in path.parts):
        return False
    try:
        relative_path = path.relative_to(_REPOSITORY_ROOT)
    except ValueError:
        return False
    if relative_path in _SPECIAL_RELATIVE_PATHS:
        return True
    if (
        relative_path.parts[:2] == (".github", "workflows")
        and path.name.startswith("franka-")
        and path.suffix in {".yaml", ".yml"}
    ):
        return True
    if not path.is_relative_to(_PACKAGE_FAMILY_ROOT):
        return False
    return path.suffix in _COMMENTABLE_SUFFIXES or path.name in _SPECIAL_FILENAMES


def _project_files() -> list[Path]:
    """Return deterministic paths covered by the responsibility contract."""
    return sorted(path for path in _REPOSITORY_ROOT.rglob("*") if _is_project_file(path))


def _python_responsibility(path: Path) -> str | None:
    """Return a Python module docstring containing an explicit responsibility."""
    module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    docstring = ast.get_docstring(module, clean=False)
    if docstring and "Responsibility:" in docstring:
        return docstring
    return None


def _visible_responsibility(path: Path) -> str | None:
    """Return the responsibility visible near the start of a non-Python file."""
    lines = path.read_text(encoding="utf-8").splitlines()
    header = "\n".join(lines[:16])
    return header if "Responsibility:" in header else None


def test_every_commentable_project_file_states_its_responsibility() -> None:
    """Readers must see each file's role before reading implementation details."""
    missing: list[str] = []
    project_files = _project_files()
    assert project_files, 'No source files collected for the responsibility contract'
    for path in project_files:
        responsibility = (
            _python_responsibility(path)
            if path.suffix == ".py" or path.name == "franka_runtime_harness"
            else _visible_responsibility(path)
        )
        if responsibility is None:
            missing.append(str(path.relative_to(_REPOSITORY_ROOT)))

    assert not missing, "Missing explicit file responsibility:\n" + "\n".join(missing)
