"""
Architecture checks for the robot-independent perception package.

Responsibility: Prevent scenario assets, legacy imports, and robot-specific
defaults from leaking back into reusable production code.
"""

import ast
from pathlib import Path
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_ROOTS = (
    PACKAGE_ROOT / "manipulation_perception",
    PACKAGE_ROOT / "scripts",
)
FORBIDDEN_IMPORT_PREFIXES = (
    "franka_motion",
    "franka_motion_perception",
)
FORBIDDEN_LITERAL_FRAGMENTS = (
    "/table_stereo",
    "panda_link",
    "franka_cube",
    "franka_motion_",
    "cube_5cm",
    "/home/yaroslav",
)


def _python_sources():
    for root in PRODUCTION_ROOTS:
        yield from root.rglob("*.py")


def test_manifest_has_no_robot_specific_dependency():
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {
        element.text or ""
        for tag in ("depend", "build_depend", "exec_depend", "test_depend")
        for element in manifest.findall(tag)
    }
    assert not any(
        dependency.startswith("franka_motion") for dependency in dependencies
    )


def test_production_code_imports_only_the_new_perception_namespace():
    for source_path in _python_sources():
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
            elif isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
        assert not any(
            imported.startswith(FORBIDDEN_IMPORT_PREFIXES)
            for imported in imports
        ), source_path


def test_production_defaults_contain_no_scenario_identity():
    for source_path in _python_sources():
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        string_literals = (
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        )
        for value in string_literals:
            assert not any(
                fragment in value for fragment in FORBIDDEN_LITERAL_FRAGMENTS
            ), (source_path, value)


def test_package_contains_only_generic_schemas_and_no_scenario_resources():
    config_files = {
        path.relative_to(PACKAGE_ROOT / "config").as_posix()
        for path in (PACKAGE_ROOT / "config").rglob("*")
        if path.is_file()
    }
    assert config_files == {
        "gpu-perception-evidence.schema.v1.json",
        "gpu_perception_probe_contract.schema.v1.json",
    }
    assert not (PACKAGE_ROOT / "models").exists()
    assert not (PACKAGE_ROOT / "scripts" / "tools").exists()
    assert not list(PACKAGE_ROOT.rglob("*.pt"))
    assert not list(PACKAGE_ROOT.rglob("*.obj"))


def test_every_production_python_file_declares_responsibility():
    for source_path in _python_sources():
        module_docstring = ast.get_docstring(
            ast.parse(source_path.read_text(encoding="utf-8"))
        ) or ""
        assert "Responsibility" in module_docstring, source_path


def test_bbox_modules_keep_dependency_direction_toward_geometry():
    common_root = PACKAGE_ROOT / "manipulation_perception" / "common"
    imported_modules = {}
    for name in ("bbox_geometry", "bbox_wire", "bbox_mask"):
        source_path = common_root / f"{name}.py"
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        imported_modules[name] = {
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        }

    bbox_prefix = "manipulation_perception.common.bbox_"
    assert not any(
        module.startswith(bbox_prefix)
        for module in imported_modules["bbox_geometry"]
    )
    assert "manipulation_perception.common.bbox_geometry" in imported_modules["bbox_wire"]
    assert "manipulation_perception.common.bbox_geometry" in imported_modules["bbox_mask"]


def test_production_consumers_import_bbox_helpers_from_owner_modules():
    compatibility_facade = "manipulation_perception.common.bbox"
    for source_path in _python_sources():
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        facade_imports = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module == compatibility_facade
        ]
        assert not facade_imports, source_path
