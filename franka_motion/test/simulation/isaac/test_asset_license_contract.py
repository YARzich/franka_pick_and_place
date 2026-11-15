"""
Test licenses and provenance boundaries for bundled Isaac assets.

Responsibility:
- Keep package metadata, redistributed Panda USD, and exclusion policy aligned.
"""

import hashlib
from pathlib import Path
import pickletools
import xml.etree.ElementTree as ET
import zipfile


_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
_ISAAC_WORLDS = _PACKAGE_ROOT / "worlds" / "isaac"


def test_package_declares_code_configuration_and_asset_licenses() -> None:
    """Package metadata must cover local code and redistributed resources."""
    package = ET.parse(_PACKAGE_ROOT / "package.xml").getroot()
    declared_licenses = {element.text for element in package.findall("license")}

    assert {"MIT", "Apache-2.0", "BSD", "AGPL-3.0"} <= declared_licenses
    assert (_PACKAGE_ROOT / "LICENSE").read_text(encoding="utf-8").startswith(
        "MIT License"
    )
    assert (
        _ISAAC_WORLDS / "FrankaPanda" / "franka-LICENSE.txt"
    ).read_text(encoding="utf-8").startswith("Apache License")

    notices = (_PACKAGE_ROOT / "THIRD_PARTY_NOTICES.md").read_text(
        encoding="utf-8"
    )
    assert "worlds/isaac/FrankaPanda" in notices
    assert "worlds/materials" in notices
    assert "config/moveit" in notices


def test_bundled_checkpoint_metadata_matches_license_inventory() -> None:
    """Check binary identities and markers without executing pickle objects."""
    models = _PACKAGE_ROOT / "models"
    notices = (models / "THIRD_PARTY_NOTICES.md").read_text(encoding="utf-8")
    license_text = (models / "LICENSES" / "AGPL-3.0.txt").read_text(
        encoding="utf-8"
    )
    assert "GNU AFFERO GENERAL PUBLIC LICENSE" in license_text
    checkpoints = sorted(models.glob("*.pt"))
    assert checkpoints, "Bundled checkpoints must have an explicit inventory"

    for checkpoint in checkpoints:
        assert checkpoint.name in notices
        assert hashlib.sha256(checkpoint.read_bytes()).hexdigest() in notices
        with zipfile.ZipFile(checkpoint) as archive:
            metadata_entries = [
                name for name in archive.namelist() if name.endswith("/data.pkl")
            ]
            assert len(metadata_entries) == 1
            metadata_strings = {
                argument
                for _, argument, _ in pickletools.genops(
                    archive.read(metadata_entries[0])
                )
                if isinstance(argument, str)
            }
        assert "AGPL-3.0 (https://ultralytics.com/license)" in metadata_strings
        assert "8.4.19" in metadata_strings


def test_moveit_license_retains_attribution_and_all_three_conditions() -> None:
    """Keep the release credit and complete terms together, without invented owners."""
    notices = (_PACKAGE_ROOT / "config/moveit/THIRD_PARTY_NOTICES.md").read_text(
        encoding="utf-8"
    )
    license_text = (_PACKAGE_ROOT / "LICENSES/BSD-3-Clause.txt").read_text(
        encoding="utf-8"
    )
    for attribution in (
        "Mike Lautman",
        "9e3c48401260a3d225903b901ecb596ce609219f",
        "https://moveit.ai/documentation/source-code-api/",
    ):
        assert attribution in notices
        assert attribution in license_text
    assert "Copyright (c) MoveIt Resources contributors." not in license_text
    for condition in (
        "1. Redistributions of source code must retain",
        "2. Redistributions in binary form must reproduce",
        "3. Neither the name of the copyright holder",
        'THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"',
    ):
        assert condition in license_text


def test_supported_assets_exclude_caches_and_unlicensed_texture_tree() -> None:
    """Canonical assets must not absorb local caches or historical textures."""
    assert not (_PACKAGE_ROOT / "worlds" / "materials").exists()
    assert not list(_ISAAC_WORLDS.rglob(".thumbs"))

    supported_roots = (
        _ISAAC_WORLDS / "table_world_usd_clean.usd",
        _ISAAC_WORLDS / "FrankaPanda" / "franka.usd",
    )
    for usd_root in supported_roots:
        payload = usd_root.read_bytes().lower()
        assert b"worlds/materials" not in payload
        assert b"materials/textures" not in payload


def test_default_table_scene_is_offline_while_legacy_scene_is_explicit() -> None:
    """The supported table scene must not require a remote NVIDIA asset."""
    clean_scene = (_ISAAC_WORLDS / "table_world_usd_clean.usd").read_bytes()
    legacy_scene = (_ISAAC_WORLDS / "table_world_usd.usd").read_bytes()

    assert b"https://" not in clean_scene
    assert b"https://" in legacy_scene
