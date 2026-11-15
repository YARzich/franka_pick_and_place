"""
Guard the first repository-wide versioning and security governance contract.

Responsibility: Keep canonical release artifacts and policy disclaimers explicit.
"""

from datetime import date
from pathlib import Path
import re
from xml.etree import ElementTree

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import pytest


_PACKAGE_FAMILY_ROOT = Path(__file__).resolve().parents[3]
_REPOSITORY_ROOT = (
    _PACKAGE_FAMILY_ROOT
    if (_PACKAGE_FAMILY_ROOT / 'SECURITY.md').is_file()
    else _PACKAGE_FAMILY_ROOT.parent
)
_CANONICAL_PACKAGES = (
    'franka_motion',
    'manipulation',
    'manipulation_curobo',
    'manipulation_moveit',
    'manipulation_msgs',
    'manipulation_perception',
    'manipulation_perception_isaac_ros',
)
_SEMVER_PATTERN = re.compile(
    r'^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)'
    r'(?:-(?:0|[1-9]\d*|\d*[A-Za-z-][0-9A-Za-z-]*)'
    r'(?:\.(?:0|[1-9]\d*|\d*[A-Za-z-][0-9A-Za-z-]*))*)?'
    r'(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?$'
)


def _package_version(package_name: str) -> str:
    """Read one package version from its manifest."""
    root = ElementTree.parse(
        _PACKAGE_FAMILY_ROOT / package_name / 'package.xml'
    ).getroot()
    version = root.findtext('version')
    assert version is not None
    return version.strip()


def _normalized_text(path: Path) -> str:
    """Read prose while ignoring presentation-only line wrapping."""
    return ' '.join(path.read_text(encoding='utf-8').lower().split())


def _assert_current_release_heading(changelog: str, version: str) -> None:
    """Require one conventional release heading with a valid ISO date."""
    heading_pattern = re.compile(
        rf'^{re.escape(version)} \((\d{{4}}-\d{{2}}-\d{{2}})\)\n(-+)$',
        flags=re.MULTILINE,
    )
    matches = heading_pattern.findall(changelog)
    assert len(matches) == 1, (version, matches)
    release_date, underline = matches[0]
    heading = f'{version} ({release_date})'
    assert len(underline) >= len(heading), (heading, underline)
    assert date.fromisoformat(release_date).isoformat() == release_date


def test_canonical_packages_use_their_reviewed_semantic_versions() -> None:
    """Allow an interface package to version independently after a breaking migration."""
    expected_versions = {
        package_name: '0.1.0' for package_name in _CANONICAL_PACKAGES
    }
    expected_versions.update(
        {
            'franka_motion': '0.2.0',
            'manipulation': '0.2.0',
            'manipulation_curobo': '0.2.0',
            'manipulation_moveit': '0.2.0',
            'manipulation_msgs': '0.3.0',
        }
    )
    for package_name in _CANONICAL_PACKAGES:
        version = _package_version(package_name)
        assert _SEMVER_PATTERN.fullmatch(version), (package_name, version)
        assert version == expected_versions[package_name], (package_name, version)


@pytest.mark.parametrize('package_name', _CANONICAL_PACKAGES)
def test_canonical_package_changelogs_match_installed_current_release(package_name) -> None:
    """Install every package changelog without changing its reviewed content."""
    source_path = _PACKAGE_FAMILY_ROOT / package_name / 'CHANGELOG.rst'
    assert source_path.is_file(), source_path
    _assert_current_release_heading(
        source_path.read_text(encoding='utf-8'),
        _package_version(package_name),
    )
    try:
        installed_share = Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        if package_name not in {'manipulation_curobo', 'manipulation_perception_isaac_ros'}:
            raise
        pytest.skip(
            f'Optional {package_name} is not installed in this build; '
            'source changelog checked, installed comparison requires the optional package'
        )
    installed_path = installed_share / 'CHANGELOG.rst'
    assert installed_path.is_file(), installed_path
    assert installed_path.read_bytes() == source_path.read_bytes(), package_name


def test_change_control_policy_states_current_compatibility_limits() -> None:
    """Require the source-only C++ promise and stable ROS wire rule."""
    policy_path = _REPOSITORY_ROOT / 'docs' / 'CHANGE_CONTROL.md'
    assert policy_path.is_file()
    policy = _normalized_text(policy_path)
    assert 'package versions follow semantic versioning independently' in policy
    assert 'stable wire contracts' in policy
    assert 'binary abi is not guaranteed' in policy
    assert 'new versioned interface' in policy


def test_security_policy_states_non_authorization_and_safety_boundaries() -> None:
    """Require explicit DDS, secret, model, and real-hardware assumptions."""
    policy_path = _REPOSITORY_ROOT / 'SECURITY.md'
    assert policy_path.is_file()
    policy = _normalized_text(policy_path)
    assert 'they do not authorize callers' in policy
    assert 'sros 2 or dds security' in policy
    assert 'do not load untrusted or unpinned artifacts' in policy
    assert 'must not be committed or written to logs or artifacts' in policy
    assert 'not a functional-safety layer' in policy
    declared_contacts = {
        maintainer.attrib['email'].lower()
        for package_name in _CANONICAL_PACKAGES
        for maintainer in ElementTree.parse(
            _PACKAGE_FAMILY_ROOT / package_name / 'package.xml'
        ).getroot().findall('maintainer')
    }
    reporting_contacts = set(re.findall(r'[\w.+-]+@[\w.-]+\.[A-Za-z]{2,}', policy))
    assert reporting_contacts, 'A private reporting contact must be available'
    assert reporting_contacts <= declared_contacts, 'Use a declared maintainer contact'
