"""
Test strict scenario input for the reusable GPU perception evidence probe.

Responsibility: Pin JSON shape, cross-field invariants, and read-only GPU argv.
"""

from copy import deepcopy
import json
from pathlib import Path

from gpu_perception_evidence_test_support import contract_value
import jsonschema
import pytest

from manipulation_perception.validation.contracts import (
    ContractError,
    GpuPerceptionContract,
    load_contract,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_SCHEMA = (
    PACKAGE_ROOT / "config" / "gpu_perception_probe_contract.schema.v1.json"
)


def test_complete_contract_round_trips_and_matches_json_schema(tmp_path: Path) -> None:
    value = contract_value()
    schema = json.loads(CONTRACT_SCHEMA.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator(schema).validate(value)
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(value), encoding="utf-8")

    contract = load_contract(path)

    assert contract.as_dict() == value
    assert len(contract.sha256) == 64
    assert [asset.name for asset in contract.assets] == ["engine", "model"]
    assert all(not hasattr(asset, "path") for asset in contract.assets)


@pytest.mark.parametrize("mutation", ["--gpu-reset", "-pm", "-lgc"])
def test_contract_rejects_mutating_nvidia_smi_arguments(mutation: str) -> None:
    value = contract_value()
    value["gpu_identity"]["argv"][1] = mutation

    with pytest.raises(ContractError, match="read-only GPU identity query"):
        GpuPerceptionContract.from_mapping(value)


def test_contract_requires_all_four_diagnostic_roles() -> None:
    value = contract_value()
    value["diagnostics"] = value["diagnostics"][:-1]

    with pytest.raises(ContractError, match="pose_adapter"):
        GpuPerceptionContract.from_mapping(value)


def test_contract_rejects_unknown_diagnostic_source_role() -> None:
    value = contract_value()
    value["diagnostics"][0]["source_topic_role"] = "unobserved"

    with pytest.raises(ContractError, match="unknown topic roles"):
        GpuPerceptionContract.from_mapping(value)


def test_contract_rejects_duplicate_topics_and_unknown_group_roles() -> None:
    duplicate = contract_value()
    duplicate["topics"][1]["topic"] = duplicate["topics"][0]["topic"]
    with pytest.raises(ContractError, match="topic names"):
        GpuPerceptionContract.from_mapping(duplicate)

    unknown = contract_value()
    unknown["correlation_groups"][0]["roles"].append("missing")
    with pytest.raises(ContractError, match="unknown roles"):
        GpuPerceptionContract.from_mapping(unknown)

    unreferenced = contract_value()
    unreferenced["correlation_groups"][0]["roles"].remove("image")
    with pytest.raises(ContractError, match="missing: image"):
        GpuPerceptionContract.from_mapping(unreferenced)


def test_terminal_role_requires_pose_stamped_and_one_group() -> None:
    wrong_type = contract_value()
    wrong_type["topics"][-1]["message_type"] = "geometry_msgs/msg/Pose"
    with pytest.raises(ContractError, match="PoseStamped"):
        GpuPerceptionContract.from_mapping(wrong_type)

    duplicate_group = contract_value()
    duplicate_group["correlation_groups"].append(
        {"name": "second", "roles": ["image", "terminal_pose"]}
    )
    with pytest.raises(ContractError, match="exactly one"):
        GpuPerceptionContract.from_mapping(duplicate_group)


def test_contract_rejects_unknown_keys_and_invalid_scalars() -> None:
    unknown = contract_value()
    unknown["implicit_default"] = True
    with pytest.raises(ContractError, match="unknown=implicit_default"):
        GpuPerceptionContract.from_mapping(unknown)

    invalid = deepcopy(contract_value())
    invalid["minimum_observation_sec"] = invalid["observation_timeout_sec"]
    with pytest.raises(ContractError, match="must be lower"):
        GpuPerceptionContract.from_mapping(invalid)


def test_contract_rejects_arbitrary_nvidia_smi_executable() -> None:
    value = contract_value()
    value["gpu_identity"]["argv"][0] = "/tmp/nvidia-smi"

    with pytest.raises(ContractError, match="/usr/bin/nvidia-smi"):
        GpuPerceptionContract.from_mapping(value)
