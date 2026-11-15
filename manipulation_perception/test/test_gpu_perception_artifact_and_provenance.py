"""
Test explicit GPU/model provenance and the standalone evidence artifact.

Responsibility: Pin read-only command execution, exact file hashing, claims, and schema.
"""

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from gpu_perception_evidence_test_support import contract_value, passing_session
import jsonschema
import pytest

from manipulation_perception.validation.artifact import build_artifact, write_artifact
from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.provenance import (
    collect_gpu_identity,
    hash_assets,
    ProvenanceError,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_SCHEMA = (
    PACKAGE_ROOT / "config" / "gpu-perception-evidence.schema.v1.json"
)
REVISION = "1" * 40


def _asset_paths(tmp_path: Path) -> dict[str, Path]:
    engine = tmp_path / "engine.plan"
    model = tmp_path / "model.pt"
    engine.write_bytes(b"engine")
    model.write_bytes(b"model")
    return {"engine": engine, "model": model}


def _gpu_identity(contract: GpuPerceptionContract):
    calls = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(
            returncode=0,
            stdout="RTX Test, GPU-uuid, 555.1, 16384\n",
            stderr="",
        )

    evidence = collect_gpu_identity(
        contract.gpu_identity,
        runner=runner,
        executable_resolver=lambda _command: "/usr/bin/nvidia-smi",
    )
    return evidence, calls


def test_asset_hashes_use_only_exact_explicit_paths(tmp_path: Path) -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())
    paths = _asset_paths(tmp_path)

    digests = hash_assets(contract.assets, paths)

    assert [item.name for item in digests] == ["engine", "model"]
    assert digests[0].path == str(paths["engine"].resolve())
    assert digests[0].sha256 == hashlib.sha256(b"engine").hexdigest()

    with pytest.raises(ProvenanceError, match="missing=model"):
        hash_assets(contract.assets, {"engine": paths["engine"]})
    with pytest.raises(ProvenanceError, match="must be absolute"):
        hash_assets(
            contract.assets,
            {"engine": Path("engine.plan"), "model": paths["model"]},
        )


def test_gpu_identity_executes_fixed_query_without_shell() -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())

    evidence, calls = _gpu_identity(contract)

    argv, kwargs = calls[0]
    assert argv == (
        "/usr/bin/nvidia-smi",
        "--query-gpu=name,uuid,driver_version,memory.total",
        "--format=csv,noheader,nounits",
    )
    assert "shell" not in kwargs
    assert kwargs["timeout"] == 5.0
    assert evidence.devices == (
        {
            "name": "RTX Test",
            "uuid": "GPU-uuid",
            "driver_version": "555.1",
            "memory_total": "16384",
        },
    )
    assert evidence.command[0] == "/usr/bin/nvidia-smi"


def test_gpu_identity_rejects_bad_exit_and_malformed_rows() -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())

    with pytest.raises(ProvenanceError, match="returned 1"):
        collect_gpu_identity(
            contract.gpu_identity,
            runner=lambda *_args, **_kwargs: SimpleNamespace(
                returncode=1, stdout="", stderr="no gpu"
            ),
            executable_resolver=lambda _command: "/usr/bin/nvidia-smi",
        )

    with pytest.raises(ProvenanceError, match="unexpected executable"):
        collect_gpu_identity(
            contract.gpu_identity,
            runner=lambda *_args, **_kwargs: SimpleNamespace(
                returncode=0,
                stdout="RTX Test, GPU-uuid, 555.1, 16384\n",
                stderr="",
            ),
            executable_resolver=lambda _command: "/tmp/nvidia-smi",
        )
    with pytest.raises(ProvenanceError, match="expected 4"):
        collect_gpu_identity(
            contract.gpu_identity,
            runner=lambda *_args, **_kwargs: SimpleNamespace(
                returncode=0, stdout="only,three,fields\n", stderr=""
            ),
            executable_resolver=lambda _command: "/usr/bin/nvidia-smi",
        )


def test_pass_artifact_is_revision_bound_schema_valid_and_non_replay(
    tmp_path: Path,
) -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())
    evaluation = passing_session(contract).evaluate(now_monotonic=103.1)
    gpu_identity, _calls = _gpu_identity(contract)
    digests = hash_assets(contract.assets, _asset_paths(tmp_path))

    artifact = build_artifact(
        contract=contract,
        run_token="test-run-token",
        revision=REVISION,
        started_at="2026-07-14T00:00:00+00:00",
        finished_at="2026-07-14T00:00:04+00:00",
        evaluation=evaluation,
        gpu_identity=gpu_identity,
        asset_digests=digests,
    )
    output = write_artifact(tmp_path / "artifact", artifact)
    value = json.loads(output.read_text(encoding="utf-8"))
    schema = json.loads(ARTIFACT_SCHEMA.read_text(encoding="utf-8"))

    jsonschema.Draft202012Validator(schema).validate(value)
    assert value["evidence_status"] == "pass"
    assert value["revision"] == {
        "value": REVISION,
        "source": "explicit_argument",
    }
    assert value["claim_scope"] == {
        "runtime_correlation": True,
        "accuracy": False,
        "replay": False,
    }
    assert value["run_token"] == "test-run-token"
    assert value["errors"] == []
    assert all(
        item["latest_source_sample"] is not None for item in value["diagnostics"]
    )
    assert output.name == "gpu-perception-evidence.v1.json"


def test_missing_provenance_produces_schema_valid_failure(tmp_path: Path) -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())
    evaluation = passing_session(contract).evaluate(now_monotonic=103.1)
    artifact = build_artifact(
        contract=contract,
        run_token="failed-run",
        revision=REVISION,
        started_at="start",
        finished_at="finish",
        evaluation=evaluation,
        gpu_identity=None,
        asset_digests=(),
        provenance_errors=("identity query unavailable",),
    )
    schema = json.loads(ARTIFACT_SCHEMA.read_text(encoding="utf-8"))

    jsonschema.Draft202012Validator(schema).validate(artifact.value)
    assert artifact.evidence_status == "fail"
    assert "identity query unavailable" in artifact.value["errors"]


@pytest.mark.parametrize("revision", ["", "abc", "A" * 40, "1" * 39])
def test_artifact_rejects_unbound_revision(revision: str) -> None:
    contract = GpuPerceptionContract.from_mapping(contract_value())
    evaluation = passing_session(contract).evaluate(now_monotonic=103.1)

    with pytest.raises(ValueError, match="revision"):
        build_artifact(
            contract=contract,
            run_token="run",
            revision=revision,
            started_at="start",
            finished_at="finish",
            evaluation=evaluation,
            gpu_identity=None,
            asset_digests=(),
        )
