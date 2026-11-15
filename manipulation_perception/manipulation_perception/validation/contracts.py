"""
Public facade for GPU perception evidence contract values and loading.

Responsibility:
    Preserve one concise import surface while model invariants and JSON parsing
    remain independently readable and testable.

Does not own:
    Contract behavior, ROS observation, provenance collection, or artifacts.
"""

from manipulation_perception.validation.contract_io import (
    contract_from_mapping,
    load_contract,
)
from manipulation_perception.validation.contract_model import (
    AssetContract,
    CONTRACT_SCHEMA_VERSION,
    ContractError,
    CorrelationGroupContract,
    DiagnosticContract,
    GPU_IDENTITY_EXECUTABLE,
    GPU_IDENTITY_FIELDS,
    GPU_IDENTITY_FORMAT_ARGUMENT,
    GPU_IDENTITY_QUERY_ARGUMENT,
    GpuIdentityContract,
    GpuPerceptionContract,
    REQUIRED_DIAGNOSTIC_ROLES,
    TopicContract,
)


__all__ = [
    "CONTRACT_SCHEMA_VERSION",
    "GPU_IDENTITY_FIELDS",
    "GPU_IDENTITY_EXECUTABLE",
    "GPU_IDENTITY_FORMAT_ARGUMENT",
    "GPU_IDENTITY_QUERY_ARGUMENT",
    "REQUIRED_DIAGNOSTIC_ROLES",
    "AssetContract",
    "ContractError",
    "CorrelationGroupContract",
    "DiagnosticContract",
    "GpuIdentityContract",
    "GpuPerceptionContract",
    "TopicContract",
    "contract_from_mapping",
    "load_contract",
]
