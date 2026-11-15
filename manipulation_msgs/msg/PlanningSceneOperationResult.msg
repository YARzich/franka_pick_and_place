# Responsibility: Define typed results shared by PlanningScene coordinator services.

uint16 NONE=0
uint16 INVALID_REQUEST=100
uint16 RESOURCE_CONFLICT=101
uint16 LEASE_NOT_FOUND=102
uint16 LEASE_EXPIRED=103
uint16 NOT_OWNER=104
uint16 REVISION_MISMATCH=105
uint16 BACKEND_FAILED=106
uint16 TIMEOUT=107
uint16 SESSION_MISMATCH=108
uint16 IDEMPOTENCY_CONFLICT=109
uint16 OPERATION_IN_PROGRESS=110
uint16 OPERATION_TICKET_INVALID=111
uint16 OPERATION_CAPACITY_EXCEEDED=112
uint16 RECONCILIATION_REQUIRED=113
uint16 DEPENDENCY_UNAVAILABLE=114
uint16 RESOURCE_QUARANTINED=115
uint16 INTERNAL_ERROR=255

bool success

# Machine-readable result. Values are append-only and message is never a
# control-flow API.
uint16 error_code

# Coordinator revision observed after this request. On success it is the new
# revision; on a rejected request it lets the caller resynchronize explicitly.
uint64 revision

# Operator context only. Clients must branch on error_code.
string message
