"""
Reusable perception logic owned by `manipulation_perception.common`.

Responsibility:
- Provide importable logic used by perception nodes, workers and tools.
- Keep ROS graph side effects out of helper modules where possible.
- Isolate algorithmic logic, runtime file writing and worker-client lifecycle.

Does not own:
- Node parameter declaration, subscriptions or publishers.
- Launch-time orchestration.
- Heavy ML model implementation internals.
"""
