# Responsibility: Aggregate component health into one ordered monitor snapshot.

# Sample time from the node's ROS clock. It is informational and may pause,
# jump or reset with /clock; never use it for freshness or ordering.
builtin_interfaces/Time sample_stamp

# Monotonic within one monitor process lifetime. Use it to order snapshots.
uint64 sequence

# Process-relative steady-clock time at sampling. It is not comparable between
# hosts or monitor restarts, but remains monotonic across ROS-time resets.
builtin_interfaces/Duration steady_uptime

# Conservative readiness policy: true only when every required component is
# LEVEL_OK. WARN, including startup grace and delayed data, blocks new motion.
bool motion_ready

# Uses HealthComponentStatus.LEVEL_* values. Consumers must tolerate future
# levels and inspect components for machine-readable causes.
uint8 overall_level
HealthComponentStatus[] components
