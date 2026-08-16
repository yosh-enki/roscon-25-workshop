uint8 QUALITY_REJECTED=0
uint8 QUALITY_ACCEPTED=1
uint8 QUALITY_OUTLIER=2

MessageHeader header
TargetIdentity identity
string<=64 map_id
string<=64 scenario_id
string<=64 pose_frame
geometry_msgs/Pose pose
float64[36] covariance
float32 quality
builtin_interfaces/Time image_time
uint64 received_monotonic_ns
string<=64 calibration_sha256
uint8 observation_state
