uint8 SAFETY_STATUS_OK=0
uint8 SAFETY_STATUS_WARNING=1
uint8 SAFETY_STATUS_HOLD=2
uint8 SAFETY_STATUS_ABORT=3
uint8 SAFETY_STATUS_FAILSAFE=4
uint8 SAFETY_STATUS_EMERGENCY_STOP=5

MessageHeader header
uint8 safety_status
bool emergency_stop_active
bool geofence_violation
bool battery_critical
bool link_lost
string<=256 safety_detail
uint64 safety_monotonic_ns
