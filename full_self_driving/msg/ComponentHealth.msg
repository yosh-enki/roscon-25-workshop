uint8 STATE_UNKNOWN=0
uint8 STATE_STARTING=1
uint8 STATE_ACTIVE=2
uint8 STATE_INACTIVE=3
uint8 STATE_DEGRADED=4
uint8 STATE_FAILED=5

string<=32 component_id
uint8 state
bool ready
uint64 last_update_monotonic_ns
uint32 queue_depth
uint32 queue_drop_count
string<=256 detail
