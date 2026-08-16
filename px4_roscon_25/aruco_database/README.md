# aruco_database

`aruco_database` รับ pose ของ **ArUco ทุก ID** จาก `aruco_tracker`, แปลงเป็น WGS84 latitude/longitude และเก็บพิกัดของ marker แต่ละตัวไว้ใน memory พร้อม persist ลงไฟล์ YAML

แพ็กเกจนี้ไม่ได้เลือก target แทน flight mode ใด ๆ ผู้ใช้เลือก `aruco_id` ใน `Direct` แล้ว query ตำแหน่งของ ID นั้นผ่าน service

## Data flow

```text
PX4 /fmu/out/vehicle_global_position
        │  (พิกัดโดรนตอนเริ่ม database)
        v
aruco_database ── latch launch origin ครั้งเดียว
        ▲
        │
aruco_tracker
    └── /aruco/detections (ทุก ID, pose ใน camera_frame)
        │
        v
aruco_database
   ├── in-memory marker map
   ├── aruco_database/database/markers.yaml
   └── /aruco_database/get_position

Direct ── query aruco_id ──> aruco_database ── latitude/longitude

aruco_tracker ── /target_pose (target ID เดียว) ──> PrecisionLand
```

## Auto origin

ค่าเริ่มต้นของแพ็กเกจเป็น auto-origin:

```text
auto_origin: true
global_position_topic: /fmu/out/vehicle_global_position
vehicle_frame: base_link_frd
world_frame: odom_ned
```

เมื่อ launch `aruco_database` node จะรอข้อความ `px4_msgs/msg/VehicleGlobalPosition` ที่มี `lat_lon_valid=true` และไม่อยู่ในโหมด dead-reckoning จากนั้นจะทำสิ่งต่อไปนี้เพียงครั้งเดียว:

1. ใช้ `lat`/`lon` ของโดรนเป็น WGS84 origin
2. อ่านตำแหน่งปัจจุบันของ `vehicle_frame` ใน `world_frame`
3. จำตำแหน่ง local นั้นไว้เป็น launch position
4. ใช้ระยะของ ArUco จาก launch position เพื่อคำนวณ latitude/longitude

การเก็บตำแหน่ง local ของโดรนในขั้นตอนที่ 2 สำคัญ เพราะ `odom_ned` อาจไม่ได้มีค่า `(0, 0)` ตอนที่สั่ง launch ถ้าไม่หักตำแหน่งนี้ออก พิกัด marker จะมี offset

ระบบจะไม่บันทึก detection จนกว่าจะได้ global position และ TF ของโดรนที่ถูกต้อง หาก GPS/global position ยังไม่พร้อม node จะรอข้อความถัดไปแบบ non-blocking และจะไม่เปลี่ยน origin ภายหลังจากที่ latch แล้ว แม้โดรนจะบินต่อไป

`VehicleGlobalPosition` เป็น fused global-position estimate จาก PX4 ไม่ใช่ค่าดิบจาก GPS โดยตรง ดังนั้น PX4 ต้องมี global-position estimate ที่ valid และต้องส่ง topic ผ่าน `MicroXRCEAgent`

ตรวจสอบ topic ได้ด้วย:

```bash
ros2 topic type /fmu/out/vehicle_global_position
ros2 topic echo /fmu/out/vehicle_global_position --once
```

ถ้า topic นี้ไม่มี ให้ตรวจ PX4 uXRCE-DDS client, `MicroXRCEAgent` และสถานะ estimator/GPS ก่อน launch database

## Manual fallback

ถ้าต้องการใช้ origin ที่กำหนดเอง หรือระบบไม่มี `/fmu/out/vehicle_global_position` ให้ปิด auto-origin และส่ง origin เอง:

```bash
ros2 launch aruco_database aruco_database.launch.py \
  auto_origin:=false \
  origin_configured:=true \
  origin_latitude_deg:=13.123456 \
  origin_longitude_deg:=100.123456
```

ถ้า `origin_configured:=true` พร้อมกับ `auto_origin:=true` ค่า manual จะมี priority และ node จะไม่รอ global-position topic

## สิ่งที่ต้องมีสำหรับการแปลงเป็น latitude/longitude

ต้องมี TF chain ต่อไปนี้:

```text
odom_ned -> base_link_frd -> ... -> camera_frame
```

ค่า coordinate ที่ใช้คือ:

- `world_frame` ค่าเริ่มต้น `odom_ned`
- `odom_ned.x` เป็น North displacement หน่วยเมตร
- `odom_ned.y` เป็น East displacement หน่วยเมตร
- `vehicle_frame` ค่าเริ่มต้น `base_link_frd`
- `camera_frame` มาจาก header ของ `/aruco/detections`

ต้องเริ่มระบบที่เผยแพร่ TF เช่น `common.launch.py` และ `aruco_tracker.launch.py` ก่อนหรือพร้อมกับ database

## ตำแหน่งไฟล์ database

ค่าเริ่มต้นถูกเปลี่ยนมาไว้ใน directory ของแพ็กเกจ:

```text
px4_roscon_25/aruco_database/database/markers.yaml
```

เมื่อใช้ `ros2 launch` ระบบจะ resolve path ผ่าน package share ของ `aruco_database` และติดตั้ง directory `database` ไปพร้อมกับแพ็กเกจ ใน workflow ของ workshop ที่ใช้ `colcon build --symlink-install` ไฟล์นี้จึงมองเห็นจาก host ได้ที่:

```text
/home/yosh/roscon-25-workshop/px4_roscon_25/aruco_database/database/markers.yaml
```

ใน container จะเป็น path ที่ mount source workspace เช่น:

```text
/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/px4_roscon_25/aruco_database/database/markers.yaml
```

ไฟล์เริ่มต้นมี `markers: []` และ node จะเพิ่ม `revision` ให้ไฟล์เมื่อเริ่มทำงาน หากมี detection ที่ valid ระบบจะ publish snapshot ทันที บันทึก ID ใหม่ทันที และ throttle การเขียนซ้ำตาม `save_min_interval_ms` โดยมี `save_period_s` เป็น fallback การใช้ path ใน source workspace ทำให้ข้อมูลไม่หายเมื่อ Docker container ที่ใช้ `--rm` ถูกลบ เพราะ source workspace ถูก mount จาก host

ถ้าต้องการใช้ไฟล์อื่น สามารถ override ได้ด้วย absolute path:

```bash
ros2 launch aruco_database aruco_database.launch.py \
  database_file:=/path/to/markers.yaml
```

## เริ่มใช้งาน

เริ่ม PX4/Gazebo และ workflow ของ `common.launch.py` กับ `aruco_tracker.launch.py` ตามปกติ จากนั้นเปิด database ได้เลยโดยไม่ต้องป้อน lat/lon เอง:

```bash
ros2 launch aruco_database aruco_database.launch.py
```

node จะใช้ lat/lon ของโดรนจาก global-position message แรกที่ valid หลังจากเริ่มทำงานเป็น launch origin

ไฟล์ database จะถูกสร้างเมื่อมี detection ที่แปลงพิกัดสำเร็จ โหนดเก็บข้อมูลไว้ใน memory และ publish snapshot แบบ realtime ทุกครั้งที่ database เปลี่ยนแปลง โดยค่าเริ่มต้นจะบันทึกไฟล์ทันทีเมื่อพบ ID ใหม่ และ throttle การเขียนซ้ำไว้ที่ทุก 200 ms การบันทึกตาม `save_period_s` ยังทำหน้าที่เป็น fallback และโหนดจะพยายามบันทึกครั้งสุดท้ายเมื่อ shutdown

ข้อมูลในไฟล์มีรูปแบบประมาณนี้:

```yaml
revision: 1786624212000
markers:
  - id: 0
    latitude_deg: 13.123457
    longitude_deg: 100.123455
    observation_count: 14
```

The top-level `revision` is persisted with the marker map. Older files without
that field are assigned a new opaque generation and migrated on startup, so a
revision token from a previous process cannot accidentally authorize a clear
against the reloaded database.

พิกัดจากการสังเกตซ้ำจะถูกปรับด้วย running average เพื่อไม่ให้ค่าจาก frame เดียวเขียนทับค่าทั้งหมดทันที การ restart node จะโหลดพิกัด WGS84 เดิมกลับเข้า memory ส่วน detection ใหม่จะใช้ launch origin ของรอบที่เริ่ม node ใหม่

## Service สำหรับ Direct

Service ชื่อ:

```text
/aruco_database/get_position
```

ชนิด:

```text
aruco_database/srv/GetArucoPosition
```

Request มีเพียง `aruco_id` และ response มี `found`, `latitude_deg`, `longitude_deg` และ `error_message`:

```bash
ros2 service call /aruco_database/get_position \
  aruco_database/srv/GetArucoPosition \
  "{aruco_id: 0}"
```

ถ้าพบข้อมูลจะได้ `found: true` และพิกัดของ ID นั้น ถ้ายังไม่เคยพบหรือ ID ไม่มีใน database จะได้ `found: false`

### แนวทางเรียกจาก Direct

ให้ `Direct` สร้าง client ใน constructor หรือ initialization ของ mode:

```cpp
auto client = node.create_client<aruco_database::srv::GetArucoPosition>(
    "/aruco_database/get_position");
```

เมื่อ `Direct` เริ่มทำงาน ให้สร้าง request:

```cpp
auto request = std::make_shared<aruco_database::srv::GetArucoPosition::Request>();
request->aruco_id = target_id;
auto future = client->async_send_request(request);
```

เมื่อ future เสร็จ:

```cpp
if (response->found) {
    const double latitude = response->latitude_deg;
    const double longitude = response->longitude_deg;
    // ใช้ latitude/longitude สร้าง global setpoint ของ Direct
} else {
    // target ยังไม่มีใน database: รอหรือ fail mode
}
```

ไม่ควรเรียก `future.get()` แบบ blocking ใน `updateSetpoint()` ของ `ModeBase` ทุก cycle ให้เก็บ future/state ไว้ แล้วตรวจผลแบบ non-blocking จาก update loop หรือ callback เพียงครั้งเดียวตอนเริ่ม mode

`aruco_database` ไม่ส่ง altitude เพราะ `Direct` เป็นผู้กำหนดความสูงบินเอง และ database ไม่ได้เลือก target แทน `Direct`

## Topics

### Input: `/aruco/detections`

ชนิด:

```text
aruco_database/msg/ArucoDetectionArray
```

ในหนึ่งข้อความมี marker ทุกตัวที่ตรวจพบในภาพเดียวกัน โดย `header.stamp` เป็น timestamp ของภาพและ `header.frame_id` ต้องเป็น frame ของกล้องที่มี TF ใช้งานได้

### Auto-origin input: `/fmu/out/vehicle_global_position`

ชนิด:

```text
px4_msgs/msg/VehicleGlobalPosition
```

ใช้เฉพาะเพื่อ latch launch origin ครั้งเดียว ไม่ได้ใช้ติดตาม origin ระหว่างการบิน

### Compatibility output: `/target_pose`

`aruco_tracker` ยังคงส่ง `geometry_msgs/msg/PoseStamped` เฉพาะ `aruco_id` ที่ตั้งใน tracker เพื่อให้ `PrecisionLand` ทำงานแบบเดิม

## Parameters

| Parameter | Default | ความหมาย |
|---|---:|---|
| `detection_topic` | `/aruco/detections` | topic detection จาก tracker |
| `global_position_topic` | `/fmu/out/vehicle_global_position` | PX4 fused global-position ที่ใช้ตั้ง auto-origin |
| `vehicle_frame` | `base_link_frd` | frame ของโดรนที่ใช้เก็บ local launch position |
| `database_file` | `aruco_database/database/markers.yaml` | ไฟล์ persistence |
| `world_frame` | `odom_ned` | frame ที่ x/y เป็น North/East |
| `auto_origin` | `true` | latch global position แรกที่ valid เป็น origin |
| `origin_configured` | `false` | ใช้ manual origin แทน auto-origin |
| `origin_latitude_deg` | `0.0` | latitude สำหรับ manual origin |
| `origin_longitude_deg` | `0.0` | longitude สำหรับ manual origin |
| `save_period_s` | `2.0` | ระยะเวลา flush database สำรอง |
| `save_on_update` | `true` | บันทึกเมื่อ detection ที่ valid เปลี่ยน database |
| `save_min_interval_ms` | `200` | ระยะห่างขั้นต่ำระหว่างการเขียนที่เกิดจาก update |
| `transform_timeout_s` | `0.05` | timeout สำหรับ TF lookup |

## ข้อจำกัดปัจจุบัน

- auto-origin ต้องมี `/fmu/out/vehicle_global_position` ที่ `lat_lon_valid=true` และต้องมี TF จาก `world_frame` ไป `vehicle_frame`
- ถ้า global position ยังไม่พร้อม node จะรอและยังไม่บันทึก ArUco
- auto-origin ใช้ global position แรกที่ valid หลัง node เริ่ม ไม่ใช่การอัปเดต origin ตามโดรนทุก frame
- marker ต้องถูกตรวจพบและมี TF ถึง `world_frame` ก่อนจึงจะบันทึกได้
- database ใช้ ID เป็น key เดียว เหมาะกับกรณีที่ใช้ ArUco dictionary เดียวกันทั้งระบบ
- ถ้า `Direct` เลือก ID ที่ยังไม่อยู่ในไฟล์และยังไม่ถูกตรวจพบ service จะตอบ `found: false`

## Realtime management interfaces

`aruco_database` remains the source of truth for the marker map. It publishes a
complete, sorted snapshot whenever an accepted detection changes the map:

```text
/aruco_database/markers
```

Type:

```text
aruco_database/msg/ArucoMarkerArray
```

The snapshot uses reliable, transient-local QoS so a late subscriber receives
the latest state. Its fields are:

```text
std_msgs/Header header
uint64 revision
ArucoMarker[] markers
```

Each `ArucoMarker` contains:

```text
int32 id
float64 latitude_deg
float64 longitude_deg
uint64 observation_count
```

`revision` increases when the map changes and is written into the YAML file with
the same snapshot. It is an opaque `uint64` generation token; a legacy file or a
newly created database receives a new generation before the first save. A
complete empty snapshot after a clear has `markers: []`; consumers should
replace their local table from every snapshot rather than append rows
indefinitely.

The database state is published on:

```text
/aruco_database/status
```

Type:

```text
aruco_database/msg/ArucoDatabaseStatus
```

The status reports `revision`, `marker_count`, `origin_ready`,
`database_dirty`, `persistence_ok`, `persistence_state`, and `last_error`.
`persistence_state` is `synced`, `pending`, or `error`. `origin_ready` is
separate from `marker_count`: records loaded from a previous YAML file can be
read while the current launch origin is still waiting for PX4.

## List and clear services

The complete map can be requested on demand without reading YAML:

```text
/aruco_database/list_markers
aruco_database/srv/ListArucoMarkers
```

Example:

```bash
ros2 service call /aruco_database/list_markers \
  aruco_database/srv/ListArucoMarkers "{}"
```

To start a new training area, use the clear service:

```text
/aruco_database/clear
aruco_database/srv/ClearArucoDatabase
```

Request:

```text
bool confirm
bool backup
bool use_expected_revision
uint64 expected_revision
```

Response:

```text
bool success
string error_message
uint64 revision
uint32 marker_count
bool persisted
string backup_file
```

`confirm` must be true. With `backup: true`, the active in-memory database is
persisted first and then copied to a unique timestamped sibling backup before
the clear. This makes the backup include accepted observations that were still
inside the update-write throttle. With `use_expected_revision: true`, the
service rejects a stale request whose revision does not match
`expected_revision`.

A successful clear is deliberately a coordinated operation: it clears the
active map, writes `revision: <new-generation>` and `markers: []` to the
configured YAML file using the temporary-file/flush/fsync/rename sequence, and
publishes a new empty snapshot. The package does not expose a file-only delete
because leaving old records in memory would allow the next save to recreate the
deleted file contents.

The existing service is unchanged for compatibility:

```text
/aruco_database/get_position
aruco_database/srv/GetArucoPosition
```

## Persistence and power loss

`save_on_update` is enabled by default. A valid detection batch is published to
ROS immediately. A newly discovered marker triggers an immediate persistence
attempt; repeated updates are limited by `save_min_interval_ms` (200 ms by
default). The periodic `save_period_s` timer remains a retry/fallback for a
dirty database, and the destructor attempts a final save.

This design avoids writing a small YAML file once per camera frame while
keeping the power-loss window short. Writes go to `markers.yaml.tmp`, flush the
stream, `fsync` the temporary file, atomically rename it over the configured
database file, and `fsync` the parent directory. This provides a durable
POSIX/Linux write sequence after a successful save; storage hardware and the
operating system still determine the final power-failure guarantees. If a save
fails, the map stays dirty and the status reports `persistence_state: error`
until a later retry succeeds.

For Node-RED integration, use `aruco_database_bridge` rather than opening the
file. The bridge converts `/aruco_database/markers` and `/aruco_database/status`
to retained MQTT JSON topics and converts its `clear_file` command into the
coordinated clear service call. See the bridge guide at:

```text
px4_roscon_25/aruco_database_bridge/README.md
```
