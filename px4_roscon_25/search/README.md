# Search mode and `search_bridge`

The `search` ROS 2 package contains two executables:

- `search`: the PX4 custom flight mode named **Search**.
- `search_bridge`: a ROS 2 to MQTT bridge for PX4 status and the small set of
  planner-management operations needed by a dashboard or Node-RED.

The bridge is **not** a generic PX4 command gateway. It does not arm, take off,
change modes, or publish PX4 commands. Its MQTT commands only list valid manual
plans and create a new working-plan copy.

This guide is the operator and dashboard-developer reference for the current
implementation. The examples use the default MQTT topic prefix `search`; use
`<prefix>` in an integration when `mqtt_topic_prefix` is changed.

## Safety boundary

Plan management is fail-closed:

- Every `search_bridge` planner command is checked against the latest PX4
  `VehicleStatus` on the ROS executor thread.
- The PX4 sample must be fresh, have a non-zero PX4 timestamp, and report
  `ARMING_STATE_DISARMED` (PX4 value `1`).
- Node-RED should enable plan controls only when the current
  `<prefix>/planner/status` payload has both:

  ```text
  status_state === "DISARMED"
  plan_management_allowed === true
  ```
- The Node-RED check is user-interface feedback, not authorization. The bridge
  repeats the check for every command and rejects commands while armed, stale,
  unknown, or offline.
- Retained PX4 and plans messages are snapshots for display. A retained
  `FRESH` PX4 message or a retained plans list is not permission to edit a plan.
- A planner-status Last Will publishes `OFFLINE` with
  `plan_management_allowed: false` if the bridge connection is lost. An orderly
  bridge shutdown publishes the same fail-closed status first.

Always verify the vehicle state in the normal PX4/QGroundControl workflow
before arming or selecting the Search mode.

## Architecture and package layout

```text
PX4 /fmu/out/vehicle_status_v1 -------------------┐
PX4 /fmu/out/vehicle_land_detected ---------------┤
                                                   v
                                      search_bridge ROS 2 node
                                      - subscribes to PX4 status
                                      - publishes MQTT over verified TLS
                                      - checks fresh DISARMED state
                                      - queues MQTT commands for ROS work
                                                   ^
                                                   |
                         Node-RED/dashboard <-> HiveMQ

Search custom mode <-> SearchPlanner <-> plans/manual and plans/working
```

`SearchMode` is the PX4 `ModeBase` executable. `SearchPlanner` is an in-process
C++ component shared by `search` and `search_bridge`; it is not another flight
mode and does not create a ROS service.

The package follows the standard ROS 2 source layout:

```text
px4_roscon_25/search/
├── src/                    # C++ implementation
├── include/search/         # public C++ headers
├── config/search_params.yaml
├── plans/manual/           # source QGroundControl .plan files
├── plans/working/          # generated timestamped working copies
├── CMakeLists.txt
└── README.md
```

The container and Node-RED do not need a direct ROS graph connection. Both the
bridge and Node-RED make outbound TLS connections to the MQTT broker.

## Prerequisites

The workshop image provides ROS 2 Humble, PX4 messages, `px4_ros2_cpp`,
`colcon`, Gazebo Harmonic, PX4 SITL, `libmosquitto`, and the `mosquitto`
client tools. See [`docs/setup.md`](../../docs/setup.md) for general workshop
requirements.

You need:

- Docker and, optionally, the VS Code Dev Containers extension.
- A HiveMQ (or compatible) MQTT broker with TLS on port `8883`.
- A broker account with username/password authentication and ACLs for the
  selected topic prefix.
- A QGroundControl connection for normal PX4 operation and arming decisions.

### Keep MQTT credentials outside the repository

The bridge supports username/password authentication. Keep both values in the
host environment file below; do not put them in `search_params.yaml`, a
Dockerfile, a launch file, a Docker image layer, a shell command, or a Git
commit.

Create the host file **before** opening or rebuilding the VS Code container:

```sh
mkdir -p "$HOME/.config/hivemq"
install -m 600 /dev/null "$HOME/.config/hivemq/search.env"
${EDITOR:-vi} "$HOME/.config/hivemq/search.env"
```

The file format is:

```text
HIVEMQ_MQTT_USERNAME=<your-hivemq-username>
HIVEMQ_MQTT_PASSWORD=<your-hivemq-password>
```

Replace the placeholders only in the local file. Keep the file at mode `600`
and do not copy it into the workspace. The devcontainer definitions and the
pure-Docker command pass this file to the container as environment variables.

The ROS parameters `mqtt_username` and `mqtt_password` are intentionally empty
in the normal configuration. When either parameter is empty, the bridge reads
the corresponding environment variable. If `mqtt_host` is configured, both
credentials are required. The bridge fails with:

```text
search_bridge: MQTT credentials are required when mqtt_host is configured
```

If only one credential is supplied, startup also fails. Credentials are never
included in MQTT payloads or normal bridge log messages.

## Start the development environment

### VS Code Dev Container

The repository has three container profiles:

- `.devcontainer/linux/devcontainer.json`: GUI-capable container using the host
  display and `/dev/dri`.
- `.devcontainer/nogui/devcontainer.json`: headless container; it forwards the
  PX4 UDP port for QGroundControl on the host.
- `.devcontainer/nvidia/devcontainer.json`: GUI-capable NVIDIA container.

The profiles all reference the host file
`${localEnv:HOME}/.config/hivemq/search.env`. Therefore the file must exist on
the **host**, not only inside a previous container, before either of these
commands:

1. Open the repository folder in VS Code.
2. Run **Dev Containers: Reopen in Container** and choose a profile.
3. If the image or dependencies changed, run **Dev Containers: Rebuild
   Container**. Confirm the host environment file still exists before the
   rebuild.

Inside a new terminal, verify that the values were passed without printing
them:

```sh
if [[ -n "${HIVEMQ_MQTT_USERNAME:-}" && -n "${HIVEMQ_MQTT_PASSWORD:-}" ]]; then
  echo "MQTT credentials are present in the container"
else
  echo "MQTT credentials are missing"
fi
```

Changing `search.env` requires recreating/rebuilding the container so Docker
re-reads the `--env-file`. Do not use `source` on the credential file in a
terminal where its contents could be recorded in shell history.

### Pure Docker

From the repository root, optionally rebuild the image after dependency or
source changes:

```sh
./docker/docker_build.sh
```

Start a GUI container:

```sh
./docker/docker_run.sh \
  --env-file "$HOME/.config/hivemq/search.env"
```

Start a headless container or a GUI NVIDIA container, respectively:

```sh
./docker/docker_run.sh \
  --no-gui \
  --env-file "$HOME/.config/hivemq/search.env"

./docker/docker_run.sh \
  --nvidia \
  --env-file "$HOME/.config/hivemq/search.env"
```

The script mounts the repository at:

```text
/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop
```

It names the container `px4-roscon-25`. Open another shell with:

```sh
docker exec -it px4-roscon-25 bash
```

The bridge and Node-RED both connect outward to HiveMQ, so no MQTT port needs
to be published by Docker. The headless script exposes UDP port `18570` for a
QGroundControl instance on the host and TCP port `8765` for Foxglove.

## Build and test the package

Run these commands inside the container from the workspace root:

```sh
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

colcon build \
  --packages-select search \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DBUILD_TESTING=ON

source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash
colcon test --packages-select search --event-handlers console_direct+
colcon test-result --verbose
```

A successful build installs both executables, the public headers, the
configuration, the plan directories, and this README below the `search`
package share directory. The runtime path can be inspected with:

```sh
ros2 pkg prefix search
```

The package parameters below are normally loaded from:

```sh
"$(ros2 pkg prefix search)/share/search/config/search_params.yaml"
```

## Plan files and directory configuration

The installed package contains source plans and generated working copies:

```text
plans/
├── manual/
│   └── aavc2026_mission.plan
└── working/
    ├── .active_working_plan
    └── 20260813T143012Z_aavc2026_mission.plan
```

`manual` is the source-plan directory. `working` contains timestamped copies;
the old abbreviated names `manu`, `work`, and the fixed filename
`working.plan` are not part of this contract.

A working copy is created lazily when `SearchPlanner` first needs one. Its name
is UTC `YYYYMMDDTHHMMSSZ_<manual_basename>.plan`. A reset always creates a new
file and does not overwrite or delete historical working copies. When a valid
`.active_working_plan` marker exists, it is selected first; the newest valid
filename is used only when the marker is absent or invalid. Therefore, a newer
file without a valid marker does not automatically override the marked plan.
A reset made by Node-RED while Search is inactive is picked up on the next
Search activation.

When Search is deactivated, it atomically saves metadata such as the
following **when a valid global position is available and the working-plan
rewrite succeeds**:

```json
"searchPlanner": {
  "entryPoint": [13.7311, 100.7898],
  "nextWaypointIndex": 3
}
```

If the global position is invalid, Search logs a warning and leaves the
working plan unchanged. `entryPoint` is the current global latitude/longitude;
`nextWaypointIndex` is the first original waypoint that still needs to be
searched. The next Search activation flies to the saved entry point and
continues with the remaining command-16 waypoints. This automatic checkpoint
is an internal resume mechanism, not an MQTT command.

Only safe manual-plan basenames are accepted by MQTT. A valid name is a single
`.plan` filename such as `aavc2026_mission.plan`; absolute paths, `/`, `\\`,
`..`, and other extensions are rejected. The bridge never exposes arbitrary
filesystem paths.

By default, empty `manual_plan_directory` and `working_plan_directory`
parameters resolve to the installed package share. For a writable deployment,
override **both parameters for both executables with the same absolute paths**.
For example:

```sh
REPO=/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop
MANUAL_DIR="$REPO/px4_roscon_25/search/plans/manual"
WORKING_DIR="$REPO/px4_roscon_25/search/plans/working"
CFG="$(ros2 pkg prefix search)/share/search/config/search_params.yaml"

ros2 run search search \
  --ros-args \
  --params-file "$CFG" \
  -p "manual_plan_directory:=$MANUAL_DIR" \
  -p "working_plan_directory:=$WORKING_DIR"

ros2 run search search_bridge \
  --ros-args \
  --params-file "$CFG" \
  -p "manual_plan_directory:=$MANUAL_DIR" \
  -p "working_plan_directory:=$WORKING_DIR" \
  -p "mqtt_host:=your-cluster-hostname"
```

Do not give `search` and `search_bridge` different plan directories. They have
separate `SearchPlanner` objects but must observe the same manual and working
files. `reset_working_plan: true` is a startup option for creating a fresh
copy; use it deliberately and do not enable it independently in both nodes
when they share a working directory. The normal setting is `false`, with
resets initiated explicitly through the guarded MQTT command.

## Configure HiveMQ and MQTT clients

### Broker connection

Use the HiveMQ cluster **hostname**, not a URL, with TLS port `8883`:

```text
mqtt_host: your-cluster-hostname
mqtt_port: 8883
```

Do not put `mqtts://` or `https://` in `mqtt_host`. The bridge always enables
TLS and certificate verification. With `mqtt_tls_ca_file` empty, it uses the
container system CA directory `/etc/ssl/certs`. Set `mqtt_tls_ca_file` only
when a custom CA file is required; there is no insecure TLS mode.

The implemented authentication method is **username/password** through
`HIVEMQ_MQTT_USERNAME` and `HIVEMQ_MQTT_PASSWORD` (or explicitly supplied ROS
parameters). **Mutual TLS (mTLS) client certificates are not implemented**:
the bridge has no client-certificate or client-key parameters. Do not design a
Node-RED or bridge setup that depends on mTLS client credentials.

Set `mqtt_client_id` to a unique value for each concurrently running bridge.
The default is `search_bridge`. Two bridge processes using the same client ID
can disconnect one another. The Node-RED broker client must also use a
*different* client ID.

### Recommended ACLs

For the default prefix, replace `<prefix>` with `search`. Configure the bridge
account with the following least-privilege permissions:

| Client | Allow publish | Allow subscribe |
|---|---|---|
| `search_bridge` | `<prefix>/px4/status`, `<prefix>/planner/plans`, `<prefix>/planner/status`, `<prefix>/planner/response` | `<prefix>/planner/command` |
| Node-RED/dashboard | `<prefix>/planner/command` | `<prefix>/px4/status`, `<prefix>/planner/plans`, `<prefix>/planner/status`, `<prefix>/planner/response` |

The bridge must be allowed to publish `<prefix>/planner/status` because the
retained Last Will uses that topic. A temporary broader `<prefix>/#` ACL can
help diagnose ACL configuration, but narrow it before deployment. Do not grant
Node-RED access to arbitrary filesystem or PX4 command topics; this package
does not use such topics.

### Non-destructive broker checks

The following checks do not publish a planner command or modify a plan.
Execute them inside the container, where `mosquitto-clients` is installed.
First check TLS certificate verification without credentials:

```sh
BROKER_HOST=your-cluster-hostname
openssl s_client \
  -connect "${BROKER_HOST}:8883" \
  -servername "$BROKER_HOST" \
  -verify_return_error \
  </dev/null
```

Then subscribe to the planner-status topic. Use the environment variables already
injected into the container; do not paste their values into the command:

```sh
PREFIX=search

timeout 10s mosquitto_sub \
  -h "$BROKER_HOST" \
  -p 8883 \
  --capath /etc/ssl/certs \
  -u "$HIVEMQ_MQTT_USERNAME" \
  -P "$HIVEMQ_MQTT_PASSWORD" \
  -t "$PREFIX/planner/status" \
  -q 1 \
  -C 1 \
  -v
```

This is subscribe-only. A received message confirms that the broker,
credentials, TLS, ACL, and topic are usable. `mosquitto_sub` may receive either
the broker's retained snapshot or a live periodic planner-status publish, and
this command does not display the MQTT retain flag. Use Node-RED's `msg.retain`
metadata when you need to distinguish those cases. If no message arrives, the
command can time out even after a successful connection; use `-d` for
short-lived client diagnostics and inspect the bridge log. Because `-P` is a
command-line option, run this only in a private container and never paste the
full process command or terminal output containing secrets into an issue or
chat.

## MQTT topic contract

The bridge normalizes leading/trailing slashes from `mqtt_topic_prefix` and
rejects MQTT wildcard characters. With the default prefix `search`, the topics
are:

| Topic | Direction | QoS | Retained | Purpose |
|---|---|---:|---:|---|
| `<prefix>/px4/status` | bridge → dashboard | 1 | yes | PX4 status snapshot and freshness metadata |
| `<prefix>/planner/plans` | bridge → dashboard | 1 | yes | Valid manual-plan names and active working filename |
| `<prefix>/planner/status` | bridge → dashboard | 1 | yes | Planner safety state and Last Will authority |
| `<prefix>/planner/command` | dashboard → bridge | 1 | **no** | `list_manual_plans` and `reset_working_plan` only |
| `<prefix>/planner/response` | bridge → dashboard | 1 | **no** | Response correlated by `request_id` |

The bridge subscribes to the command topic with QoS 1. A command received with
its MQTT retain flag set is ignored; the command topic must never be retained.
Configure the Node-RED MQTT-out node with QoS 1 and retain disabled.

Status and plan snapshots are retained so a newly connected dashboard can
render something immediately. Treat the retained PX4 and plans values as
informational until a current planner-status update and local timeout policy
allow a control. Responses are not retained, so Node-RED must subscribe before
publishing a command and must not expect an old response after reconnecting.

The bridge remembers up to 256 processed request IDs for the lifetime of its
process. A duplicate request ID replays the cached response instead of
performing the command a second time while that ID remains in the cache. The
cache is lost when the bridge restarts, and older IDs can be evicted after the
256-entry limit.

There are deliberate no-response or non-correlatable cases: retained command
messages are ignored, commands dropped because the 64-entry queue is full or a
MQTT disconnect clears the queue are not executed, and malformed input that
cannot provide a request ID can only produce a response with an empty ID. A
Node-RED flow must not wait forever for such a response; use a local timeout and
show an unknown outcome for operations that may have changed the working plan.

## PX4 status payload

`search_bridge` subscribes to the PX4 ROS 2 topic shown below. This is the
versioned topic emitted by the PX4 DDS interface used by this workshop:

| PX4 topic | ROS message | Use |
|---|---|---|
| `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` | Arming, navigation, failsafe and connection fields |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` | Optional `landed` field |

A normal fresh payload on `<prefix>/px4/status` contains all of these fields:

```json
{
  "source": "px4",
  "topic": "/fmu/out/vehicle_status_v1",
  "sequence": 42,
  "status_state": "FRESH",
  "mqtt_connected": true,
  "px4_timestamp_us": 123456789,
  "px4_timestamp_valid": true,
  "received_at_unix_ms": 1786624212000,
  "status_age_ms": 37,
  "arming_state": 1,
  "arming_state_name": "DISARMED",
  "armed": false,
  "nav_state": 0,
  "failsafe": false,
  "failsafe_and_user_took_over": false,
  "pre_flight_checks_pass": true,
  "gcs_connection_lost": false,
  "is_vtol": false,
  "system_id": 1,
  "component_id": 1,
  "armed_time": 0,
  "takeoff_time": 0,
  "landed": true,
  "plan_management_allowed": true,
  "plan_management_message": "Planner management is allowed while PX4 is DISARMED"
}
```

Field meanings and nullability:

| Field | Type | Meaning |
|---|---|---|
| `source` | string | Always `px4` for this payload |
| `topic` | string | Always `/fmu/out/vehicle_status_v1` |
| `sequence` | number | Bridge-local status payload sequence |
| `status_state` | string | `UNKNOWN`, `FRESH`, `STALE`, or `OFFLINE`; see below |
| `mqtt_connected` | boolean | Bridge MQTT connection state when the payload was generated |
| `px4_timestamp_us` | number or null | PX4 `VehicleStatus.timestamp`; null if no sample/offline |
| `px4_timestamp_valid` | boolean | True only when the PX4 timestamp is non-zero |
| `received_at_unix_ms` | number or null | Host wall-clock time the bridge accepted the sample |
| `status_age_ms` | number or null | Monotonic age since the bridge accepted the sample |
| `arming_state` | number or null | PX4 arming-state enum value |
| `arming_state_name` | string | `DISARMED`, `ARMED`, or `UNKNOWN` |
| `armed` | boolean or null | Null with no status; true only for PX4 `ARMED` |
| `nav_state` | number or null | PX4 navigation-state enum value |
| `failsafe` | boolean or null | PX4 failsafe flag |
| `failsafe_and_user_took_over` | boolean or null | PX4 failsafe/user-takeover flag |
| `pre_flight_checks_pass` | boolean or null | PX4 preflight-check result |
| `gcs_connection_lost` | boolean or null | PX4 GCS connection flag |
| `is_vtol` | boolean or null | PX4 VTOL flag |
| `system_id` | number or null | PX4 system ID |
| `component_id` | number or null | PX4 component ID |
| `armed_time` | number or null | PX4 armed timestamp/value |
| `takeoff_time` | number or null | PX4 takeoff timestamp/value |
| `landed` | boolean or null | From `VehicleLandDetected`; null until a sample arrives |
| `plan_management_allowed` | boolean | True only for fresh, disarmed status |
| `plan_management_message` | string | Human-readable fail-closed explanation |

If the bridge has not received a `VehicleStatus`, the state is `UNKNOWN` and
PX4 fields other than `source`, `topic`, `sequence`, `status_state`,
`mqtt_connected`, `px4_timestamp_valid`, `landed`,
`plan_management_allowed`, and `plan_management_message` are `null` as
specified above. If a message was received with timestamp `0`, the bridge
reports `STALE`, `px4_timestamp_valid: false`, and keeps the received fields;
it does not treat that message as safe. A clean bridge shutdown publishes an
`OFFLINE` PX4 snapshot, but `<prefix>/px4/status` is **not** the MQTT Last Will
topic. After an unclean bridge loss, its retained PX4 topic can remain the last
live snapshot (including `mqtt_connected: true`). Use the retained planner
status and its `OFFLINE` Last Will as the authoritative offline signal.

The default `px4_status_timeout_s` is `2.0`. `FRESH` means the latest accepted
sample has a non-zero timestamp and `status_age_ms` is at most that timeout.
`STALE` means a sample exists but its timestamp is invalid or its age exceeds
the timeout. The bridge normally ignores delayed/out-of-order samples whose
PX4 timestamp is older than or equal to the last accepted timestamp. It has a
reboot heuristic: a non-zero timestamp below `10,000,000` microseconds is
accepted as a likely PX4 reboot and can replace the previous sample. Treat the
resulting status as valid only after the new sample has a non-zero timestamp and
continues updating within the timeout.

`status_state: "FRESH"` does **not** mean disarmed. An armed vehicle can have a
fresh PX4 status; use `<prefix>/planner/status` for the plan-control gate.

## Planner status and state semantics

Subscribe to `<prefix>/planner/status` for the dashboard safety state:

```json
{
  "source": "search_bridge",
  "topic": "search/planner/status",
  "status_state": "DISARMED",
  "status_age_ms": 37,
  "arming_state": 1,
  "arming_state_name": "DISARMED",
  "plan_management_allowed": true,
  "active_working_plan": "20260813T143012Z_aavc2026_mission.plan",
  "last_error": "",
  "message": "Planner management is allowed while PX4 is DISARMED"
}
```

`topic` contains the expanded configured topic, so it will be
`<prefix>/planner/status` at runtime. `active_working_plan` can be an empty
string before the planner has initialized or if no working plan is available.

| Planner `status_state` | Meaning | Enable plan controls? |
|---|---|---:|
| `UNKNOWN` | No PX4 status has arrived | No |
| `STALE` | PX4 sample exists but is too old or has an invalid timestamp | No |
| `NOT_DISARMED` | PX4 status is fresh but arming state is not `DISARMED` | No |
| `DISARMED` | PX4 status is fresh and arming state is `DISARMED` | Only if `plan_management_allowed === true` |
| `OFFLINE` | MQTT bridge is disconnected; generated by orderly shutdown or Last Will | No |

The `plan_management_allowed` boolean is the machine-readable gate. The
required UI condition is both:

```js
plannerStatus.status_state === "DISARMED" &&
plannerStatus.plan_management_allowed === true
```

The bridge independently enforces the same condition immediately before
listing or resetting plans.

## Plans payload and dropdown behavior

Subscribe to `<prefix>/planner/plans` and use the `plans` array for a dropdown:

```json
{
  "source": "search_bridge",
  "published_at_unix_ms": 1786624212000,
  "default_manual_plan": "aavc2026_mission.plan",
  "plans": ["aavc2026_mission.plan", "area_b.plan"],
  "active_working_plan": "20260813T143012Z_aavc2026_mission.plan",
  "available": true,
  "message": "Manual plans are available"
}
```

Rules for a dashboard:

1. Populate options only from `msg.payload.plans`; do not construct paths or
   allow arbitrary text to become a path.
2. Show `default_manual_plan` as the initial selection only when it is present
   in the returned list.
3. Display `active_working_plan` as read-only status; it is a generated working
   filename, not a manual-plan selection.
4. Disable the dropdown/reset controls while the planner status is
   `UNKNOWN`, `STALE`, `NOT_DISARMED`, or `OFFLINE`.
5. `available: true` means the bridge completed its refresh/list operation; the
   array may still be empty when there are no valid source files or the manual
   directory is not a directory. `available: false` means an exception occurred
   during planner refresh or listing. Render `message` for diagnosis in either
   case and do not send a reset when the list is empty.
6. A retained plans message can be old. It is useful for rendering but is not
   an authorization signal.

The bridge refreshes plans after connecting and periodically only while the PX4
status is fresh and disarmed. It does not scan or modify plan files while the
vehicle is armed or while status is unknown/stale. During an allowed refresh,
planner initialization may create the default timestamped working copy and
`.active_working_plan` marker if no valid working copy exists; listing is not a
promise that the working directory remains untouched.

## Planner commands and responses

The command topic is `<prefix>/planner/command`. Every command needs a new,
non-empty string `request_id` and a string `command`.

### List plans

```json
{
  "request_id": "req-001",
  "command": "list_manual_plans"
}
```

This command is also guarded by the fresh-DISARMED check. A successful response
means the bridge completed its refresh/list operation and attempted to publish
the plans payload; it does not confirm broker delivery or a subscriber receipt:

```json
{
  "request_id": "req-001",
  "success": true,
  "message": "Manual plans published",
  "responded_at_unix_ms": 1786624212000
}
```

### Reset the working plan

```json
{
  "request_id": "req-002",
  "command": "reset_working_plan",
  "plan_name": "aavc2026_mission.plan"
}
```

`plan_name` must be a safe manual `.plan` basename whose source file is
currently valid. Normally choose it from the latest `plans` payload, but the
bridge does not enforce membership in that previous snapshot; a newly added
valid manual file can be accepted before the next list refresh. The bridge
validates the source file before copying it. A successful reset creates a new
working file, clears resume metadata by starting from the source plan, and
returns its generated filename:

```json
{
  "request_id": "req-002",
  "success": true,
  "message": "Working plan reset from aavc2026_mission.plan",
  "responded_at_unix_ms": 1786624212000,
  "active_working_plan": "20260813T143012Z_aavc2026_mission.plan"
}
```

Rejection responses keep the request ID when it was validly parsed. The
message identifies the fail-closed reason:

| Condition | `message` |
|---|---|
| No `VehicleStatus` yet | `PX4 VehicleStatus is unknown; planner management is disabled` |
| Sample timestamp invalid or too old | `PX4 VehicleStatus is stale; planner management is disabled` |
| Fresh status but not disarmed | `Planner management is allowed only while PX4 is DISARMED` |

For example, a fresh-but-armed rejection is:

```json
{
  "request_id": "req-003",
  "success": false,
  "message": "Planner management is allowed only while PX4 is DISARMED"
}
```

Malformed JSON, a non-object payload, missing/empty `request_id`, a non-string
`command`, an invalid `plan_name`, an unsupported command, an unavailable plan,
and an invalid plan file are rejected. Only these commands are implemented:

```text
list_manual_plans
reset_working_plan
```

The response topic is `<prefix>/planner/response`, QoS 1, non-retained. Match
`request_id` before updating the UI. Do not infer success from a plans-message
change alone.

### Reset timeout and retry warning

A reset writes a new file and publishes state; it is not an idempotent operation
across bridge restarts. If Node-RED times out waiting for a response:

- Do not automatically resend with a new request ID. The first reset may have
  succeeded, and a new ID can create another working copy.
- First inspect the latest plans/status messages and the filesystem/operator
  state, then decide whether a retry is necessary.
- Reusing the same request ID can replay a cached response while the same bridge
  process is alive **and the ID has not been evicted from the 256-entry cache**,
  but the cache is in memory and is lost after a bridge restart. It is not a
  durable transaction log.
- Use a longer response timeout than the normal MQTT round trip and show an
  explicit `unknown outcome` state instead of silently repeating a reset.

`list_manual_plans` does not modify the manual source files, but its planner
refresh can initialize the working directory and create the default working
copy if none exists. It is still rejected unless the vehicle state is fresh
and disarmed.

## Node-RED/dashboard implementation

### Broker node setup

Configure one MQTT broker connection in Node-RED with:

- Server: the HiveMQ cluster hostname, without a URL scheme.
- Port: `8883`.
- TLS: enabled with normal server certificate verification.
- Username/password: stored in Node-RED's credential store or another host-only
  secret mechanism.
- Client ID: unique and different from `search_bridge`.

Create MQTT-in nodes for:

```text
<prefix>/px4/status
<prefix>/planner/plans
<prefix>/planner/status
<prefix>/planner/response
```

Create one MQTT-out node for `<prefix>/planner/command`. Configure it for QoS 1
and retain `false`. Use a JSON node or parse JSON in a Function node. Keep the
original `msg.retain` metadata available on status messages so the dashboard can
distinguish a broker-delivered retained snapshot from a live update.

### Recommended safety state machine

Maintain a single flow-level safety state from planner-status messages:

```text
START / no live planner status -> DISABLED
retained planner status         -> display only; remain DISABLED until live update
UNKNOWN / STALE / NOT_DISARMED  -> DISABLED
DISARMED + allowed=true         -> ENABLED for a short local timeout
OFFLINE                         -> DISABLED immediately
no live update before timeout   -> DISABLED
```

A retained `DISARMED` message may be displayed immediately, but it must not
start the local authorization timer. The bridge publishes planner status every
`planner_publish_period_ms` (default `2000` ms), so a dashboard should use a
short local watchdog. The following Function-node example uses `3000` ms; tune
it for the deployment, but keep it fail-closed and never use it to weaken the
bridge's `px4_status_timeout_s` check.

**Function node: process `<prefix>/planner/status`**

```js
const p = (typeof msg.payload === "string")
  ? JSON.parse(msg.payload)
  : msg.payload;

if (!p || typeof p !== "object") {
  return null;
}

const now = Date.now();
const retained = msg.retain === true;
let state = flow.get("plannerSafety") || {
  displayPayload: null,
  livePayload: null,
  lastLiveAt: 0,
  bridgeOffline: false
};

state.displayPayload = p;

// OFFLINE must fail closed even if the broker marks the Last Will retained.
if (p.status_state === "OFFLINE") {
  state.bridgeOffline = true;
  state.livePayload = p;
  state.lastLiveAt = now;
} else if (retained) {
  // Never carry a previous live authorization across a retained snapshot.
  state.bridgeOffline = false;
  state.livePayload = null;
  state.lastLiveAt = 0;
  state.allowed = false;
} else {
  state.bridgeOffline = false;
  state.livePayload = p;
  state.lastLiveAt = now;
}

const live = state.livePayload;
const liveRecently = live !== null &&
  (now - state.lastLiveAt) <= 3000;
state.allowed = !state.bridgeOffline && liveRecently &&
  live.status_state === "DISARMED" &&
  live.plan_management_allowed === true;

flow.set("plannerSafety", state);
msg.plannerSafety = {
  allowed: state.allowed,
  statusState: p.status_state,
  retained: retained,
  lastLiveAt: state.lastLiveAt
};
return msg;
```

Wrap `JSON.parse` in a `try/catch` in production if malformed messages can
reach the Function node. Connect the `allowed` value to the button/dropdown
enabled state, but keep the bridge response as the final result.

Use an Inject node every 500 ms to run a watchdog Function. It must set the UI
to disabled when no live planner-status update arrived before the local timeout:

```js
const state = flow.get("plannerSafety");
if (!state) {
  return null;
}

if (state.allowed && Date.now() - state.lastLiveAt > 3000) {
  state.allowed = false;
  flow.set("plannerSafety", state);
  return {
    payload: {
      type: "planner_safety_timeout",
      message: "No live planner status update; plan controls disabled"
    }
  };
}
return null;
```

### Build a guarded reset command

The UI should store only the selected basename, for example in
`flow.selectedManualPlan`. Before an MQTT-out node, use a Function node like
this:

```js
const state = flow.get("plannerSafety");
const planName = flow.get("selectedManualPlan");
const now = Date.now();

if (!state || state.allowed !== true ||
    now - state.lastLiveAt > 3000) {
  node.warn("Reset blocked: planner is not currently DISARMED");
  return null;
}

if (typeof planName !== "string" ||
    !/^[^/\\]+\\.plan$/.test(planName)) {
  node.warn("Reset blocked: select a manual .plan basename");
  return null;
}

const requestId = `${now.toString(36)}-${Math.random().toString(36).slice(2)}`;
flow.set("pendingPlannerRequest", {
  requestId,
  command: "reset_working_plan",
  sentAt: now,
  planName
});

msg.topic = "search/planner/command";  // replace search when prefix changes
msg.qos = 1;
msg.retain = false;
msg.payload = JSON.stringify({
  request_id: requestId,
  command: "reset_working_plan",
  plan_name: planName
});
return msg;
```

Set the topic from one configuration value rather than hard-coding it in a
production flow. The regular expression is only a UI check; the bridge still
performs the authoritative validation. Use an equivalent Function for
`list_manual_plans`, omitting `plan_name`.

### Correlate responses

On `<prefix>/planner/response`, parse the JSON and compare
`payload.request_id` with the pending request. Display `success` and `message`,
then clear the pending request. Ignore responses for other requests. Add a
response timeout (for example 5–10 seconds) that reports `unknown outcome` and
disables the controls; do not silently issue a new reset with a new ID.

Always subscribe to the response topic before enabling the dashboard button.
Responses are non-retained and can be missed by a client that reconnects after
the operation.

## Start Gazebo, PX4, ROS 2, Search, and the bridge

`common.launch.py` starts ROS support nodes and `MicroXRCEAgent`. It does **not**
start Gazebo, PX4, `search`, or `search_bridge`. Use separate terminals in this
order inside the container. Source the environments in each terminal if the
shell did not already source them:

```sh
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash
cd /home/ubuntu/roscon-25-workshop_ws
```

### Terminal 1: Gazebo

With GUI:

```sh
./gazebo_models/run_world.sh kmitl_airfield
```

For a headless container:

```sh
./gazebo_models/run_world.sh kmitl_airfield --headless
```

Wait until the Gazebo server is running before starting PX4.

### Terminal 2: PX4 SITL

Start the x500 PX4 instance after Gazebo is ready:

```sh
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_PARAM_UXRCE_DDS_SYNCT=0 \
/home/ubuntu/px4_sitl/bin/px4 \
  -w /home/ubuntu/px4_sitl/romfs
```

Wait for PX4 to start its simulator and DDS client. QGroundControl should be
connected before normal arming. For a headless container, connect a host QGC to
UDP `127.0.0.1:18570`.

### Terminal 3: common ROS 2 launch

```sh
ros2 launch px4_roscon_25 common.launch.py
```

This starts the clock bridge, robot-state publisher, TF publisher, Foxglove
bridge, static transform, and one `MicroXRCEAgent udp4 -p 8888`. Do not start a
second agent on the same UDP port.

### Terminal 4: Search mode node

```sh
CFG="$(ros2 pkg prefix search)/share/search/config/search_params.yaml"
ros2 run search search \
  --ros-args \
  --params-file "$CFG"
```

This registers the PX4 custom mode named `Search`. Select it through the normal
PX4/QGroundControl workflow only after the vehicle and mission are ready.

### Terminal 5: MQTT bridge

Set the broker hostname without putting credentials on the command line:

```sh
CFG="$(ros2 pkg prefix search)/share/search/config/search_params.yaml"
MQTT_HOST=your-cluster-hostname

ros2 run search search_bridge \
  --ros-args \
  --params-file "$CFG" \
  -p "mqtt_host:=${MQTT_HOST}"
```

If the YAML already contains the correct `mqtt_host`, the `-p` override can be
omitted. Do not pass `mqtt_password` or `mqtt_username` through `ros2 run`.
The host parameter is not a secret; the credentials must come from the
container environment file. On success, the log includes the configured PX4
source topic and MQTT topics. On startup with an empty `mqtt_host`, the exact
behavior is:

```text
mqtt_host is empty; search_bridge will run ROS subscriptions but will not connect to MQTT
```

The ROS bridge can therefore be run in ROS-only mode, but no MQTT state or
commands will be available until a host and both credentials are configured.

## Search flight and resume behavior

The parser reads the QGroundControl `mission` object, optional `cruiseSpeed`,
the first `CameraCalc.DistanceToSurface`, and nested waypoint items with
`command == 16`. `cruiseSpeed` is informational; actual speed is controlled by
`max_horizontal_speed_m_s`. Search converts global latitude/longitude to local
NED coordinates, climbs to `search_altitude_m`, flies the entry point and the
remaining route, and holds at the final waypoint.

The relevant Search parameters in `config/search_params.yaml` are:

| Parameter | Default | Use |
|---|---:|---|
| `search_altitude_m` | `15.0` | Flight altitude; a non-positive value uses the plan altitude |
| `max_horizontal_speed_m_s` | `5.0` | Maximum horizontal speed |
| `waypoint_reach_radius_m` | `4.0` | Radius for considering a waypoint reached |
| `max_yaw_rate_rad_s` | `2.0` | Maximum yaw rate |
| `default_manual_plan` | `aavc2026_mission.plan` | Source selected when no valid working copy exists |
| `reset_working_plan` | `false` | Create a fresh working copy during planner initialization |

The automatic deactivate checkpoint preserves progress. A deliberate
`reset_working_plan` command creates a fresh working copy and therefore starts
from the beginning of the selected source plan on the next activation.

## Troubleshooting

### MQTT does not connect

- Check that the host is a hostname only, the port is `8883`, and DNS/network
  access from the container works.
- Confirm the credential file existed before the container was created and that
  both environment variables are non-empty inside the container.
- Ensure the broker account permits the bridge client ID and the exact topic
  directions in the ACL table.
- Check the bridge log for `Connecting to MQTT broker ... with TLS` and the
  subsequent connection result. The bridge never logs the password.
- If `mqtt_host` is empty, ROS subscriptions continue but MQTT is intentionally
  disabled.

### TLS or certificate verification fails

- Use the HiveMQ hostname as SNI; do not use an IP address or URL scheme.
- Confirm the container clock is correct and the system CA bundle is present.
- Leave `mqtt_tls_ca_file` empty for the normal system CA bundle. Set a custom
  CA file only when the deployment requires one and make sure the path exists
  inside the container.
- The bridge always calls certificate verification and has no insecure fallback.

### Authentication or ACL failure

- Username/password authentication is supported; mTLS client certificates are
  not supported by this executable.
- Make sure both credentials are present. A configured broker host with missing
  credentials produces `MQTT credentials are required when mqtt_host is configured`.
- Use distinct client IDs for the bridge and Node-RED. A duplicate MQTT client
  ID can disconnect the previous connection.
- Verify that the bridge can publish the planner-status Last Will topic and
  subscribe to the command topic; missing either permission can make the bridge
  appear connected but unusable.

### No `/fmu/out/vehicle_status_v1` messages

Check the startup order and the DDS agent:

```sh
ros2 topic list | grep '/fmu/out/vehicle_status_v1'
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort
ros2 topic info /fmu/out/vehicle_status_v1 -v
```

The bridge subscribes to `/fmu/out/vehicle_status_v1`. Ensure
Gazebo and PX4 are running, `common.launch.py` started the single
`MicroXRCEAgent` on UDP `8888`, and PX4 reports a connected DDS client.

### Planner status remains `UNKNOWN` or `STALE`

- `UNKNOWN` means no `VehicleStatus` sample has reached the bridge.
- `STALE` means the sample timestamp is zero/invalid or no newer sample arrived
  within `px4_status_timeout_s` (default `2.0` seconds).
- Verify that the PX4 topic is publishing with a non-zero `timestamp` and that
  the ROS/DDS connection is not being interrupted.
- Restarting PX4 can reset its timestamp; wait for new valid samples rather than
  treating a retained status snapshot as authorization.

### Planner status is `NOT_DISARMED`

This is expected while PX4 is armed or reports another arming state. Disarm
through the normal operator workflow and wait for a fresh `DISARMED` status.
MQTT cannot override this gate.

### Plans are unavailable or the dropdown is empty

- Check that `manual_plan_directory` exists and is readable by the container
  user, and that `working_plan_directory` exists or can be created.
- Confirm `manual_plan_directory` and `working_plan_directory` are identical
  for `search` and `search_bridge`.
- Only regular `.plan` files with valid mission data and without
  `searchPlanner` resume metadata appear in `plans`.
- Inspect the bridge `message` and `last_error` fields; parser failures are
  intentionally excluded from the selectable list.
- Use `ros2 pkg prefix search` to confirm which installed package share is being
  used when empty directory parameters resolve unexpectedly.

### A command produces no response

- Subscribe to `<prefix>/planner/response` before sending the command; the
  response is non-retained.
- Verify the command topic prefix, the Node-RED MQTT client ID, and the bridge
  ACL/subscription.
- Confirm the command was published with QoS 1 and retain disabled. The bridge
  intentionally ignores retained commands.
- Ensure the JSON has a non-empty `request_id` and one of the two supported
  command names. Match the response request ID in Node-RED.
- For reset timeouts, follow the explicit retry warning above; do not blindly
  create another working copy.

### Node-RED controls are disabled after reconnect

This is the safe behavior. A retained plans/PX4 snapshot is display-only, and
the local watchdog waits for a live planner-status update. Check that the
Node-RED broker node receives `<prefix>/planner/status`, that its client ID is
unique, and that the bridge is not publishing `OFFLINE`.

## Configuration reference

The ROS parameter file contains sections for both executables. Parameters that
must be kept consistent are shown together:

| Parameter | `search` | `search_bridge` | Notes |
|---|---:|---:|---|
| `manual_plan_directory` | yes | yes | Use the same path in both nodes |
| `working_plan_directory` | yes | yes | Use the same path in both nodes |
| `default_manual_plan` | yes | yes | Safe `.plan` basename |
| `reset_working_plan` | yes | yes | Normally `false`; creates a fresh copy when enabled |
| `mqtt_host` | no | yes | Hostname only; empty disables MQTT |
| `mqtt_port` | no | yes | Default `8883` |
| `mqtt_username` | no | yes | Leave empty to use `HIVEMQ_MQTT_USERNAME` |
| `mqtt_password` | no | yes | Leave empty to use `HIVEMQ_MQTT_PASSWORD` |
| `mqtt_client_id` | no | yes | Default `search_bridge`; must be unique |
| `mqtt_topic_prefix` | no | yes | Default `search`; no MQTT wildcards |
| `mqtt_tls_ca_file` | no | yes | Empty uses `/etc/ssl/certs` |
| `px4_status_timeout_s` | no | yes | Default `2.0` seconds |
| `status_publish_period_ms` | no | yes | Default `500` ms; minimum `100` ms |
| `planner_publish_period_ms` | no | yes | Default `2000` ms; minimum `100` ms |

If a parameter is changed in a deployment-specific YAML file, keep the topic
prefix, plan directories, broker ACLs, and Node-RED topics aligned. Never use a
cached dashboard value to bypass the bridge's safety check.
