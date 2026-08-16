# `aruco_database_bridge`

`aruco_database_bridge` is a ROS 2 to MQTT bridge for the
[`aruco_database`](../aruco_database/README.md) package. It is intended for a
Node-RED dashboard that must display every known ArUco marker in real time and
start a new training area without opening or editing the YAML database file.

The bridge keeps `aruco_database` as the only source of truth:

- `aruco_database` owns the in-memory marker map and `markers.yaml`.
- `aruco_database` publishes ROS 2 marker/status snapshots and implements the
  list and clear services.
- `aruco_database_bridge` converts those snapshots to JSON and publishes them
  to MQTT.
- Node-RED sends management commands to MQTT; the bridge validates and queues
  them, then calls the ROS 2 services on the ROS executor thread.

The bridge does **not** detect ArUco markers, transform camera poses, write
YAML directly, control PX4, arm the vehicle, or provide a generic ROS command
gateway.

## Architecture

```text
aruco_tracker
      |
      | /aruco/detections
      v
aruco_database
  - TF/WGS84 conversion
  - in-memory marker map
  - atomic YAML persistence
  - ROS marker/status topics
  - ROS list/get/clear services
      |
      | ROS 2
      v
aruco_database_bridge
  - subscribes to retained ROS snapshots
  - publishes retained MQTT JSON snapshots
  - queues Node-RED MQTT commands
  - calls list/get/clear services asynchronously
      |
      | MQTT over verified TLS
      v
Node-RED dashboard
```

The ROS topics used by the bridge are fixed management interfaces:

```text
/aruco_database/markers  aruco_database/msg/ArucoMarkerArray
/aruco_database/status   aruco_database/msg/ArucoDatabaseStatus
```

The bridge uses reliable, transient-local ROS QoS for these topics so that a
bridge started after the database node receives the latest snapshot. The
existing database input and Direct compatibility service are unchanged:

```text
/aruco/detections                 aruco_database/msg/ArucoDetectionArray
/aruco_database/get_position      aruco_database/srv/GetArucoPosition
```

## MQTT prerequisites

The workshop image provides ROS 2 Humble, `colcon`, `libmosquitto`, and the
Mosquitto client tools. A deployment needs:

- a compatible MQTT broker;
- TLS on the configured port, normally `8883`;
- a broker account with ACLs for the selected topic prefix;
- a unique MQTT client ID for the bridge;
- a Node-RED MQTT client ID different from the bridge client ID.

The bridge uses the Mosquitto C client with server certificate verification.
It supports username/password authentication. Mutual TLS client certificates
are not implemented by this executable.

### Keep credentials outside the repository

Leave `mqtt_username` and `mqtt_password` empty in the ROS parameter file and
provide credentials through the environment:

```text
HIVEMQ_MQTT_USERNAME=<broker-username>
HIVEMQ_MQTT_PASSWORD=<broker-password>
```

For a host credential file, use a private file with mode `600` and pass it to
the container before it is created. Do not put credentials in this repository,
a Dockerfile, a launch command, a README, an MQTT payload, or a normal log
message.

The bridge reads the environment only when the corresponding ROS parameter is
empty. If `mqtt_host` is configured, both credentials must be present. If only
one credential is available, startup fails closed.

## Build

Build the core and bridge together inside the workshop container:

```sh
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
cd /home/ubuntu/roscon-25-workshop_ws

colcon build \
  --packages-select aruco_database aruco_database_bridge \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DBUILD_TESTING=ON

source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash
```

The bridge package installs its executable, public header, configuration,
launch file, and this README below its package share directory:

```sh
ros2 pkg prefix aruco_database_bridge
```

If `libmosquitto-dev` is not available in another deployment image, install
the development package before building and the matching runtime library
before running. The CMake target discovers Mosquitto through `pkg-config`.

## Configuration

The installed parameter file is:

```text
$(ros2 pkg prefix aruco_database_bridge)/share/aruco_database_bridge/config/params.yaml
```

Default parameters:

| Parameter | Default | Meaning |
|---|---:|---|
| `use_sim_time` | `true` | Use the ROS simulation clock for ROS message timestamps |
| `mqtt_host` | empty | Broker hostname without `mqtt://`, `mqtts://`, or `https://`; empty disables MQTT |
| `mqtt_port` | `8883` | Broker TLS port |
| `mqtt_username` | empty | Username; empty uses `HIVEMQ_MQTT_USERNAME` |
| `mqtt_password` | empty | Password; empty uses `HIVEMQ_MQTT_PASSWORD` |
| `mqtt_client_id` | `aruco_database_bridge` | Unique MQTT client ID |
| `mqtt_topic_prefix` | `aruco_database` | Concrete prefix; MQTT wildcards are rejected |
| `mqtt_tls_ca_file` | empty | Optional CA file; empty uses `/etc/ssl/certs` |
| `status_publish_period_ms` | `1000` | Periodic retained status publish; minimum 100 ms |
| `core_status_timeout_ms` | `5000` | Maximum age of a core status heartbeat before `core_available` becomes false |

The bridge can run in ROS-only mode when `mqtt_host` is empty. It continues to
subscribe to the database topics and logs that MQTT is disabled. No MQTT
commands or MQTT snapshots are available in that mode.

## Start the nodes

Start the normal PX4, TF, camera, and tracker workflow first. Then start the
core database node:

```sh
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash

ros2 launch aruco_database aruco_database.launch.py
```

Start the bridge in a separate terminal. The broker host is a hostname only:

```sh
CFG="$(ros2 pkg prefix aruco_database_bridge)/share/aruco_database_bridge/config/params.yaml"
MQTT_HOST=your-broker-hostname

ros2 launch aruco_database_bridge aruco_database_bridge.launch.py \
  mqtt_host:="${MQTT_HOST}"
```

The launch file accepts all parameters as arguments. For example:

```sh
ros2 launch aruco_database_bridge aruco_database_bridge.launch.py \
  mqtt_host:=your-broker-hostname \
  mqtt_client_id:=aruco_database_bridge_vehicle_1 \
  mqtt_topic_prefix:=aruco_database
```

Do not pass passwords in the command line. Use the injected environment
variables or a deployment secret store.

## MQTT topic contract

With the default prefix, the bridge uses these topics:

| Topic | Direction | QoS | Retained | Purpose |
|---|---|---:|---:|---|
| `aruco_database/markers` | bridge → Node-RED | 1 | yes | Full marker snapshot for the dashboard |
| `aruco_database/status` | bridge → Node-RED | 1 | yes | Database, persistence, core, and bridge status |
| `aruco_database/event` | bridge → Node-RED | 1 | no | Non-retained notifications such as clear and persistence transitions |
| `aruco_database/command` | Node-RED → bridge | 1 | **no** | `status`, `refresh`, `get_marker`, and `clear_file` commands |
| `aruco_database/response` | bridge → Node-RED | 1 | **no** | Response correlated by `request_id` |

The bridge rejects retained messages received on the command topic. Configure
the Node-RED MQTT-out node with QoS 1 and retain disabled. A retained command
could be replayed after a reconnect, which is unsafe for `clear_file`.

The marker and status topics are retained so a dashboard can render immediately
after connecting. The retained values are display snapshots; Node-RED should
still use the status state and a local timeout before enabling destructive
controls.

The bridge publishes an MQTT Last Will on the retained status topic:

```json
{
  "schema": "aruco_database.status.v1",
  "state": "offline",
  "bridge_online": false,
  "core_available": false,
  "persistence_ok": false,
  "file_state": "offline"
}
```

An orderly shutdown publishes the same offline status before disconnecting.

## Marker snapshot payload

`aruco_database/markers` contains the complete sorted marker list, not just the
last marker. This means Node-RED can replace its table from one message and
cannot retain a deleted per-ID topic accidentally:

```json
{
  "schema": "aruco_database.markers.v1",
  "source": "aruco_database_bridge",
  "published_at_unix_ms": 1786624212000,
  "core_available": true,
  "revision": 21,
  "markers": [
    {
      "id": 1,
      "latitude_deg": 13.000000,
      "longitude_deg": 103.000000,
      "observation_count": 12
    },
    {
      "id": 2,
      "latitude_deg": 13.000321,
      "longitude_deg": 103.000512,
      "observation_count": 8
    }
  ]
}
```

The core publishes a new ROS snapshot after an accepted detection batch. The
bridge forwards it as soon as the MQTT connection is available. When the
running average changes, the latitude/longitude in the next snapshot changes;
`revision` increments once per changed detection message and is persisted by the
core. The value is an opaque generation token, and legacy or newly created
files receive a new generation before the first save. A clear publishes an
empty retained snapshot:

```json
{
  "schema": "aruco_database.markers.v1",
  "revision": 22,
  "core_available": true,
  "markers": []
}
```

Persistence state is carried by the retained `aruco_database/status` topic,
not embedded in the marker snapshot. This prevents a marker payload from
showing an old `pending` or `error` value after a later save retry.

There are no required MQTT topics such as `markers/1`, `markers/2`, and so on.
A single retained snapshot is easier for Node-RED to render and guarantees that
the table represents one consistent database revision.

## Status payload

A normal status payload looks like this:

```json
{
  "schema": "aruco_database.status.v1",
  "source": "aruco_database_bridge",
  "published_at_unix_ms": 1786624212000,
  "bridge_online": true,
  "core_available": true,
  "state": "ready",
  "revision": 21,
  "marker_count": 2,
  "origin_ready": true,
  "database_dirty": false,
  "persistence_ok": true,
  "file_state": "synced",
  "last_error": ""
}
```

Possible high-level states include:

| State | Meaning |
|---|---|
| `core_unavailable` | No fresh core status has arrived within `core_status_timeout_ms` (or none has arrived yet) |
| `waiting_for_origin` | The core is alive but cannot accept new detections yet |
| `ready_empty` | The database is available and currently has no marker |
| `ready` | The database has one or more markers |
| `persistence_error` | The core could not save the YAML file |
| `offline` | The MQTT bridge connection has gone away |

`core_available` is based on the age of the last ROS status message, not merely
whether the bridge has ever received one. If no fresh status arrives within
`core_status_timeout_ms` (5 seconds by default), the retained status changes to
`core_unavailable` and Node-RED must disable clear controls until the core
heartbeat returns.

`file_state` is derived from the core status:

- `synced`: the in-memory map is not dirty and the last persistence result was
  successful;
- `pending`: a valid update exists that has not been written yet;
- `error`: the last save failed and the core will retry;
- `unknown`/`offline`: the bridge does not have a usable core state.

## Events and responses

Events are useful for notifications but are not the source of truth for the
Node-RED table:

```json
{
  "schema": "aruco_database.event.v1",
  "event": "database_cleared",
  "message": "Persistent database cleared",
  "published_at_unix_ms": 1786624212000
}
```

The bridge also emits `persistence_error` and `persistence_recovered` on
transitions reported by the core status topic. Status remains the authoritative
source for the current persistence state; events are notifications only.

Every command response contains the original `request_id`:

```json
{
  "schema": "aruco_database.response.v1",
  "request_id": "node-red-002",
  "command": "clear_file",
  "success": true,
  "message": "Database cleared successfully",
  "responded_at_unix_ms": 1786624212000,
  "revision": 22,
  "marker_count": 0,
  "persisted": true,
  "backup_file": "/path/markers.yaml.backup-20260814T120500Z-123"
}
```

Responses are not retained. Node-RED must subscribe before publishing a
command. The bridge keeps up to 256 completed request IDs during one process
lifetime. Reusing a cached request ID replays its response instead of executing
the command again. A duplicate ID that is still in flight receives a transient
`already being processed` response and does not cancel or complete the
original request. In-flight requests remain reserved across an MQTT
reconnect; the cache is not durable and is lost when the bridge restarts.

## Commands

All commands must be a JSON object with a non-empty string `request_id` and a
string `command`.

### Publish status again

```json
{
  "request_id": "node-red-001",
  "command": "status"
}
```

### Refresh the marker snapshot

Both `refresh` and `list` are accepted aliases:

```json
{
  "request_id": "node-red-002",
  "command": "refresh"
}
```

The bridge calls `/aruco_database/list_markers`, updates its cached snapshot,
and publishes `aruco_database/markers`.

### Query one marker

```json
{
  "request_id": "node-red-003",
  "command": "get_marker",
  "aruco_id": 5
}
```

This calls the existing `/aruco_database/get_position` service. The response
contains `found`, `latitude_deg`, and `longitude_deg` when the ID exists.
Normal dashboards should use the retained full snapshot instead of polling
one marker at a time.

### Clear persistent database for a new practice area

```json
{
  "request_id": "node-red-004",
  "command": "clear_file",
  "confirm": true,
  "backup": true,
  "use_expected_revision": true,
  "expected_revision": 21
}
```

The bridge calls `/aruco_database/clear`. `confirm: true` is required. The
optional `backup: true` asks the core to persist the active in-memory snapshot
first, then copy it to a unique timestamped backup before clearing it.
`expected_revision` prevents a stale Node-RED page from clearing a database
that changed after the page was loaded, including across a core restart because
the revision generation is persisted.

The operation is deliberately implemented as a coordinated database reset:

1. validate the confirmation and expected revision;
2. if backup is requested, persist the active snapshot and create a unique backup;
3. clear the active in-memory map;
4. atomically and durably write the new revision plus `markers: []` to the configured YAML file;
5. publish an empty retained marker snapshot and a synced status.

Deleting only the file while leaving the core's in-memory map unchanged would
cause the next save to recreate the old markers. For that reason Node-RED's
`clear_file` operation is the safe "clear persistent file and start a new
session" operation. The core owns this behavior; the bridge never deletes the
file itself.

If persistence fails, the response has `success: false` and `persisted: false`.
The map may already be empty in memory. When the failure occurs before the
atomic rename, the previous on-disk file remains intact; the core reports
`file_state: error` and keeps the map dirty for retry/diagnosis. Node-RED must
not delete the file manually.

## Core persistence behavior

The existing core timer remains available through `save_period_s`, but the
updated default also enables update-triggered persistence:

```yaml
save_on_update: true
save_min_interval_ms: 200
save_period_s: 2.0
```

The database publishes an in-memory/ROS/MQTT update immediately after an
accepted detection batch. New marker IDs force an immediate file write.
Repeated updates are written no more often than `save_min_interval_ms`; the
timer is the fallback for a dirty database. This prevents a camera running at
30 FPS from rewriting the YAML file 30 times per second while keeping the
power-loss window small. Writes use a sibling temporary file, flush and
`fsync` that file, atomically rename it over the database, and `fsync` the
parent directory. A failed save before the rename leaves the previous valid
file in place; the core reports the error and retries while the map remains
dirty.

The persistence interval is a trade-off between storage wear and the amount of
latest averaging that could be lost on an abrupt power loss. Set
`save_min_interval_ms` to `0` only when the storage can tolerate a write for
every accepted detection message. A clean shutdown still attempts a final save.

## Node-RED dashboard flow

A basic dashboard needs these MQTT-in nodes:

```text
aruco_database/markers
aruco_database/status
aruco_database/event
aruco_database/response
```

It needs one MQTT-out node:

```text
aruco_database/command
```

Configure the MQTT-out node with QoS 1 and retain disabled.

### Render the marker table

The marker message is already an array. A Function node can normalize it for a
Dashboard table:

```js
const p = typeof msg.payload === "string"
  ? JSON.parse(msg.payload)
  : msg.payload;

if (!p || !Array.isArray(p.markers)) {
    return null;
}

msg.payload = p.markers.map((m) => ({
    id: m.id,
    latitude: m.latitude_deg,
    longitude: m.longitude_deg,
    observations: m.observation_count
}));
msg.arucoRevision = p.revision;
return msg;
```

Connect this to a Dashboard table with columns `id`, `latitude`,
`longitude`, and `observations`. Do not append rows to a local list without
replacing the previous snapshot; otherwise a cleared marker can remain in the
UI.

### Clear button

Use the latest `revision` from the marker/status snapshot. The button should
be disabled while the bridge is offline, the core is unavailable, or a clear
request is pending. Before the MQTT-out node, use a confirmation dialog and
send:

```js
const status = flow.get("arucoDatabaseStatus");
const markers = flow.get("arucoDatabaseMarkers");
const pendingRequest = flow.get("arucoDatabasePendingRequest");

if (pendingRequest || !status || !markers ||
    typeof markers.revision !== "number" || status.bridge_online !== true ||
    status.core_available !== true ||
    status.state === "persistence_error") {
    node.warn("ArUco database is not ready for clear");
    return null;
}

const requestId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
msg.topic = "aruco_database/command";
msg.qos = 1;
msg.retain = false;
msg.payload = JSON.stringify({
    request_id: requestId,
    command: "clear_file",
    confirm: true,
    backup: true,
    use_expected_revision: true,
    expected_revision: markers.revision
});
flow.set("arucoDatabasePendingRequest", requestId);
return msg;
```

The Function node is only a user-interface guard. The ROS core service still
validates the request and the expected revision. Match
`response.request_id` before showing success. If a clear response times out,
do not automatically resend it with a new request ID: the first operation may
have succeeded. Inspect the next retained snapshot/status and show an
`unknown outcome` state until the operator confirms the result.

## ROS interface reference

The bridge consumes:

| Name | Type | Use |
|---|---|---|
| `/aruco_database/markers` | `aruco_database/msg/ArucoMarkerArray` | Full marker snapshot |
| `/aruco_database/status` | `aruco_database/msg/ArucoDatabaseStatus` | Persistence/origin state |
| `/aruco_database/list_markers` | `aruco_database/srv/ListArucoMarkers` | On-demand refresh |
| `/aruco_database/get_position` | `aruco_database/srv/GetArucoPosition` | Existing single-ID lookup |
| `/aruco_database/clear` | `aruco_database/srv/ClearArucoDatabase` | Coordinated clear and persist |

The core's existing `get_position` service remains unchanged for existing
consumers such as a future Direct mode. The bridge does not modify the
`/target_pose` compatibility path used by `aruco_tracker` and `precision_land`.

Inspect the graph with:

```sh
ros2 topic list | grep aruco_database
ros2 topic type /aruco_database/markers
ros2 topic echo /aruco_database/status --once
ros2 service list | grep aruco_database
```

The ROS core service can be tested without MQTT:

```sh
ros2 service call /aruco_database/list_markers \
  aruco_database/srv/ListArucoMarkers "{}"

ros2 service call /aruco_database/clear \
  aruco_database/srv/ClearArucoDatabase \
  "{confirm: true, backup: true, use_expected_revision: false, expected_revision: 0}"
```

Use the clear command only when intentionally starting a new marker database.

## Broker ACLs

For the default prefix, the bridge account needs only:

| Client | Publish | Subscribe |
|---|---|---|
| `aruco_database_bridge` | `aruco_database/markers`, `status`, `event`, `response` | `aruco_database/command` |
| Node-RED | `aruco_database/command` | `aruco_database/markers`, `status`, `event`, `response` |

A temporary broader `aruco_database/#` ACL may help diagnose a broker setup,
but narrow the ACL before deployment. Do not give Node-RED access to arbitrary
filesystem paths or PX4 command topics.

## Troubleshooting

### The bridge starts but no MQTT messages appear

- Check that `mqtt_host` is a hostname only and is not empty.
- Check that the container has both environment credentials.
- Confirm the broker ACL permits the exact topic directions above.
- Confirm Node-RED subscribes before publishing commands.
- Check that the bridge client ID is unique.
- Leave `mqtt_tls_ca_file` empty for the normal system CA bundle.

### MQTT TLS or authentication fails

- Use the broker hostname as TLS SNI, not an IP address or URL.
- Verify the container clock and CA bundle.
- Confirm both username and password are present.
- The bridge intentionally has no insecure TLS fallback and does not support
  mTLS client certificates.

### `core_unavailable` remains in status

- Start `aruco_database` before or alongside the bridge.
- Check that the generated interface package is sourced:

  ```sh
  source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash
  ```

- Check that the bridge can see the ROS topics and services:

  ```sh
  ros2 topic echo /aruco_database/status --once
  ros2 service list | grep aruco_database
  ```

### The marker table does not update

- Subscribe to `aruco_database/markers` directly with a broker client.
- Check that `/aruco/detections` is publishing and that the core has a valid
  origin and TF chain.
- Inspect `aruco_database/status`; `waiting_for_origin`, `pending`, or
  `persistence_error` explain different failure modes.
- Make the Node-RED Function replace the table payload from each complete
  snapshot instead of appending rows.

### Clear reports a revision conflict

A marker arrived after Node-RED last received its snapshot. Subscribe to the
new marker snapshot, update the UI revision, confirm the new practice area,
and send a new clear request. This is intentional protection against clearing
new data from a stale page.

### Clear reports persistence failure

The core cleared its active map but could not atomically replace the YAML file.
The response has `persisted: false`; the old file should remain available and
the status reports `file_state: error`. Check directory permissions, disk
space, and the path passed to the core's `database_file` parameter. Do not have
Node-RED delete the file manually.

## Security and operational boundary

- MQTT commands are authenticated by the broker credentials and constrained by
  broker ACLs; ROS 2 itself does not provide authorization for arbitrary local
  service callers.
- `clear_file` requires explicit confirmation and can require an expected
  revision.
- The bridge never accepts a database path from MQTT.
- MQTT command callbacks only queue strings; ROS service calls run on the ROS
  executor thread rather than in the Mosquitto network callback.
- Commands are not retained and the queue is bounded.
- Payloads never contain MQTT passwords.
- Retained status and marker snapshots are for display. Node-RED should show a
  local offline/timeout state when live updates stop.

## Current limitations

- The marker record contains ID, averaged latitude/longitude, and observation
  count. It does not contain marker dictionary, confidence, covariance,
  altitude, or a per-observation history.
- The core uses a running average and does not reject spatial outliers.
- The bridge provides a full snapshot rather than one MQTT topic per marker.
- The clear operation starts a new active database session; a file-only delete
  without clearing the active map is intentionally not exposed.
- The bridge command request cache is in memory and is not a durable job log.
- `status_publish_period_ms` controls periodic retained status republishing;
  `core_status_timeout_ms` controls how long the last core heartbeat remains
  trusted. Marker updates are still forwarded immediately when the MQTT
  connection is available.
