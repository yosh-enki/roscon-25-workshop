# Module 05: Foxglove Mission Control & UI

The `full_self_driving` system features a modern, aerospace-grade ground control station layout built natively on **Foxglove Studio** and powered by a custom packaged panel extension (`roscon25.fsd-mission-control-1.0.0.foxe`).

---

## 1. Aerospace HUD 3-Column Layout Architecture

The mission control interface ([`foxglove/roscon-25-workshop.json`](file:///home/yosh/roscon-25-workshop/foxglove/roscon-25-workshop.json)) is divided into three functional zones designed for low-latency situational awareness:

```
┌────────────────────────┬──────────────────────────────────┬────────────────────────┐
│ 1. MISSION CONTROL HUD │ 2. SPATIAL & SENSOR VISUALIZER   │ 3. PLAN & FLIGHT LOG   │
│                        │                                  │                        │
│ • Flight Safety Banner │ • 3D Scene Visualizer            │ • Plan Artifact Upload │
│   (Mode, State, GPS)   │   - Drone URDF Model             │ • Plan Selection Chips │
│ • Vehicle Telemetry    │   - Coordinate Frames (TF2)      │   (AAVC2026 / SAR)     │
│   (Alt, Speed, Batt)   │   - Live Target Lock Marker      │ • Waypoint Progress    │
│ • Target Pad Chips     │   - Geodesic Pad Locations       │   [████████░░] 80%     │
│   [Pad 1] .. [Pad 6]   │                                  │ • Mission Journal      │
│ • Assign Target Button │ • Downward Camera Feed           │   (Persistent Events)  │
│ • Cargo Latch Controls │   - Annotated ArUco Bounding Box │ • Raw Topic Plotter    │
│   [OPEN] [CLOSE & LOCK]│   - 6-DoF Optical Center Frustum │   (Altitude vs Time)   │
└────────────────────────┴──────────────────────────────────┴────────────────────────┘
```

---

## 2. Custom Extension: FSD Mission Control (`.foxe`)

The repository bundles a pre-compiled, custom Foxglove extension located at [`foxglove/roscon25.fsd-mission-control-1.0.0.foxe`](file:///home/yosh/roscon-25-workshop/foxglove/roscon25.fsd-mission-control-1.0.0.foxe).

### 2.1 Left Panel: FSD Mission Control
- **Dynamic Telemetry Gauges**:
  - `Altitude (AGL)`: Derived from `/full_self_driving/telemetry` (Real-time bar and numeric display).
  - `Horizontal Speed`: Real-time velocity magnitude ($m/s$).
  - `Battery Remaining`: Color-coded battery percentage ($>30\%$ Green, $20-30\%$ Amber, $<20\%$ Red).
- **Target Selection Chips**:
  - Fast selection buttons: `Pad 1`, `Pad 2`, `Pad 3`, `Pad 4`, `Pad 5`, `Pad 6`.
  - Operator clicks chip $\rightarrow$ Clicks **`ASSIGN PAD {id}`** $\rightarrow$ Triggers `/full_self_driving/target/select` service with `marker_id`, dictionary `DICT_4X4_50`, and namespace `aavc2026`.
- **Payload Latch Actions**:
  - **`CLOSE & LOCK`**: Calls `/full_self_driving/payload/prepare` with `state=1` (arms latch before takeoff).
  - **`OPEN`**: Calls `/full_self_driving/payload/prepare` with `state=0` (releases cargo manually on ground).

### 2.2 Center Panel: 3D Scene & Computer Vision
- **3D World Display**:
  - Displays `map`, `odom`, `base_link`, and `camera_frame` using live `/tf` and `/tf_static`.
  - Renders the quadrotor URDF mesh model ([`simulation/urdf/x500.urdf`](file:///home/yosh/roscon-25-workshop/full_self_driving/simulation/urdf/x500.urdf)).
  - Renders ArUco target markers dynamically when published on `/full_self_driving/perception/all_id_observations`.
- **Image Visualizer**:
  - Streams `/full_self_driving/perception/annotated_image` at 30 Hz with color-coded 2D bounding boxes, ID labels, and 3D coordinate axes rendered on the marker centroid.

### 2.3 Right Panel: FSD Plan Manager
- **Plan Upload & Selection**:
  - Operator can upload `.plan` QGroundControl mission files directly through Foxglove.
  - Quick-switch chips for pre-loaded mission plans (e.g. `aavc2026_mission.plan`).
- **Real-Time Waypoint Progress**:
  - Subscribes to `/full_self_driving/flight/working_plan_status` ([`WorkingPlanStatus.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/WorkingPlanStatus.msg)).
  - Displays current waypoint index, total waypoints, and percentage progress bar.

---

## 3. Foxglove Bridge & Port Configuration

The companion stack includes `foxglove_bridge` exposing a high-performance WebSocket server on port `8765` (configurable).

### 3.1 Custom Port Launching
If port `8765` is in use by another application:
```bash
ros2 launch full_self_driving full_self_driving.launch.py foxglove_port:=8766
```

### 3.2 Connecting from Foxglove Studio
1. Open Foxglove Studio (Desktop or Web: [https://studio.foxglove.dev](https://studio.foxglove.dev)).
2. Click **Open connection** $\rightarrow$ Select **Foxglove WebSocket**.
3. URL: `ws://localhost:8765` (or host IP if connecting from remote machine).
4. Load Layout: Menu $\rightarrow$ **Layout** $\rightarrow$ **Import from file** $\rightarrow$ Select [`foxglove/roscon-25-workshop.json`](file:///home/yosh/roscon-25-workshop/foxglove/roscon-25-workshop.json).
5. Install Extension (if prompted): Drag and drop [`foxglove/roscon25.fsd-mission-control-1.0.0.foxe`](file:///home/yosh/roscon-25-workshop/foxglove/roscon25.fsd-mission-control-1.0.0.foxe) into the Foxglove window.

---

## 4. Automated Layout Validation

The repository provides an automated layout validator script to verify layout schema and panel bindings in CI:

```bash
python3 scripts/validate_foxglove_layout.py
```
- Validates panel ID structure, custom extension IDs, and subscribed ROS 2 topic names.
