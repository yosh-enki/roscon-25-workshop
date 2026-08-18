# Implementation Plan: Full Self-Driving

## Overview

Implement the production `full_self_driving` ROS 2 package as a sequence of runnable vertical slices. The implementation language is **C++17/20 for the ROS nodes, domain core, adapters, and tests, with Python only for `full_self_driving.launch.py`**. This is selected from the existing workspace baseline: the prototype behavior is C++ ROS 2 and the current build uses `ament_cmake`, C++17/20, `px4_msgs`, and `px4_ros2_cpp`.

The existing prototype is a read-only behavioral baseline. Port its proven algorithms and sequencing into new production files, using new production names, namespaces, bounded interfaces, configuration objects, lifecycle ownership, and persistence boundaries. Do not modify, import, link, install, launch, or depend on `aruco_tracker`, `aruco_database`, `transit_in`, `transit_out`, `search`, `precision_land`, `px4_roscon_25`, `gazebo_models`, their messages/services/topics/parameters, or their launch/scripts at runtime. The KMITL world/materials may be copied once into production-owned simulation assets with an explicit manifest and provenance/hash; the old directory remains a read-only reference and is not a runtime path.

The production flight architecture is fixed throughout the plan: exactly one registered `FullSelfDrivingMode`, exactly one owning `FullSelfDrivingModeExecutor` derived from the actual pinned `ModeExecutorBase` API, and internal `TransitIn`, `TransitOut`, `Search`, `Direct`, and `PrecisionLand` strategies inside that mode. No strategy registers a mode, creates an executor, schedules PX4 modes, or publishes raw flight topics. All companion flight updates use the verified, version-matched `px4_ros2_cpp` abstractions while the registered mode is active; there is no Offboard fallback, direct `/fmu/in/offboard_control_mode`, direct `/fmu/in/trajectory_setpoint`, raw PX4 bridge, generic setpoint service, or alternate flight-control library.

Every leaf task is required to leave the package runnable through the one public launch entry point. The `Launch update` line in each task identifies the exact point at which `launch/full_self_driving.launch.py` is changed. Long-lived launch commands below are to be run manually in a terminal, never in a watch mode or through an agent background process.

## Execution conventions

Run build and test commands inside the supplied workshop container after sourcing the container's ROS/PX4 overlays. Set `WORKSPACE` to the repository root and provide `ROS_SETUP`/`PX4_ROS_SETUP` for overlay locations that vary by container:

```bash
export WORKSPACE=/home/yosh/roscon-25-workshop
source "${ROS_SETUP:?Set ROS_SETUP to the ROS 2 setup script for this container}"
source "${PX4_ROS_SETUP:?Set PX4_ROS_SETUP to the external PX4/ROS overlay setup script for this container}"
source "$WORKSPACE/install/setup.bash"
cd "$WORKSPACE"
```

For each slice, use a writable test state directory outside installed package share and a valid engineer-owned simulation configuration. The plan may add a test-only `engineering_config_simulation.yaml` and test fixtures, but the runtime must still load one authoritative resolved configuration and hash it. A path passed as `engineering_config:=...` is only the configuration selector; it is never a policy override.

The common build/test commands are:

```bash
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source "$WORKSPACE/install/setup.bash"
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --verbose
```

The production simulation command, once the approved fixture is installed, is exactly:

```bash
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true world:=kmitl_airfield headless:=false
```

During development, the same public launch file may receive the explicit configuration selector and `headless:=true` for deterministic CI/smoke use:

```bash
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true world:=kmitl_airfield headless:=true \
  engineering_config:="$FSD_TEST_CONFIG"
```

Every task's regression check includes the previously completed slices, `colcon test`, and a source/dependency boundary check appropriate to the new code. A task may add a test-only replay/fault-injection executable to the launch under an explicit test argument, but no test fixture may become a production flight dependency.

## Prototype behavior-port and change-control rules

1. At the package foundation, create `test/fixtures/prototype_behavior_map.yaml` and immutable replay/golden-data directories. Each entry maps a read-only prototype source symbol to the new production component, records the preserved algorithm/flow, identifies the test fixture, and lists any intentional production change.
2. Port the proven ArUco detector operations (`detectMarkers`, undistortion, `solvePnP`, Rodrigues/quaternion conversion, all-marker drawing, configured-target annotation), shared PX4 odometry/home/land freshness and waypoint-settle handling, Search plan parsing/working-copy/checkpoint behavior, and PrecisionLand spiral/transform/controller/state behavior before adding production-only guards. Do not replace a proven algorithm with an unvalidated alternative.
3. The current repository has no `Direct` implementation. Task 10 records `DIRECT_PROTOTYPE_BASELINE=ABSENT` in the test mapping and implements only the design-approved map-assisted navigation and fallback guards. If a future read-only reference is discovered, add a replay fixture before changing the production strategy.
4. Every intentional behavior difference must carry a stable `safety_change_id` in the test fixture, cite the applicable requirement/design property, explain why the production boundary requires it (genericity, bounded API, lifecycle, freshness, authority, or safety), and have a regression test. No silent divergence is accepted.
5. Production source scans must fail if production code includes prototype package headers, links prototype targets, uses prototype message/service types, advertises prototype topics/parameters, launches prototype files, calls `gazebo_models/run_world.sh`, or contains an Offboard/raw-control path.

## Tasks

- [x] 1. Establish the standalone package and integrated simulation foundation
  - [x] 1.1 Create the minimal standalone production package and first public launch contract
    - **Prerequisites:** None. Use C++17/20 for production code and Python for launch; do not add a dependency on any prototype package.
    - **Production files/components:** Create `full_self_driving/package.xml`, `CMakeLists.txt`, `resource/full_self_driving`, `launch/full_self_driving.launch.py`, `config/schemas/`, `src/runtime/launch_probe.cpp`, `test/fixtures/prototype_behavior_map.yaml`, and the initial `test/launch/` boundary fixture. Add only production-owned package dependencies and the pinned interface-library placeholders needed for later gates.
    - **Implementation:** Make the launch file the only installed public entry point. Add a bounded launch probe that reports package/config/profile state and exits readiness when the simulation manifest is not yet complete; do not register a PX4 mode. Make the launch arguments `simulation`, `world`, `headless`, and the authoritative configuration selector explicit and reject unknown/unsafe values. Add CMake test registration and install rules without importing old package names.
    - **Launch update:** Introduce `launch/full_self_driving.launch.py`; the foundation launch starts only the production launch probe and reports `SIMULATION_PROFILE_NOT_READY` until Task 1.2 completes.
    - **Run:** Build with the common `colcon build` command, then run `ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:=kmitl_airfield headless:=true engineering_config:="$FSD_TEST_CONFIG"`. In a second terminal inspect `ros2 node list` and the launch probe output; the expected state is explicit not-ready, not a silent success.
    - **Pass criteria:** The new package builds and installs; the one launch entry point starts the production probe; the selector is treated as a path selector only; no mode, PX4 command, camera subscription, or prototype process is started; invalid profile/path input fails with a bounded error before readiness.
    - **Regression checks:** Run `colcon test`; scan `package.xml`, `CMakeLists.txt`, and the launch file for prototype package names, `gazebo_models`, Offboard symbols, raw `/fmu/in` control topics, and arbitrary shell/path expansion.
    - **Requirements:** 1.1, 1.3, 1.4, 1.10, 4.2, 5.6, 7.7, 7.9.

  - [x] 1.2 Wire the manifest-driven Gazebo, PX4/SITL, MicroXRCE-DDS, clock, camera, and TF foundation
    - **Prerequisites:** 1.1. The exact PX4/ROS versions remain an implementation gate, but the launch must use manifest-selected executables and paths rather than prototype launch files.
    - **Production files/components:** Add `simulation/manifests/profile_simulation.yaml`, `simulation/manifests/kmitl_airfield.yaml`, production-owned `simulation/worlds/kmitl_airfield.sdf` and referenced materials/textures, `simulation/bridges/clock.yaml`, `simulation/bridges/camera.yaml`, `simulation/bridges/tf.yaml`, and the manifest/process resolver used by `full_self_driving.launch.py`.
    - **Implementation:** Copy the KMITL asset content into the production-owned asset tree only after recording source path/version/hash in the manifest; preserve relative resource resolution. Validate executable identity, working directory, ROMFS/autostart, resource roots, bounded arguments, required `/clock`, camera/camera-info, TF, and MicroXRCE-DDS readiness. Start Gazebo, configured PX4 SITL, the required agent, bridges, and production probe in dependency order; supervise exits and reverse shutdown. Do not invoke `gazebo_models/run_world.sh`, `px4_roscon_25/common.launch.py`, or any prototype launch file.
    - **Launch update:** Replace the Task 1.1 not-ready branch with the complete simulation process graph. `world:=` selects an allowlisted catalog ID and `headless:=` changes presentation only; neither changes flight policy.
    - **Run:** Run the common build, then manually run `ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:=kmitl_airfield headless:=true engineering_config:="$FSD_TEST_CONFIG"`. Verify `gz`/PX4/agent output and, from another terminal, `ros2 topic echo /clock --once`, `ros2 topic echo /fmu/out/vehicle_status_v1 --once`, `ros2 topic list | grep -E '/camera|/camera_info|/tf'`, and `ros2 node list`.
    - **Pass criteria:** The selected world/resources, PX4 SITL fixture, MicroXRCE-DDS agent, `/clock`, camera/camera-info, TF, and production probe are live; readiness reports every dependency; a required child failure withdraws readiness and triggers reverse-order shutdown; the launch never starts a fake hardware branch or old dependency.
    - **Regression checks:** Repeat the Task 1.1 invalid manifest/path tests and `colcon test`; compare the process graph against the manifest and assert no prototype, Offboard, raw-control, or arbitrary plugin path occurs.
    - **Requirements:** 1.1, 1.3, 1.4, 1.5, 1.6, 1.8, 7.7.

  - [x] 1.3 Add the first launch and prototype-boundary regression harness
    - **Prerequisites:** 1.2. The harness must inspect source, installed launch metadata, dependency manifests, and the live process graph without changing prototype files.
    - **Production files/components:** Add `test/launch/launch_manifest_test.cpp`, `test/security/forbidden_dependency_scan.py`, and CTest/launch-test registration. Add a test-only process failure fixture that terminates a required child.
    - **Implementation:** Assert one public launch file, production-owned asset resolution, reverse dependency shutdown, and explicit hardware deferral. Scan CMake/package/launch/test fixtures/generated interfaces for prototype packages, old public contracts, `OffboardControlMode`, direct `/fmu/in/offboard_control_mode`, direct `/fmu/in/trajectory_setpoint`, raw PX4 publishers, generic setpoint services, and alternate control libraries.
    - **Launch update:** Add the test fixture and failure hooks to the existing launch under an explicit test argument; the default command remains unchanged.
    - **Run:** Run `colcon test --packages-select full_self_driving --event-handlers console_direct+`; run the launch test with `FSD_LAUNCH_TEST=1 ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:=kmitl_airfield headless:=true engineering_config:="$FSD_TEST_CONFIG"` and inject a declared child exit.
    - **Pass criteria:** Tests fail closed on a forbidden dependency, an incomplete manifest, or an unexpected process; required-child failure withdraws readiness and shuts down in reverse order; no prototype file is modified.
    - **Regression checks:** Re-run the live foundation inspection from 1.2 and all earlier CTest cases.
    - **Requirements:** 1.3, 1.4, 1.5, 1.6, 5.6, 7.7.

- [x] 2. Deliver the first meaningful behavior slice: production ArUco perception
  - [x] 2.1 Port the prototype ArUco detector into a production lifecycle perception component
    - **Prerequisites:** 1.2. Read-only behavior baseline: `px4_roscon_25/aruco_tracker/ArucoTracker.cpp/.hpp`; production code must not include or link it.
    - **Production files/components:** Add `msg/MessageHeader.msg`, `msg/TargetIdentity.msg`, `msg/AllIdObservation.msg`, `msg/AllIdObservationBatch.msg`, `src/perception/aruco_detector.hpp/.cpp`, `src/perception/perception_node.cpp`, and the required `fsd_perception` executable/registration.
    - **Implementation:** Port the proven OpenCV sequence: bounded image conversion, dictionary detector, all-marker detection/drawing, camera-info calibration update, corner undistortion, marker-size object points, `solvePnP`, Rodrigues/quaternion conversion, finite-value checks, and annotated image output. Refactor it into an `rclcpp_lifecycle::LifecycleNode` with production configuration/catalog IDs and bounded QoS/queues. Publish all accepted marker observations using the new production messages; do not publish `aruco_database` messages or `/target_pose`.
    - **Launch update:** Add `fsd_perception` to the launch after camera/TF readiness, initially publishing all-ID observations and an inactive/active health signal; no target lock or flight transition is allowed.
    - **Run:** Build and launch the simulation manually. Inspect `ros2 topic echo /full_self_driving/perception/all_id_observations --once`, `ros2 topic echo /full_self_driving/health`, and the annotated image using the configured visualization client or `ros2 topic hz`/image inspection. Confirm camera-info is accepted before poses are emitted.
    - **Pass criteria:** A live camera frame produces bounded all-ID observations with identity, frame, covariance, calibration hash, quality, timestamp, and scope; the annotated image shows every detected marker; calibration loss is visible and fails closed; the node configures/activates/deactivates without recreating flight objects.
    - **Regression checks:** Run the foundation process graph and boundary scans; confirm no `/target_pose`, `aruco_database`, prototype parameter, or prototype topic is advertised.
    - **Requirements:** 1.1, 1.8, 3.2, 3.4, 7.1, 7.4.

  - [x] 2.2 Connect production camera/TF inputs and perception status/visualization through the public launch
    - **Prerequisites:** 2.1. Use only manifest-resolved camera topics and transforms; do not copy the prototype launch file.
    - **Production files/components:** Complete `simulation/bridges/camera.yaml` and `simulation/bridges/tf.yaml`, add `src/perception/perception_status.cpp` if needed, and add production-owned launch actions for image bridge, camera-info bridge, static/dynamic TF, and visualization output.
    - **Implementation:** Resolve the camera topic/frame from the simulation manifest, publish complete production health/status, bound image queue behavior, and make `use_sim_time` explicit for every simulation node. Preserve the prototype camera-to-body convention only as a manifest calibration transform; do not embed the KMITL model name or camera frame in flight code.
    - **Launch update:** The public launch now starts the production camera/image/camera-info/TF bridge and `fsd_perception` as one connected camera → detector → all-ID observation/visualization slice.
    - **Run:** Run `ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:=kmitl_airfield headless:=true engineering_config:="$FSD_TEST_CONFIG"`; verify `ros2 topic echo /full_self_driving/perception/all_id_observations`, `ros2 topic echo /full_self_driving/perception/annotated_image --once`, TF lookup for the configured camera frame, and the complete perception `ComponentHealth`.
    - **Pass criteria:** The same public launch supplies camera, `/clock`, TF, and PX4 dependencies; all-ID observations and visualization are live; frame/calibration errors are explicit; no hardcoded site/model/route value is added to detector code.
    - **Regression checks:** Repeat 1.2 process/readiness checks, CTest, and forbidden dependency scan.
    - **Requirements:** 1.1, 1.5, 1.8, 3.2, 3.4, 7.1, 7.4.

  - [x] 2.3 Add ArUco replay/parity fixtures against the proven prototype behavior
    - **Prerequisites:** 2.1 and 2.2. Capture expected outputs from the read-only prototype offline or use checked-in synthetic images/calibration; the production test must not build, import, or execute the prototype.
    - **Production files/components:** Add `test/fixtures/prototype_behavior/aruco/` image/camera-info/golden observations, the ArUco entry in `test/fixtures/prototype_behavior_map.yaml`, and `test/perception/aruco_replay_test.cpp`.
    - **Implementation:** Compare detection IDs, pose frame, pose/quaternion normalization, annotation presence, and all-ID cardinality within declared tolerances. Record any production-only fields (scope, dictionary/namespace, covariance, calibration hash, bounded quality) as approved safety/API changes with `safety_change_id` values.
    - **Launch update:** Add an explicit `replay_fixture:=aruco` test mode that publishes the fixture camera stream into the same production perception node; default simulation launch behavior is unchanged.
    - **Run:** Run `ctest --test-dir build/full_self_driving -R aruco_replay --output-on-failure`, then run the public launch with `replay_fixture:=aruco` and inspect the production all-ID topic and annotated image.
    - **Pass criteria:** The production detector matches the proven detection/pose behavior within fixture tolerances, preserves every detected ID, and rejects invalid calibration/frames; no prototype target-only shortcut is reintroduced.
    - **Regression checks:** Run 2.2 live-camera smoke, all foundation tests, and the dependency/offboard scan.
    - **Requirements:** 3.2, 3.4, 7.1; design Property 7 as a separation prerequisite.

- [x] 3. Add selected-target live-lock qualification and map/scenario-scoped registry
  - [x] 3.1 Implement production target identity, live-lock qualification, and a test-only selection provider
    - **Prerequisites:** 2.2. The standard runtime will receive the selected identity from `MissionContext` later; this task may add only a test-only typed selection provider for exercising the slice before Task 4.3.
    - **Production files/components:** Add `src/domain/target_identity.cpp`, `src/domain/live_target_lock.cpp`, `src/perception/target_coordinator.cpp`, `msg/LiveTargetLock.msg`, and `test/fixtures/target_selection_provider.cpp`.
    - **Implementation:** Consume all-ID observations and qualify only the selected marker ID, dictionary, and namespace after scope, frame/TF, calibration, freshness, quality, covariance, consecutive-observation, and spatial-consistency gates. Publish candidate/qualified/stale/lost lock data; never call a mode switch, arm, takeoff, or executor API from perception.
    - **Launch update:** Add the coordinator to `fsd_perception`; add `test_selection:=...` only as an explicit test argument. The default launch has no hardcoded selected target and therefore cannot create a lock until a context is committed.
    - **Run:** Launch with a fixture selection and `replay_fixture:=aruco`; publish mixed marker IDs/dictionaries/namespaces through the test input and inspect `/full_self_driving/perception/live_target_lock` and health. Then repeat without a selected identity and verify no qualified lock.
    - **Pass criteria:** Only an identity-matching, fresh, qualified stream reaches `QUALIFIED`; all-ID observations remain available for registry use; stale/lost transitions are emitted; perception owns data only and cannot change flight authority.
    - **Regression checks:** Repeat the live camera and ArUco parity launch, and verify no legacy `/target_pose` or prototype contract is present.
    - **Requirements:** 3.2, 3.7, 3.8, 4.5, 5.2, 7.1.

  - [x] 3.2 Implement the map/scenario-scoped pad registry lifecycle node
    - **Prerequisites:** 3.1. Registry records must use production `TargetIdentity` and all-ID observations, not the old `aruco_database` messages/services.
    - **Production files/components:** Add `msg/PadRecord.msg`, `msg/PadRegistrySnapshot.msg`, `msg/PadRegistryStatus.msg`, `src/registry/pad_registry.hpp/.cpp`, `src/registry/pad_registry_node.cpp`, and the production registry service endpoint stub needed for later clear/backup wiring.
    - **Implementation:** Key records by map, scenario, namespace, dictionary, and marker ID. Apply timestamp, transform, calibration, quality, covariance, outlier, and scope checks; publish complete active-scope snapshots/status with revision and durability placeholders. Keep registry existence separate from `LiveTargetLock`; expose a deterministic test injection path only under a test launch argument.
    - **Launch update:** Add `fsd_pad_registry` as a lifecycle node after perception and wire all-ID observations to registry snapshots/status.
    - **Run:** Launch with the ArUco replay/selection fixture, inject observations for two map/scenario scopes, and inspect `/full_self_driving/pad_registry`, `/full_self_driving/pad_registry/status`, and the live-lock topic. Verify a record in the inactive scope cannot appear in the active snapshot.
    - **Pass criteria:** Accepted observations update only their validated scope; cross-scope/identity records are excluded; registry records never produce a live lock or landing permission; status contains revision, quality/age, and component health.
    - **Regression checks:** Repeat all camera, TF, all-ID, and live-lock checks; run CTest and scan for old database contracts.
    - **Requirements:** 3.1, 3.4, 3.5, 3.8, 7.1, 7.4.

  - [x] 3.3 Add the map/scenario registry-isolation property test
    - **Prerequisites:** 3.2. Use a C++ property generator for identities, scopes, record ages, quality, covariance, and revisions.
    - **Production files/components:** Add `test/property/property_6_registry_isolation.cpp` and register a named CTest `fsd_property_6_registry_isolation`.
    - **Implementation:** Exercise lookup, observation acceptance, clear preconditions, backup state, and scope changes; assert that no record crosses map/scenario/namespace/dictionary/ID boundaries and that clear remains revisioned/disarmed-only.
    - **Launch update:** Add `property_fixture:=registry_isolation` to the test-only launch path so generated records can be observed through the same registry status topic.
    - **Run:** Run `ctest --test-dir build/full_self_driving -R fsd_property_6_registry_isolation --output-on-failure`, then run the public launch with the fixture and inspect the registry revision/status.
    - **Pass criteria:** All generated cases satisfy design Property 6; rejected scope/revision/armed mutations have no state change or flight side effect.
    - **Regression checks:** Run 3.2 scoped registry smoke and all earlier perception tests.
    - **Requirements:** 3.1, 3.5, 3.6; **Property 6: Map/scenario registry isolation**.

  - [x] 3.4 Add the all-ID/live-lock separation property test
    - **Prerequisites:** 3.1 and 3.2.
    - **Production files/components:** Add `test/property/property_7_all_id_live_lock.cpp` and register `fsd_property_7_all_id_live_lock`.
    - **Implementation:** Generate accepted/rejected all-ID observations and selected identities; assert registry updates may occur for valid all-ID data, but a qualified lock requires exact identity, scope, freshness, quality, covariance, transform, consecutive, and spatial gates.
    - **Launch update:** Add the property replay fixture to the test-only launch without changing the default process graph.
    - **Run:** Run the named CTest and the public launch with mixed-ID replay; inspect all-ID count and lock state.
    - **Pass criteria:** No registry record or nonmatching observation can create `QUALIFIED`; all valid selected-target locks contain the committed identity and evidence timestamps.
    - **Regression checks:** Repeat the ArUco parity and registry isolation tests.
    - **Requirements:** 3.2, 3.7, 3.8; **Property 7: All-ID/live-lock separation**.

- [x] 4. Freeze bounded production contracts and make configuration/context authoritative
  - [x] 4.1 Generate the complete bounded ROS 2 interface boundary and contract probe
  - [x] 4.2 Implement the authoritative engineering configuration loader, resolver, validator, and canonical hash
  - [x] 4.3 Implement MissionContextStore, OperatorSelection, commit/lock snapshots, and authoritative readiness
  - [x] 4.4 Add the authoritative engineering-configuration property test
  - [x] 4.5 Add the configuration-hash consistency property test
  - [x] 4.6 Add the disarmed operator-selection isolation property test

  - [x] 4.7 Add the complete Ownmode-readiness property test
  - [x] 4.8 Add the concrete bounded ROS interface property test

- [x] 5. Make managed plan artifacts and working-plan progress runnable
  - [x] 5.1 Port the proven QGroundControl plan parser/printer into PlanManager with immutable managed artifacts
    - **Prerequisites:** 4.1, 4.2, and 4.3. Read-only behavior baseline: `search/src/PlanParser.cpp/.hpp` and `search/src/SearchPlanner.cpp/.hpp`.
    - **Production files/components:** Add `src/domain/plan_parser.cpp`, `src/domain/plan_printer.cpp`, `src/runtime/plan_manager.cpp`, `msg/PlanArtifactReference`, the typed `UploadPlanArtifact` and `SelectPlanArtifact` handlers, and the read-only managed artifact projection used by `list_plan_artifacts`.
    - **Implementation:** Port bounded JSON parsing, nested mission-item walking, command-16 waypoint extraction/source indexes, `CameraCalc.DistanceToSurface` handling, finite coordinate validation, canonical route hashing, and canonical printing. Ingest bytes rather than paths, enforce safe basename/size/depth/item limits, atomically store immutable managed artifacts, reject hash-changing replacement, and never expose the source path.
    - **Launch update:** Add PlanManager to the runtime launch and wire upload/list/select into MissionContext; the default launch uses only a test fixture artifact when explicitly requested.
    - **Run:** Through the typed test driver, upload `aavc2026_mission.plan` fixture bytes, list/select the managed ID, inspect `MissionContext`, and attempt traversal, oversized, malformed, unsupported safety-item, duplicate/hash-changing uploads. Run the public launch and `colcon test`.
    - **Pass criteria:** The accepted artifact is immutable, managed by ID/hash, parsed into a canonical route, and selectable only through a revision-guarded typed operation; invalid/path-based inputs have no file/state side effect.
    - **Regression checks:** Repeat config/context/live-lock/registry launch; scan for old `search` package or arbitrary path use.
    - **Requirements:** 2.2, 2.4, 2.5, 2.6, 2.8, 2.9, 2.11, 7.9.

  - [x] 5.2 Add WorkingPlan generation, reset, checkpoint, resume, and status wiring
    - **Prerequisites:** 5.1. Preserve the prototype's timestamped-copy/active-marker semantics as behavior, while replacing filenames with managed IDs and adding generation/revision/durability fields.
    - **Production files/components:** Add `src/domain/working_plan.cpp`, `src/runtime/working_plan_store.cpp`, `msg/SearchCheckpoint`, `msg/WorkingPlanStatus`, and handlers for `CreateOrSelectWorkingPlan`, `ResetWorkingPlan`, and checkpoint updates.
    - **Implementation:** Keep manual artifacts immutable; create a separate generated working record with source hash, map/scenario, generation, canonical route, checkpoint, reason, and progress. Reset is disarmed/revision/confirmation guarded, increments generation, sets empty checkpoint and `0%`; normal resume starts at checkpoint position or next source index. Use the durable store hook even before full journal recovery is wired.
    - **Launch update:** Add working-plan status and reset/create services to the public launch; the plan slice is now observable without flight-mode registration.
    - **Run:** Use the typed test driver to create, checkpoint, reset, and reload a working plan; inspect `/full_self_driving/working_plan/status` and `MissionContext`. Launch after a process restart and confirm the selected generation/checkpoint is reported.
    - **Pass criteria:** Reset never edits the manual source; generation increases and progress is exactly zero; checkpoint data is complete and a resume never silently restarts from source index zero.
    - **Regression checks:** Repeat artifact upload/hash/path tests, context revision/lock tests, and perception/registry launch.
    - **Requirements:** 2.3, 2.4, 2.5, 2.6, 2.7, 2.10, 6.7.

  - [x] 5.3 Add the plan immutability and safe-path property test
    - **Prerequisites:** 5.1.
    - **Production files/components:** Add `test/property/property_4_plan_immutability.cpp` and register `fsd_property_4_plan_immutability`.
    - **Implementation:** Generate safe/unsafe names, paths, bytes, duplicate IDs, and replacement attempts; assert managed identity/hash semantics and no arbitrary filesystem access or source replacement.
    - **Launch update:** Exercise the existing upload service through the public launch with a managed temporary directory.
    - **Run:** Run the named CTest and inspect the managed artifact list/status after each accepted/rejected upload.
    - **Pass criteria:** Design Property 4 holds; immutable artifacts remain byte/hash stable and rejected inputs create no partial file.
    - **Regression checks:** Run 5.1 and 5.2 plan/context smoke.
    - **Requirements:** 2.2, 2.11; **Property 4: Plan immutability and safe paths**.

  - [x] 5.4 Add the working-plan generation and resume property test
    - **Prerequisites:** 5.2.
    - **Production files/components:** Add `test/property/property_5_working_plan.cpp` and register `fsd_property_5_working_plan`.
    - **Implementation:** Generate valid working plans/checkpoints and resets; assert generation increases, progress/checkpoint clear on reset, source hash is preserved, and resume begins at the checkpoint or next source index.
    - **Launch update:** Use the existing working-plan service/read-model path with a test fixture.
    - **Run:** Run the named CTest, then restart the public launch with a checkpointed working plan and inspect the resumed status.
    - **Pass criteria:** Design Property 5 holds for all generated plans and no reset mutates the immutable source.
    - **Regression checks:** Repeat artifact immutability and context revision tests.
    - **Requirements:** 2.3, 2.7; **Property 5: Working-plan generation correctness**.

  - [x] 5.5 Add parser/printer round-trip and prototype SearchPlanner parity tests
    - **Prerequisites:** 5.1 and 5.2. Read-only baseline is the checked-in prototype `PlanParser`/`SearchPlanner`; production tests must use copied fixtures or independently generated inputs.
    - **Production files/components:** Add nested QGC plan fixtures, malformed/unsupported fixtures, `test/plan/plan_round_trip_test.cpp`, `test/plan/working_plan_parity_test.cpp`, and the Search entry in `prototype_behavior_map.yaml`.
    - **Implementation:** Compare extracted waypoint order/source indexes, altitude fallback, route hash, timestamped working generation, active selection, entry-point update, and resume route semantics. Record any bounded schema/generation differences as approved changes.
    - **Launch update:** Add `plan_fixture:=parity` to the test launch path and expose the working-plan status/checkpoint.
    - **Run:** Run `ctest --test-dir build/full_self_driving -R 'plan_(round_trip|parity)' --output-on-failure`; run the public launch with the parity fixture and inspect route/checkpoint status.
    - **Pass criteria:** Printed accepted plans parse back to equivalent supported navigation structures and the production working-plan behavior matches the proven baseline where not superseded by a cited safety/API constraint.
    - **Regression checks:** Run 5.1/5.2 service smoke and all previous launch tests.
    - **Requirements:** 2.3, 2.4, 2.5, 2.6, 2.7.

- [x] 6. Add durable state, lifecycle supervision, recovery gates, and the preparation gateway
  - [x] 6.1 Implement PersistenceManager durable boundaries, journals, backups, and restart loading
    - **Prerequisites:** 4.3 and 5.2. Storage paths must be outside installed package share and selected by the authoritative config.
    - **Production files/components:** Add `src/persistence/persistence_manager.hpp/.cpp`, snapshot/journal/backup schemas under `config/schemas/`, `msg/RecoveryStatus`, durable sequence integration for context/plan/registry, and storage fault-injection adapters under `test/fakes/`.
    - **Implementation:** Implement validate → sibling temporary write → flush/fsync/equivalent → atomic rename → directory durability → journal sequence → bounded backup. Persist config hash, selection/context, artifacts/working plans/checkpoints, registry, executor placeholder, payload placeholder, recovery markers, and evidence references. On restart validate hashes/sequences/scope/config compatibility and enter `RECOVERY_REQUIRED` for ambiguity; never auto-arm/resume/action/release.
    - **Launch update:** Start the persistence manager inside `fsd_flight_runtime`; publish `RecoveryStatus` and durability fields from the public launch. Add `storage_fault:=...` only as an explicit test argument.
    - **Run:** Commit a context/working plan through the typed test driver, stop/restart the public launch, inspect durable sequence and recovery status, and inject failures between each write/flush/rename/journal/backup boundary.
    - **Pass criteria:** State is called durable only after the configured boundary; failed writes preserve the last valid state; ambiguous restart blocks readiness and all automatic actions; registry clear/test backup has a durable backup before replacement.
    - **Regression checks:** Repeat config/context/plan/live-lock/registry smoke after restart and run all CTest cases.
    - **Requirements:** 1.9, 2.7, 3.6, 4.1, 6.1, 6.2, 6.3, 6.7, 6.8, 6.9.

  - [x] 6.2 Implement lifecycle ownership, activation order, and reverse process supervision
    - **Prerequisites:** 6.1. `fsd_flight_runtime` must remain a stable regular node; perception, registry, evidence, and gateway are lifecycle-managed.
    - **Production files/components:** Add lifecycle implementations for `fsd_perception`, `fsd_pad_registry`, `fsd_evidence`, and `fsd_gateway`; add `src/runtime/lifecycle_supervisor.cpp`, launch readiness/transition actions, and failure fixtures.
    - **Implementation:** Configure in the approved order, activate registry → perception → evidence → gateway, wait for config/storage/health, and only then report runtime readiness. Deactivate/cleanup in reverse order; preserve state; stop child processes reverse-dependency order. Do not construct/register a PX4 mode yet.
    - **Launch update:** Replace ad hoc node startup with lifecycle transitions and explicit readiness summary; runtime starts unregistered and remains so through every lifecycle failure.
    - **Run:** Run the public launch and inspect lifecycle states/health. Inject configure, activate, deactivate, cleanup, and child-process failures with `lifecycle_fault:=...`; verify reverse cleanup and preserved recovery state.
    - **Pass criteria:** Required lifecycle nodes are active before runtime readiness; gateway activates last; any failure withdraws readiness, preserves durable state, and shuts down safely; no simulated dependency replaces a missing hardware dependency.
    - **Regression checks:** Repeat 6.1 restart/fault tests and all earlier perception/registry/context/plan launch checks.
    - **Requirements:** 5.2, 6.4, 6.5, 6.6, 7.1.

  - [x] 6.3 Implement the typed Node-RED/MQTT preparation and inspection gateway
    - **Prerequisites:** 6.1, 6.2, and 4.1. Use a local/fake TLS broker for automated tests; never place credentials in the repository or command lines.
    - **Production files/components:** Add `src/gateway/fsd_gateway.cpp`, fixed command envelope/schema validation, MQTT/TLS/ACL configuration adapters, typed ROS clients for every allowed preparation/inspection service, and gateway status/read-model translation.
    - **Implementation:** Allow only the design command set; enforce request size/age/rate, non-retained commands, request ID idempotency, expected revisions, disarmed/locked/recovery gates, and complete authoritative response reconciliation. Explicitly reject arm/disarm/Ownmode/takeoff/land/RTL/goto/setpoint/raw-control/release/arbitrary ROS/filesystem commands. Gateway must never publish PX4 or payload control commands.
    - **Launch update:** The gateway becomes the last activated lifecycle node and publishes/consumes the production read model; a test broker endpoint is selected only by the security test manifest.
    - **Run:** Launch with the local TLS broker fixture, publish allowed `select_map_scenario`, `select_target_identity`, plan, validation, commit, inspection, and recovery requests, then publish retained/stale/replayed/forbidden requests. Inspect typed responses and context revisions.
    - **Pass criteria:** Allowed disarmed preparation reaches the authoritative store; retained/stale/replayed/forbidden/security-invalid requests are rejected; gateway disconnect leaves the locked flight context usable but cannot authorize a mutation; no PX4/payload/flight side effect is observed.
    - **Regression checks:** Repeat lifecycle/readiness and persistence/restart tests; verify status topics are informational rather than authorization.
    - **Requirements:** 2.1, 2.8, 2.9, 2.10, 4.2, 4.3, 4.4, 6.8, 6.9, 7.6, 7.9.

  - [x] 6.4 Add the gateway-command-boundary property test
    - **Prerequisites:** 6.3.
    - **Production files/components:** Add `test/property/property_10_gateway_boundary.cpp` and register `fsd_property_10_gateway_boundary`.
    - **Implementation:** Generate every disallowed command plus malformed, retained, stale, replayed, oversized, unauthorized, armed, locked, and stale-revision envelopes; assert no PX4, payload, filesystem, or state mutation side effect.
    - **Launch update:** Use the existing local broker/test gateway path; do not add a flight-control bridge.
    - **Run:** Run the named CTest and publish the negative envelope set while the public launch is active; inspect response/audit and PX4/payload side-effect counters.
    - **Pass criteria:** Design Property 10 holds for every forbidden command and invalid envelope.
    - **Regression checks:** Repeat 6.3 allowed-command smoke and 6.1 durability checks.
    - **Requirements:** 2.9, 4.2, 4.3; **Property 10: Gateway command boundary**.

  - [x] 6.5 Add the durable-boundary integrity property test
    - **Prerequisites:** 6.1.
    - **Production files/components:** Add `test/property/property_16_durable_boundary.cpp` and register `fsd_property_16_durable_boundary`.
    - **Implementation:** Generate snapshots/journal entries and inject failure at validation, temporary write, flush/fsync, rename, directory sync, journal, and backup stages; assert only fully completed records are marked durable and last-valid state is preserved.
    - **Launch update:** Use `storage_fault:=...` in the public launch and publish durability/readiness status.
    - **Run:** Run the named CTest and the launch fault sequence; inspect durable sequence, status, and on-disk managed IDs.
    - **Pass criteria:** Design Property 16 holds for every fault point; no partially written state is advertised as committed.
    - **Regression checks:** Repeat 6.1 restart recovery and 5.2 checkpoint smoke.
    - **Requirements:** 6.1, 6.3; **Property 16: Durable boundary integrity**.

  - [x] 6.6 Add the recovery-safety property test
    - **Prerequisites:** 6.1.
    - **Production files/components:** Add `test/property/property_17_recovery_safety.cpp` and register `fsd_property_17_recovery_safety`.
    - **Implementation:** Generate ambiguous snapshot, journal, working-plan, registry, config-hash, executor-placeholder, evidence, and payload states; assert `RECOVERY_REQUIRED`, no auto-arm/resume/strategy switch/payload operation, explicit ambiguity codes, and disarmed resolution requirement.
    - **Launch update:** Use the existing restart/recovery fixture in the public launch.
    - **Run:** Run the named CTest, restart the public launch from each generated durable state, and inspect `RecoveryStatus`/readiness.
    - **Pass criteria:** Design Property 17 holds for every ambiguous restart.
    - **Regression checks:** Repeat valid restart and lifecycle order from 6.1/6.2.
    - **Requirements:** 6.2, 6.8, 6.9; **Property 17: Recovery safety**.

  - [x] 6.7 Add the snapshot commit and recovery ordering property test
    - **Prerequisites:** 6.1 and 6.2.
    - **Production files/components:** Add `test/property/property_18_snapshot_commit.cpp` and register `fsd_property_18_snapshot_commit`.
    - **Implementation:** Trace commit publication ordering and restart reconciliation across snapshot, journal, registry, working plan, payload, executor checkpoint, and evidence records; assert publication happens only after the configured durable boundary and ambiguity enters recovery.
    - **Launch update:** Add a trace collector to the existing persistence/lifecycle test path.
    - **Run:** Run the named CTest and inspect the ordered event/durable sequence trace from a public launch commit/restart.
    - **Pass criteria:** Design Property 18 holds; no status can claim committed/durable before the required ordering completes.
    - **Regression checks:** Repeat gateway/context/plan commit and restart smoke.
    - **Requirements:** 6.1, 6.2, 6.3; **Property 18: Snapshot commit and recovery ordering**.

- [x] 7. Verify the actual PX4 API and establish the single registered-mode authority path
  - [x] 7.1 Create the pinned/version-matched `px4_ros2_cpp`/`px4_msgs` API verification gate
    - **Prerequisites:** 6.2. Use the versions actually installed by the workshop overlay (documented baseline is ROS 2 Humble/PX4 1.16-era tooling) but record exact package/repository commit IDs from the build environment.
    - **Production files/components:** Add `config/pinned_api_manifest.yaml`, `test/px4_api_probe/`, `src/adapters/px4_api_capabilities.cpp`, and CMake configure/compile checks. Do not write guessed `ModeBase`/`ModeExecutorBase` signatures.
    - **Implementation:** Inspect and compile against the actual headers for `ModeBase`, `ModeExecutorBase`, registration, mode requirements, arming checks, watchdog, activation/deactivation, library-managed setpoint types, and documented takeoff/land/RTL/result APIs. Record constructor/hook/result names and verify message compatibility. Fail configuration if the installed API does not match the manifest; do not substitute `MissionExecutor`, `ActionInterface`, Offboard, or another library.
    - **Launch update:** Add the API capability probe to the public launch readiness summary before any mode registration; a failed probe keeps the runtime unregistered.
    - **Run:** Run `ros2 pkg prefix px4_ros2_cpp`, `ros2 pkg prefix px4_msgs`, the generated API probe build, and the public launch. Inspect the pinned manifest/capability status and verify an intentional mismatch fails before readiness.
    - **Pass criteria:** Every production PX4 call site can be compiled from verified headers; the exact API is recorded; no undocumented signature or alternate control path is used; mismatch is a clear implementation gate failure.
    - **Regression checks:** Run all non-flight slices with mode registration disabled and the source/dependency scan.
    - **Requirements:** 5.2, 5.6, 5.7; implementation gate for design Open Decision 1.

  - [x] 7.2 Implement one `FullSelfDrivingMode`, one `FullSelfDrivingModeExecutor`, and shared PX4 state/odometry adapters
    - **Prerequisites:** 7.1 and 6.1/6.2. Use only signatures proven in 7.1.
    - **Production files/components:** Add `src/flight/full_self_driving_mode.hpp/.cpp`, `src/flight/full_self_driving_mode_executor.hpp/.cpp`, `src/adapters/px4_state_cache.hpp/.cpp`, `src/flight/internal_strategy.hpp`, and the production regular runtime integration.
    - **Implementation:** Construct exactly one registered mode named `Full Self-Driving` and exactly one owning executor. Port the useful shared prototype handling into the production adapter: global/local odometry validity, home position, land detection freshness, heading from velocity/attitude, monotonic data timeout, and safe deactivation checkpoint. Keep the mode in `WAITING_FOR_MODE` with no mission action; internal strategy selection is a domain decision, not a second scheduler.
    - **Launch update:** After lifecycle/transport/recovery/config gates pass, the runtime constructs/registers the one mode/executor; before that point it remains unregistered. Publish registration/ownership status.
    - **Run:** Launch the public simulation, inspect the registration/readiness status and PX4 ROS topics, and verify the runtime has exactly one mode/executor object through a test diagnostic. Do not arm until Task 7.3 authority smoke.
    - **Pass criteria:** One registered external mode is visible to PX4/QGroundControl; no second mode, executor, scheduler, raw PX4 publisher, or Offboard symbol exists; shared state freshness and safe deactivation are available to later strategies.
    - **Regression checks:** Repeat lifecycle/config/recovery and all perception/registry/plan/gateway tests with the registered mode inactive.
    - **Requirements:** 4.1, 5.2, 5.6, 5.7, 5.12, 6.4, 6.6.

  - [x] 7.3 Run the minimal registered-mode and authority smoke through QGroundControl/PX4
    - **Prerequisites:** 7.2. QGroundControl/PX4/RC remain the authority; Node-RED is not used to select or arm the mode.
    - **Production files/components:** Add `test/integration/registered_mode_authority_smoke.cpp`, PX4 status/readiness adapters, and an operator-safe diagnostic for active authority/takeover/deactivation.
    - **Implementation:** Verify dynamic `Full Self-Driving` visibility, mode requirements/arming checks, activation/deactivation, watchdog handling, and authority loss. The mode must perform no takeoff or setpoint mission action in this slice; it reports `WAITING_FOR_MODE` and yields immediately on PX4/QGC/RC takeover.
    - **Launch update:** Add the authority smoke observability and test hook to the same public launch; no alternate launch or mode executable is allowed.
    - **Run:** Manually run the exact public launch, connect QGroundControl, select the registered `Full Self-Driving` mode through QGC, observe PX4 status/readiness, and exercise disarm/takeover only through normal PX4/QGC/RC controls. Run the named integration test.
    - **Pass criteria:** QGC can select the registered mode; arming readiness is authoritative and lists failures; gateway has no arm/select command; PX4/RC/QGC takeover causes deactivation/yield and a durable checkpoint; watchdog/transport loss never causes a fallback controller.
    - **Regression checks:** Repeat 7.2 registration count, lifecycle order, recovery, and full preflight preparation tests.
    - **Requirements:** 4.1, 4.2, 4.3, 5.2, 5.12, 7.3.

  - [x] 7.4 Add the coordinator-owned transition property test
    - **Prerequisites:** 7.2 and 7.3.
    - **Production files/components:** Add `test/property/property_12_coordinator_transitions.cpp` and register `fsd_property_12_coordinator_transitions`.
    - **Implementation:** Generate perception observations, qualified/lost locks, health changes, and strategy decisions; assert perception emits data/events only and every transition is decided by `MissionCoordinator` and applied through the one mode/executor.
    - **Launch update:** Use the existing test transition trace in the public launch.
    - **Run:** Run the named CTest and inspect the coordinator/executor event trace while replaying target observations.
    - **Pass criteria:** Design Property 12 holds; no perception callback calls a mode-switch/API or publishes flight control.
    - **Regression checks:** Repeat registered-mode authority smoke and live-lock/registry tests.
    - **Requirements:** 5.2; **Property 12: Coordinator-owned mode transitions**.

  - [x] 7.5 Add the stronger-authority-wins property test
    - **Prerequisites:** 7.3.
    - **Production files/components:** Add `test/property/property_20_authority.cpp` and register `fsd_property_20_authority`.
    - **Implementation:** Generate PX4/QGC/RC/failsafe takeover and lower-priority gateway/UI/status events; assert the higher-priority authority wins, mode work stops, and status/evidence expose the true state.
    - **Launch update:** Add takeover fault injection to the existing authority smoke path.
    - **Run:** Run the named CTest and manually exercise QGC/RC takeover in the public simulation while observing status/events.
    - **Pass criteria:** Design Property 20 holds; no gateway/UI/cache can suppress or reinterpret takeover.
    - **Regression checks:** Repeat 7.3 authority smoke and persistence checkpoint verification.
    - **Requirements:** 4.2, 5.12, 7.3; **Property 20: Stronger safety authority wins**.

  - [x] 7.6 Add the lifecycle-before-registration property test
    - **Prerequisites:** 6.2 and 7.2.
    - **Production files/components:** Add `test/property/property_22_lifecycle_registration.cpp` and register `fsd_property_22_lifecycle_registration`.
    - **Implementation:** Generate lifecycle/transport/storage/recovery/health startup traces; assert all required lifecycle nodes are active and healthy before registration, and every transition failure withdraws readiness without a competing mode.
    - **Launch update:** Use the existing lifecycle fault hooks and registration status in the public launch.
    - **Run:** Run the named CTest and launch each lifecycle fault fixture; inspect registration sequence and readiness.
    - **Pass criteria:** Design Property 22 holds for every startup/failure ordering.
    - **Regression checks:** Repeat 6.2 lifecycle shutdown and 7.3 registered-mode smoke.
    - **Requirements:** 5.2, 6.4, 6.5; **Property 22: Lifecycle activation precedes external-mode registration**.

- [x] 8. Port takeoff and TransitIn as the first internal flight strategy
  - [x] 8.1 Add library-supported takeoff and the production internal TransitIn strategy
    - **Prerequisites:** 7.1–7.3. Read-only behavior baseline: `transit_in/TransitIn.cpp/.hpp`; use the exact verified library action/setpoint APIs, not guessed signatures.
    - **Production files/components:** Add `src/flight/strategies/takeoff_strategy.cpp`, `src/flight/strategies/transit_in_strategy.hpp/.cpp`, route value objects, and the coordinator/executor transition from `WAITING_FOR_MODE` → `TAKEOFF` → `TRANSIT_IN`.
    - **Implementation:** Port the proven TransitIn behavior: fresh land/home data gates, global/local odometry validity, target altitude above home, configured waypoint order, velocity/heading limits, first-setpoint guard, geodesic distance/altitude/vertical-velocity settle checks, monotonic timeout, and explicit failure result. Resolve all values from the locked snapshot; do not copy the prototype's node/mode/parameter names. Keep the strategy internal to the single registered mode and use only library-managed abstractions.
    - **Launch update:** The one registered mode now starts takeoff and TransitIn after QGC/PX4 grants authority and the snapshot is locked; the public launch publishes phase/checkpoint/status.
    - **Run:** Prepare/commit a valid context through the typed driver, launch the public simulation, select/arm `Full Self-Driving` through QGC, and observe takeoff, configured inbound route, phase changes, route checkpoint, and failure behavior when PX4 state becomes stale. Use the configured route fixture, not hardcoded example coordinates.
    - **Pass criteria:** Takeoff and each TransitIn route boundary require readiness and durable checkpoints; route reaches configured points with configured settle gates; stale/invalid PX4 data stops progression safely; no direct PX4 flight topic is published.
    - **Regression checks:** Repeat 7.3 authority, lifecycle, config, plan, perception, registry, and live-lock status tests before and after the flight smoke.
    - **Requirements:** 5.1, 5.2, 5.8, 6.7, 7.1.

  - [x] 8.2 Add TransitIn behavior-parity and failure-gate tests
    - **Prerequisites:** 8.1. Use copied inputs/golden traces from the read-only prototype, not the prototype executable.
    - **Production files/components:** Add `test/fixtures/prototype_behavior/transit_in/`, TransitIn entry in `prototype_behavior_map.yaml`, `test/flight/transit_in_parity_test.cpp`, and deterministic PX4 odometry/home/land fakes.
    - **Implementation:** Compare waypoint ordering, target-altitude computation, heading selection, first-update behavior, arrival/settle conditions, freshness timeout, and failure result. Add explicit `safety_change_id` coverage for snapshot/config, durability, and single-mode ownership changes.
    - **Launch update:** Add `flight_fixture:=transit_in` to the existing public launch for fake-telemetry replay; default simulation remains real PX4/SITL.
    - **Run:** Run `ctest --test-dir build/full_self_driving -R transit_in_parity --output-on-failure`, then run the public launch with the fixture and inspect phase/checkpoint/status.
    - **Pass criteria:** Production TransitIn matches proven behavior within declared tolerances and fails closed for every invalid/stale gate.
    - **Regression checks:** Repeat the manual QGC takeoff/TransitIn smoke and all preflight/status tests.
    - **Requirements:** 5.1, 5.8, 6.7.

- [x] 9. Port Search as the checkpointed internal acquisition strategy
  - [x] 9.1 Integrate the production Search strategy with WorkingPlan route/checkpoint behavior
    - **Prerequisites:** 5.1, 5.2, and 8.1. Read-only behavior baseline: `search/src/SearchMode.cpp`, `SearchPlanner.cpp`, and `PlanParser.cpp`.
    - **Production files/components:** Add `src/flight/strategies/search_strategy.hpp/.cpp`, production route/map-projection adapter, and coordinator branch `ACQUIRE_TARGET` → `SEARCH` when Direct is unavailable or disabled.
    - **Implementation:** Port Search's working-plan load/refresh, checkpoint entry point, next-source-index calculation, local map projection, climb-to-search-altitude behavior, waypoint progression/heading/speed limits, final hold, and safe deactivation checkpoint. Use the production `PlanManager` and snapshot values; do not read arbitrary plan paths or use the prototype `SearchMode`/MQTT bridge. Search publishes progress and remains data-only with respect to target lock.
    - **Launch update:** Add the internal Search strategy to the one mode/executor and expose a test acquisition fixture that withholds a qualified live lock; the same public launch visibly runs Search and updates working-plan status.
    - **Run:** Prepare a valid plan/working plan, launch/arm through QGC, select the Search fallback fixture, and observe `SEARCH`, progress/checkpoints, and deactivation/resume through the public status/read model. Verify a qualified lock is the only transition trigger out of Search.
    - **Pass criteria:** Search follows the working generation/checkpoint, not immutable source directly; progress is durable; it never silently restarts; it cannot select a mode or publish raw flight control outside the owning mode.
    - **Regression checks:** Repeat 8.1 takeoff/TransitIn, all-ID/live-lock/registry, plan reset, lifecycle, and authority takeover tests.
    - **Requirements:** 2.3, 2.7, 5.1, 5.2, 5.9, 6.7.

  - [x]* 9.2 Add Search replay/parity tests against the proven prototype planner
    - **Prerequisites:** 9.1. Use the checked-in plan fixture and copied expected route/checkpoint traces.
    - **Production files/components:** Add `test/fixtures/prototype_behavior/search/`, Search entries in `prototype_behavior_map.yaml`, `test/flight/search_parity_test.cpp`, and fake map/odometry adapters.
    - **Implementation:** Compare nested route extraction, entry-point insertion, next-source-index behavior, altitude fallback, waypoint progression, final hold, reset/resume, and deactivation update. Cover malformed plan and invalid global position failure behavior.
    - **Launch update:** Extend `flight_fixture:=search` in the same public launch.
    - **Run:** Run `ctest --test-dir build/full_self_driving -R search_parity --output-on-failure`, then launch the Search fixture and inspect phase/progress/checkpoint.
    - **Pass criteria:** Ported Search behavior matches the prototype baseline except for recorded bounded production changes; no prototype dependency is linked or launched.
    - **Regression checks:** Repeat 9.1 live Search and 8.1 TransitIn smoke.
    - **Requirements:** 2.3–2.7, 5.1, 5.8.

- [x] 10. Add Direct navigation and explicit Search fallback
  - [x] 10.1 Implement the design-approved Direct internal strategy and acquisition fallback
    - **Prerequisites:** 3.2, 4.3, 8.1, and 9.1. The current repository contains no Direct prototype source; record that absence in the behavior map and do not invent a replacement perception/landing algorithm.
    - **Production files/components:** Add `src/flight/strategies/direct_strategy.hpp/.cpp`, trusted-record/path/energy gate evaluation in `MissionCoordinator`, and the branch transitions `ACQUIRE_TARGET` → `DIRECT` or `SEARCH`.
    - **Implementation:** Use a current, trusted registry record matching locked map/scenario/identity only for a configured safe navigation position above the record. Apply path/clearance/energy/route gates; on stale/unsafe/timeout stop Direct and select Search only when the working plan is valid. Direct completion enters `PRECISION_LAND.SEARCH` and never creates a live lock, authorizes descent, verifies target, or releases payload.
    - **Launch update:** Add Direct to the internal strategy selector in the single mode/executor and add acquisition fixtures for trusted, stale, cross-scope, and unsafe registry records.
    - **Run:** Launch/arm with a trusted record and observe Direct navigation; repeat with stale/unsafe/cross-scope records and observe Search fallback; verify the live-lock topic remains unqualified until a fresh observation arrives.
    - **Pass criteria:** Direct is navigation assistance only; all branch decisions are coordinator-owned and durable; fallback is explicit and bounded; no map coordinate alone reaches Approach/Descend or payload.
    - **Regression checks:** Repeat Search/TransitIn manual smoke, registry scope tests, live-lock qualification, authority takeover, and persistence checkpoints.
    - **Requirements:** 3.1, 3.3, 3.5, 5.9, 5.10, 6.7.

  - [x]* 10.2 Add the Direct-never-substitutes-for-live-lock property test
    - **Prerequisites:** 10.1.
    - **Production files/components:** Add `test/property/property_8_direct_lock_separation.cpp` and register `fsd_property_8_direct_lock_separation`.
    - **Implementation:** Generate valid/stale/cross-scope trusted records and acquisition states; assert Direct may navigate only, cannot emit a live lock, enter descent, verify landing target, or authorize payload operation.
    - **Launch update:** Exercise the existing Direct/fallback fixtures in the public launch.
    - **Run:** Run the named CTest and inspect phase/lock/payload-gate events during Direct completion.
    - **Pass criteria:** Design Property 8 holds for all generated records and branch outcomes.
    - **Regression checks:** Repeat 10.1 Direct/Search smoke and 3.4 live-lock separation.
    - **Requirements:** 3.3, 5.9, 5.10; **Property 8: Direct never substitutes for visual lock**.

  - [x]* 10.3 Add acquisition branch integration tests for trusted Direct, stale fallback, and no-plan failure
    - **Prerequisites:** 10.1.
    - **Production files/components:** Add `test/flight/acquisition_branch_test.cpp` with deterministic registry, plan, path, energy, and lock fixtures.
    - **Implementation:** Exercise every branch and assert event IDs, strategy selection, checkpoint reason, and readiness/action error. Include the no-valid-working-plan case, which must hold/abort explicitly rather than guess.
    - **Launch update:** Add `acquisition_fixture:=...` to the same public launch and expose the decision in `MissionEvent`/status.
    - **Run:** Run the named CTest and each fixture through the public launch; inspect branch event and phase.
    - **Pass criteria:** Branch selection is deterministic, scope/age/path/energy guarded, and never bypasses live-lock qualification.
    - **Regression checks:** Repeat Search/Direct manual smoke and all earlier context/plan/registry tests.
    - **Requirements:** 3.1, 3.3, 5.9, 5.10.

- [x] 11. Port PrecisionLand as a live-lock-gated internal hierarchy
  - [x] 11.1 Implement production PrecisionLand Search/Approach/Descend/Landed_Verify
    - **Prerequisites:** 2.2, 3.1, 7.2, 9.1, and 10.1. Read-only behavior baseline: `precision_land/PrecisionLand.cpp/.hpp`.
    - **Production files/components:** Add `src/flight/strategies/precision_land_strategy.hpp/.cpp`, production camera/body/world transform adapter, bounded controller state, and coordinator transitions into/out of the hierarchy.
    - **Implementation:** Port the proven spiral search generation, target pose world transform (optical → vehicle/body → NED/world), approach altitude hold, P/PI lateral correction with clamping, descent velocity, position/velocity settle logic, target timeout, and land detection. Refactor input from legacy `/target_pose` to the production qualified `LiveTargetLock`; enforce identity/scope/freshness/quality/covariance/spatial gates. During Approach hold safe altitude on loss; during Descend stop descent and follow configured reacquisition/abort policy; complete only after stable landed verification. Emit only library-managed setpoint abstractions from the owning `FullSelfDrivingMode`.
    - **Launch update:** Add the hierarchical PrecisionLand strategy after Direct/Search acquisition in the one mode/executor; add target-lock replay and land-detection fixtures under explicit test arguments.
    - **Run:** Launch with a live-lock replay or real camera target, arm/select the registered mode through QGC, and observe `PRECISION_LAND.SEARCH`, `APPROACH`, `DESCEND`, `LANDED_VERIFY`, target-loss transitions, and status/evidence. Confirm no legacy target topic exists.
    - **Pass criteria:** Approach and descent use only fresh qualified live locks; stale/lost lock immediately stops/reverses descent per policy; landing verification precedes payload eligibility; no separate PrecisionLand mode/executor or direct trajectory topic exists.
    - **Regression checks:** Repeat Direct/Search/TransitIn, all-ID/lock/registry, QGC authority, persistence, and lifecycle tests.
    - **Requirements:** 3.3, 3.7–3.10, 5.1–5.3, 5.10, 6.7.

  - [x]* 11.2 Add PrecisionLand replay/parity tests for spiral, transforms, controller, and state flow
    - **Prerequisites:** 11.1. Use copied/golden inputs from the prototype and synthetic deterministic odometry/target traces.
    - **Production files/components:** Add `test/fixtures/prototype_behavior/precision_land/`, PrecisionLand entry in `prototype_behavior_map.yaml`, and `test/flight/precision_land_parity_test.cpp`.
    - **Implementation:** Compare spiral waypoint order, optical/body/world transform, approach altitude capture, P/PI/clamp outputs, target timeout, position/velocity settle gates, land completion, and target-loss transitions. Record production lock/persistence/authority differences as safety changes.
    - **Launch update:** Extend the target-lock/land-detection replay fixture in the same public launch.
    - **Run:** Run `ctest --test-dir build/full_self_driving -R precision_land_parity --output-on-failure`, then launch the replay and inspect strategy state/setpoint abstraction diagnostics.
    - **Pass criteria:** Ported proven behavior matches golden traces within tolerances; safety/API changes are explicit and tested; no prototype runtime dependency is used.
    - **Regression checks:** Repeat 11.1 live-lock-gated landing smoke and all previous flight slices.
    - **Requirements:** 5.3, 7.1.

  - [x]* 11.3 Add the precision-landing freshness property test
    - **Prerequisites:** 11.1.
    - **Production files/components:** Add `test/property/property_13_precision_land_freshness.cpp` and register `fsd_property_13_precision_land_freshness`.
    - **Implementation:** Generate lock age, quality, covariance, identity, spatial consistency, target-loss timing, and phase sequences; assert only valid fresh locks drive Approach/Descend and loss stops/reverses descent according to policy.
    - **Launch update:** Use existing target-loss fault injection in the public launch.
    - **Run:** Run the named CTest and replay stale/lost lock cases through the public launch while inspecting phase/events.
    - **Pass criteria:** Design Property 13 holds for all generated lock/phase sequences.
    - **Regression checks:** Repeat 11.1 live lock and 10.2 Direct separation tests.
    - **Requirements:** 3.7–3.9, 5.3; **Property 13: Precision landing freshness**.

- [x] 12. Add payload safety, second takeoff, TransitOut, and ReturnStrategy
  - [x] 12.1 Implement the named payload adapter/controller and disarmed preparation feedback
    - **Prerequisites:** 4.1, 4.3, 6.3, and 11.1. Use a deterministic simulation/fault-injection adapter; do not expose GPIO/servo/pulse fields.
    - **Production files/components:** Add `src/payload/payload_controller.hpp/.cpp`, `src/payload/simulation_payload_adapter.cpp`, `msg/PayloadStatus`, `PreparePayload.srv` integration, and named operation/state validators.
    - **Implementation:** Support only configured named preparation operations, commanded-versus-feedback state, `cargo_loaded`/`secured`, adapter health, bounded latency, and preparation idempotency. Keep all physical mapping inside the adapter and all gateway operations disarmed-only; internal release remains inaccessible to Node-RED.
    - **Launch update:** Add the simulation payload adapter/status to the public launch and expose pre-arm `prepare_payload` through the typed gateway.
    - **Run:** Through Node-RED/MQTT test fixture or typed driver while disarmed, run `OPEN_FOR_LOADING`, `VERIFY_SECURED`, and `PREPARE_FOR_SORTIE`; inspect `/full_self_driving/payload/status`. Repeat while armed/locked and with feedback faults.
    - **Pass criteria:** Preparation state derives only from approved feedback; missing/contradictory feedback blocks readiness; no raw actuator or in-flight release command is accepted.
    - **Regression checks:** Repeat context/readiness, gateway boundary, persistence, lifecycle, and perception/registry launch tests.
    - **Requirements:** 2.1, 4.2, 4.4, 5.4, 7.1, 7.9.

  - [x] 12.2 Implement durable internal payload operation, idempotency, unknown-result handling, and sequence transition
    - **Prerequisites:** 12.1, 6.1, and 11.1. The operation is requested only by the coordinator after `LANDED_VERIFIED`.
    - **Production files/components:** Add `src/flight/payload_operation_strategy.cpp`, durable payload intent/result records, `PayloadOperationRequest/Result` domain types, and executor transitions to `TAKEOFF_AFTER_DELIVERY` or configured safe return/abort.
    - **Implementation:** Gate landing/stability/target identity/live-lock/policy/count/adapter/feedback; persist intent before command and result after feedback; return the prior durable result for duplicate operation ID; mark timeout/contradictory/power-loss outcome `UNKNOWN` and never auto-retry or generate a new operation ID.
    - **Launch update:** Add payload operation to the one mode/executor and status/evidence path; no gateway release command is added.
    - **Run:** Run the nominal simulation fixture through verified landing and inject success, explicit failure, timeout, restart, duplicate ID, and contradictory feedback. Inspect payload result, event, recovery, and phase.
    - **Pass criteria:** Payload operation occurs only after all gates and durable intent; success is durable before second takeoff; unknown never retries and follows configured return/recovery; duplicate operation is idempotent.
    - **Regression checks:** Repeat 11.1 landing, gateway negative commands, persistence fault/restart, and authority takeover tests.
    - **Requirements:** 5.1, 5.4, 5.11, 6.2, 6.7, 6.9.

  - [x] 12.3 Add second takeoff and port TransitOut as a distinct internal route strategy
    - **Prerequisites:** 8.1, 8.2, and 12.2. Read-only behavior baseline: `transit_out/TransitOut.cpp/.hpp`; do not reverse TransitIn automatically.
    - **Production files/components:** Add `src/flight/strategies/transit_out_strategy.hpp/.cpp`, outbound route value/config validation, and the transition `TAKEOFF_AFTER_DELIVERY` → `TRANSIT_OUT`.
    - **Implementation:** Port TransitOut's proven fresh land/home/odometry/heading/waypoint/settle/timeout behavior into new production names and the common adapter, but load a distinct snapshot outbound route/altitude/policy. Persist result/checkpoint before each boundary and require second-takeoff gates before route progression.
    - **Launch update:** Add second takeoff and TransitOut to the one mode/executor; configure a distinct outbound fixture and expose phase/checkpoint status.
    - **Run:** Run the nominal payload-success simulation through second takeoff and configured outbound route using QGC/PX4 authority; inspect phases/checkpoints and inject stale data/route failure.
    - **Pass criteria:** Successful payload result is durable before second takeoff; TransitOut follows its own configured route and gates; no inbound reversal assumption or separate mode exists.
    - **Regression checks:** Repeat TransitIn parity/manual smoke, payload unknown/failure cases, PrecisionLand lock-loss, and authority tests.
    - **Requirements:** 5.1, 5.5, 5.8, 5.11, 6.7.

  - [x] 12.4 Implement explicit ReturnStrategy and verified recovery landing
    - **Prerequisites:** 12.3 and 7.1. Use only the verified documented library RTL/land action when configured; otherwise use the configured internal route strategy.
    - **Production files/components:** Add `src/flight/return_strategy.cpp`, configured route/route-then-library-RTL/library-RTL adapters, recovery target/landing verification, and final sortie completion/reset transitions.
    - **Implementation:** Resolve ReturnStrategy from the locked snapshot, verify route/action/reference/energy/geofence gates, handle payload failure/unknown and health degradation, preserve PX4 authority, and finalize only after recovery landing/evidence/durable completion. Never assume one RTL behavior or one field home coordinate.
    - **Launch update:** Add ReturnStrategy to the same executor schedule and publish `RETURN_STRATEGY`/`RETURN_LANDED`/completion status. Keep `simulation:=false` deferred.
    - **Run:** Exercise configured route, route-then-library-RTL, and library-RTL fixtures through the public simulation; inject payload unknown, energy, link, and takeover faults; inspect recovery landing verification and final snapshot.
    - **Pass criteria:** Explicit configured return behavior is selected/verified; failure/unknown follows safe return/abort; completion is durable and disarmed reset is required for the next editable context.
    - **Regression checks:** Repeat the full nominal path through 12.3 and all authority/persistence/lock tests.
    - **Requirements:** 5.5, 5.12, 6.2, 6.7, 6.9.

  - [x]* 12.5 Add the payload-operation safety property test
    - **Prerequisites:** 12.1 and 12.2.
    - **Production files/components:** Add `test/property/property_14_payload_safety.cpp` and register `fsd_property_14_payload_safety`.
    - **Implementation:** Generate landing/stability/target/policy/count/feedback/idempotency/unknown sequences; assert operation only follows all gates, duplicate IDs cause at most one side effect, and unknown never automatically retries.
    - **Launch update:** Use the existing payload fault fixtures in the public launch.
    - **Run:** Run the named CTest and the nominal/unknown payload simulation sequence; inspect durable intent/result and operation count.
    - **Pass criteria:** Design Property 14 holds for all generated command sequences.
    - **Regression checks:** Repeat 12.2 payload and 11.1 landing smoke.
    - **Requirements:** 5.4; **Property 14: Payload operation safety**.

  - [x]* 12.6 Add the explicit ReturnStrategy property test
    - **Prerequisites:** 12.4.
    - **Production files/components:** Add `test/property/property_15_return_strategy.cpp` and register `fsd_property_15_return_strategy`.
    - **Implementation:** Generate route, configured library-action, energy, geofence, and failure/unknown combinations; assert outbound/recovery behavior always comes from the locked snapshot and never assumes inbound reversal or implicit RTL.
    - **Launch update:** Exercise return-strategy fixtures through the public launch.
    - **Run:** Run the named CTest and each configured return fixture; inspect phase/evidence.
    - **Pass criteria:** Design Property 15 holds and invalid configuration fails before readiness.
    - **Regression checks:** Repeat 12.3/12.4 full return smoke.
    - **Requirements:** 5.5; **Property 15: Return-strategy explicitness**.

  - [x]* 12.7 Add the end-to-end mission sequence ordering property/integration test
    - **Prerequisites:** 8.1, 9.1, 10.1, 11.1, 12.2, 12.3, and 12.4.
    - **Production files/components:** Add `test/property/property_11_mission_sequence.cpp` and `test/integration/nominal_sortie_sequence_test.cpp`, registering `fsd_property_11_mission_sequence`.
    - **Implementation:** Generate valid/invalid event/action sequences and assert each transition requires its guards and durable boundary in the order takeoff → TransitIn → Direct/Search → live-lock-gated PrecisionLand → landed verification → payload → second takeoff → TransitOut/return route → ReturnStrategy/recovery landing.
    - **Launch update:** Use the complete nominal/fault fixture in the public launch and collect ordered `MissionEvent` traces.
    - **Run:** Run the named CTest and manually execute the configured nominal sortie through QGC/PX4; compare the event trace and status phases.
    - **Pass criteria:** Design Property 11 holds; rejected transitions leave state unchanged or enter explicit safety overlay; no action is skipped or silently advanced.
    - **Regression checks:** Repeat each individual flight slice and authority/persistence fault test.
    - **Requirements:** 5.1–5.5; **Property 11: Mission sequence ordering**.

- [x] 13. Add truthful status, durable evidence, and noninterfering observability
  - [x] 13.1 Complete the DashboardStatus/read-model projection and operator-safe status paths
    - **Prerequisites:** 4.1, 4.3, 6.3, and the flight slices through 12.4.
    - **Production files/components:** Complete `src/runtime/dashboard_status.cpp`, `/full_self_driving/status` and related status publishers, gateway read-model mapping, and QGC application-presence adapter state.
    - **Implementation:** Publish complete consistent snapshots containing config/flight state, health, registration/Ownmode/armed/landed/takeoff, executor phase/action, plan/checkpoint/progress, selected target/registry/live lock, payload, persistence, recovery, failures, and safe action. Distinguish ground-link health from optional QGC GUI presence; never infer GUI presence from telemetry. Status is read-only and cannot authorize a mutation.
    - **Launch update:** Replace partial probes with the complete dashboard projection in the same public launch.
    - **Run:** Run the public launch through preparation, an active phase, gateway disconnect/reconnect, takeover, and recovery; inspect `/full_self_driving/status` as a complete snapshot and verify retained/transient-local late-join behavior.
    - **Pass criteria:** All fields are truthful/complete and internally consistent; stale/retained status cannot authorize; ground-link and QGC-presence values remain distinct.
    - **Regression checks:** Repeat nominal/fault sequence, gateway boundary, lifecycle, and persistence tests.
    - **Requirements:** 4.3, 4.4, 7.1, 7.3, 7.4, 7.8.

  - [x] 13.2 Implement ordered MissionEvent/evidence recording and final manifest correlation
    - **Prerequisites:** 6.1 and 13.1. Evidence must not block the mode-update path.
    - **Production files/components:** Complete `src/evidence/evidence_sink.hpp/.cpp`, `fsd_evidence` lifecycle node, `MissionEvent`/manifest writers, event correlation helpers, bounded drop/gap reporting, and final manifest generation.
    - **Implementation:** Record config/commit/lock, lifecycle, action intent/result, route checkpoint, target-lock transitions, landing verification, payload intent/feedback/result, takeover/deactivation, recovery decision, and completion with mission ID, sortie ID, snapshot hash, event/durable sequence, source/component, and idempotency key. Use asynchronous bounded queues and explicitly mark non-durable/dropped records.
    - **Launch update:** Wire evidence into every runtime boundary and add manifest/status to the public launch; evidence failure changes observability/durability status but never creates a control path.
    - **Run:** Execute the nominal and fault-injected public launch, then use the typed `get_evidence_manifest` inspection operation to verify ordered correlated records and intentional gaps.
    - **Pass criteria:** Every safety-relevant boundary has ordered correlated evidence; manifests finalize atomically; evidence/storage failure is visible and nonblocking.
    - **Regression checks:** Repeat persistence fault/recovery, full mission, gateway disconnect, and takeover tests.
    - **Requirements:** 6.1–6.7, 7.2, 7.8, 7.9.

  - [x] 13.3 Add bounded asynchronous logging, diagnostics, metrics, and optional tracing
    - **Prerequisites:** 13.1 and 13.2. Observability may gate readiness only when the resolved policy says so.
    - **Production files/components:** Add `src/observability/structured_logger.cpp`, diagnostics publishers, fixed metric definitions/labels, asynchronous exporters, bounded rotating logs, and optional trace adapters configured by ID.
    - **Implementation:** Implement the design metric set for registration/activation, library-managed update/setpoint abstraction rate, action duration/result, persistence/checkpoint, queues/drops, lifecycle, PX4 freshness, resources, payload feedback, and target-lock latency. Exclude raw images/PX4 loops/secrets/paths from logs/traces; never block the mode update path.
    - **Launch update:** Add observability components/timers to the public launch with configured asynchronous queues and failure diagnostics.
    - **Run:** Run the public launch with metrics/evidence exporter success, delay, and failure fixtures; inspect `/diagnostics`, status, logs, and bounded queue/drop counters during a nominal/fault sortie.
    - **Pass criteria:** Exporter/evidence/dashboard failure does not block or control flight; metrics/diagnostics remain bounded and truthful; no secret/raw-control data is emitted.
    - **Regression checks:** Repeat the full sequence, lifecycle, resource, persistence, and gateway security tests.
    - **Requirements:** 7.1, 7.2, 7.4, 7.5, 7.8.

  - [x]* 13.4 Add the status-observability truthfulness property test
    - **Prerequisites:** 13.1.
    - **Production files/components:** Add `test/property/property_18_status_truthfulness.cpp` and register `fsd_property_18_status_truthfulness`.
    - **Implementation:** Generate component health, PX4/ground-link, QGC-presence, stale/retained status, flight, target, payload, persistence, and recovery states; assert complete truthful snapshots and no GUI-presence inference.
    - **Launch update:** Use the existing status fault fixtures in the public launch.
    - **Run:** Run the named CTest and inspect status across gateway disconnect, retained snapshot, and live updates.
    - **Pass criteria:** Design Property 18 holds for all read-model states.
    - **Regression checks:** Repeat 13.1 status smoke and 7.5 authority test.
    - **Requirements:** 7.1, 7.4; **Property 18: Status observability truthfulness**.

  - [x]* 13.5 Add the evidence-correlation property test
    - **Prerequisites:** 13.2.
    - **Production files/components:** Add `test/property/property_19_evidence_correlation.cpp` and register `fsd_property_19_evidence_correlation`.
    - **Implementation:** Generate mission boundaries, event duplicates, queue drops, restarts, locks, payload results, recovery decisions, and completion; assert correlation IDs/sequences and ordered durable/non-durable truth.
    - **Launch update:** Use the existing evidence trace/manifest path.
    - **Run:** Run the named CTest and inspect the final manifest after nominal and fault-injected public launches.
    - **Pass criteria:** Design Property 19 holds; duplicate idempotency keys do not duplicate side effects or corrupt event order.
    - **Regression checks:** Repeat persistence and full-sortie evidence smoke.
    - **Requirements:** 7.2; **Property 19: Evidence correlation**.

  - [x]* 13.6 Add the observability noninterference property test
    - **Prerequisites:** 13.3.
    - **Production files/components:** Add `test/property/property_25_observability_noninterference.cpp` and register `fsd_property_25_observability_noninterference`.
    - **Implementation:** Generate exporter stalls, evidence backlog, log rotation/storage failures, diagnostics drops, and metrics load; assert mode updates remain bounded and no observability component can select a mode or emit control.
    - **Launch update:** Use the existing observability fault fixtures.
    - **Run:** Run the named CTest and stress/fault public launch; inspect update-rate, queue/drop, and safety status.
    - **Pass criteria:** Design Property 25 holds; degraded observability is visible without blocking flight or creating a fallback.
    - **Regression checks:** Repeat 13.3 metrics smoke and 7.3 authority smoke.
    - **Requirements:** 7.1, 7.5; **Property 25: Observability noninterference and truthfulness**.

- [x] 14. Harden the implementation and prove negative/security boundaries
  - [x] 14.1 Add repository-wide source, dependency, generated-interface, and launch compliance scans
    - **Prerequisites:** All implementation slices through 13.3. This task must scan production files, test fixtures, manifests, generated interfaces, and installed launch metadata.
    - **Production files/components:** Add `test/security/production_boundary_scan.py`, CMake/CTest registration, dependency allowlist, and a generated report fixture used only by CI.
    - **Implementation:** Fail on prototype imports/links/launches/contracts, `gazebo_models` runtime paths, old package/topic/parameter names, Offboard symbols/topics, direct flight-control publishers, generic setpoint services, raw GPIO/servo/path/executable/JSON fields, second schedulers/modes/executors, and unbounded public fields. Allow prototype names only in explicitly marked read-only parity fixture metadata.
    - **Launch update:** Add a pre-readiness boundary check to the public launch and keep the runtime launch graph production-only.
    - **Run:** Run the named CTest/scan and the exact public launch; intentionally inject a forbidden fixture reference in a temporary test file to verify the scan fails, then remove it.
    - **Pass criteria:** The clean production tree passes; any prohibited path blocks readiness/CI; old prototype files remain unmodified.
    - **Regression checks:** Run the complete prior CTest suite and live nominal simulation smoke.
    - **Requirements:** 1.3, 1.4, 4.2, 5.6, 5.7, 7.1, 7.7.

  - [x] 14.2 Add TLS/MQTT ACL/DDS-Security and typed-input negative integration tests
    - **Prerequisites:** 6.3, 13.1, and 14.1. Use test certificates/broker identities and local protected secret fixtures; never use real credentials.
    - **Production files/components:** Add `test/security/gateway_security_test.cpp`, broker/TLS fixtures, DDS-Security permission fixtures where supported, manifest/config/plan sanitization tests, and certificate rotation/revocation checks.
    - **Implementation:** Test wrong issuer/SAN/expiry/revocation/clock, broker ACL violations, retained/replayed commands, unauthorized DDS participants, malformed bounded requests, traversal/symlink/oversized plans/configs/manifests, evidence rollback, secret permissions/redaction, and security failure behavior. Rejections must not mutate state or create alternate control.
    - **Launch update:** Add `security_fixture:=...` to the public launch; default launch uses approved secure configuration and no plaintext fallback.
    - **Run:** Run the named CTest and each security fixture against the public launch/local broker; inspect bounded audit errors, status, and side-effect counters.
    - **Pass criteria:** Security failures fail closed, remain observable, preserve authoritative state, and produce no PX4/flight/payload/filesystem/control side effect; certificate rotation does not replay commands.
    - **Regression checks:** Repeat gateway allowed-command, lifecycle, status, persistence, and exact launch smoke.
    - **Requirements:** 4.2, 4.3, 7.6, 7.7, 7.8.

  - [x] 14.3 Add resource, queue, storage, evidence, and adapter-failure proof
    - **Prerequisites:** 6.1, 12.1, 13.2, and 13.3.
    - **Production files/components:** Add deterministic slow/failing adapter fakes, image/event/MQTT flood fixtures, storage reserve checks, CPU/memory/temperature/queue tests, and `test/integration/resource_failure_test.cpp`.
    - **Implementation:** Exercise bounded latest-image queues, ordered event backpressure, MQTT size/rate/age limits, artifact/JSON bounds, storage reserve, slow payload/camera/PX4/evidence/metrics adapters, and resource degradation. Affected operations must hold/fail closed without blocking the registered mode update path or adding a fallback.
    - **Launch update:** Add resource/fault injection only under explicit test arguments; publish queue/drop/resource/health status through the normal launch.
    - **Run:** Run the named CTest and public launch with each flood/slow/failure fixture; inspect diagnostics, safety overlay, update-rate, and durable sequence.
    - **Pass criteria:** Bounds are enforced, no unbounded queue or memory growth occurs in the test budget, and every affected operation has an explicit safe result.
    - **Regression checks:** Repeat full nominal sequence, observability noninterference, persistence fault, and authority takeover tests.
    - **Requirements:** 6.3, 7.1, 7.5, 7.6, 7.7.

  - [x]* 14.4 Add the security-rejection-no-side-effect property test
    - **Prerequisites:** 14.1 and 14.2.
    - **Production files/components:** Add `test/property/property_26_security_rejection.cpp` and register `fsd_property_26_security_rejection`.
    - **Implementation:** Generate invalid identity/ACL/certificate/replay/bounds/path/config/plan/manifest/resource/raw-control requests; assert rejection/audit, unchanged authoritative revision, no PX4/flight/payload/filesystem side effect, and no alternate control path.
    - **Launch update:** Use the existing security fixture path in the public launch.
    - **Run:** Run the named CTest and the security fixture against the live public launch; inspect side-effect counters and audit events.
    - **Pass criteria:** Design Property 26 holds for all generated security-invalid inputs.
    - **Regression checks:** Repeat 14.2 security integration and 14.1 source scan.
    - **Requirements:** 4.2, 7.6, 7.7; **Property 26: Security rejection has no flight side effect**.

- [x] 15. Complete exact simulation acceptance and adapter-invariance proof
  - [x] 15.1 Run the complete end-to-end simulation acceptance suite
    - **Prerequisites:** All required tasks 1.1–14.3; optional property tests should be enabled in the acceptance profile when available.
    - **Production files/components:** Add `test/acceptance/full_sortie_acceptance.py`, complete fixture catalog/config, launch readiness summary assertions, and CI/CTest registration for the exact acceptance command.
    - **Implementation:** Exercise preparation via typed gateway, immutable plan/working generation, target/registry, commit/lock, QGC registered-mode selection/arming, takeoff, TransitIn, trusted Direct or Search fallback, live-lock PrecisionLand, landing verification, named payload feedback, second takeoff, distinct TransitOut, configured ReturnStrategy, recovery landing, evidence manifest, disarm/reset, and fault branches for stale target, takeover, energy, transport, persistence, unknown payload, and restart. Do not bypass QGC/PX4 authority in the flight test.
    - **Launch update:** Freeze `full_self_driving.launch.py` as the one production launch and make the exact command below the acceptance entry point; all test fixtures are selected through approved config/test arguments, not alternate launch files.
    - **Run:** Manually run exactly `ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:=kmitl_airfield headless:=false`, connect QGC, and run `ctest --test-dir build/full_self_driving -R fsd_acceptance --output-on-failure` against the same launch profile. Inspect readiness, phases, status, events, payload, recovery, and final manifest.
    - **Pass criteria:** The exact command starts the selected world/resources, manifest PX4 SITL, MicroXRCE-DDS, `/clock`/camera/TF bridges, all production nodes, readiness summary, supervision, and reverse shutdown; the complete sortie and safety branches pass without prototype/Offboard dependencies.
    - **Regression checks:** Run the complete `colcon test`, source/dependency scan, API probe, and all prior vertical-slice manual smoke procedures.
    - **Requirements:** 1.1–1.9, 2.1–2.11, 3.1–3.10, 4.1–4.5, 5.1–5.12, 6.1–6.9, 7.1–7.9.

  - [x] 15.2 Verify clean install, one-launch ownership, restart/recovery, and regression from an isolated workspace
    - **Prerequisites:** 15.1. Do not alter the prototype or rely on an already sourced prototype install.
    - **Production files/components:** Add isolated-workspace CI script/configuration, install-tree launch/package checks, and final test report collection under `test/acceptance/`.
    - **Implementation:** Build/install only `full_self_driving` plus declared external dependencies, inspect the install tree for exactly one public launch, start from a clean state directory, restart at every action boundary, and verify no hidden prototype/gazebo_models runtime path exists. Confirm `simulation:=false` fails clearly without an approved hardware manifest.
    - **Launch update:** Finalize install-time resource lookup and fail-closed hardware branch in the same public launch; remove any temporary foundation-only production path that is no longer needed.
    - **Run:** In a clean shell, source only the ROS/PX4 overlays and production install, run the exact acceptance command, then run `ros2 pkg prefix full_self_driving`, inspect installed launch/resources, and run the full CTest suite. Run `ros2 launch full_self_driving full_self_driving.launch.py simulation:=false` and verify `HARDWARE_PROFILE_NOT_CONFIGURED` when no manifest is approved.
    - **Pass criteria:** A clean install reproduces the full behavior with one launch; restart/recovery is explicit and safe; hardware false branch never starts fake/simulated components; all earlier slices regress cleanly.
    - **Regression checks:** Repeat 15.1 exact command, all source/security/API scans, and final git diff check to ensure prototype/design/requirements files were not modified.
    - **Requirements:** 1.3, 1.5–1.8, 5.6, 6.2, 6.4–6.6, 7.7.

  - [x]* 15.3 Add the simulation/hardware adapter-invariance property test
    - **Prerequisites:** 15.1 and 15.2. Use the existing approved profile-manifest schema; do not require real Pi hardware.
    - **Production files/components:** Add `test/property/property_24_adapter_invariance.cpp` and register `fsd_property_24_adapter_invariance`.
    - **Implementation:** Generate approved simulation/hardware adapter selections and compare domain policy, snapshot hashes/ownership boundaries, ROS contracts, persistence protocol, Node-RED preparation API, and mode/executor ownership. Assert only declared transport/camera/TF/payload/telemetry/process/resource adapters differ.
    - **Launch update:** Exercise `simulation:=true` and a complete fake/declared hardware manifest through the same `full_self_driving.launch.py`; no second launch file is allowed.
    - **Run:** Run the named CTest, the exact simulation launch, and the hardware-manifest validation branch without starting fake hardware.
    - **Pass criteria:** Design Property 24 holds; adapter selection cannot alter domain safety rules or add a control path.
    - **Regression checks:** Repeat 15.1/15.2 exact launch and source scan.
    - **Requirements:** 1.1, 1.8; **Property 24: Simulation/hardware adapter invariance**.

- [x] 16. Defer Raspberry Pi 4 hardware bringup behind an explicit manifest gate
  - [x]* 16.1 Add the deferred hardware manifest/schema and fail-closed `simulation:=false` branch
    - **Prerequisites:** 15.2. This task is **deferred/optional** until an approved real FMU, camera, payload, telemetry, clock/TF, process, resource, permission, calibration, and power-loss validation package exists.
    - **Production files/components:** Add `simulation/manifests/hardware_schema.yaml`, `src/launch/hardware_manifest_validator.cpp`, hardware adapter interface stubs with no fake runtime implementation, and tests for `HARDWARE_PROFILE_NOT_CONFIGURED`.
    - **Implementation:** Validate executable identity, device/resource paths, permissions, adapter IDs, calibration/TF, security material, and approval evidence. Reject incomplete/deferred Raspberry Pi 4 bringup; never silently substitute Gazebo, fake payload/camera, or simulated telemetry. Keep domain contracts and the one launch entry point unchanged.
    - **Launch update:** Add only the explicit manifest-selected hardware branch to `full_self_driving.launch.py`; without a complete approved manifest it fails before readiness and does not start simulated or fake components.
    - **Run:** Run `ros2 launch full_self_driving full_self_driving.launch.py simulation:=false` without a manifest and verify `HARDWARE_PROFILE_NOT_CONFIGURED`; run validator tests with incomplete/tampered manifests. Do not claim or perform real Pi 4 bringup in this task.
    - **Pass criteria:** Hardware remains clearly deferred until approval/evidence; the false branch fails closed; no new control path, prototype dependency, or alternate launch is introduced.
    - **Regression checks:** Repeat 15.2 exact simulation launch, adapter-invariance test, source scan, and all production CTest cases.
    - **Requirements:** 1.6, 1.7, 1.8, 7.6, 7.7.

## Checkpoints

- [x] Checkpoint A — After Task 1: ensure the standalone package builds, the single launch starts the manifest-driven simulation dependencies, readiness is explicit, and no prototype/runtime boundary violation exists.
- [x] Checkpoint B — After Task 3: ensure live camera input produces all-ID observations, selected-target qualification is separate, registry scopes are isolated, and no flight behavior depends on an unqualified record.
- [x] Checkpoint C — After Task 6: ensure config/context/plan/gateway mutations are typed, revisioned, disarmed-only, durable, lifecycle-supervised, and recover safely before any mode registration.
- [x] Checkpoint D — After Task 7: ensure the actual pinned PX4 APIs compile, exactly one registered mode/executor exists, QGC/PX4 authority is observable, and no flight behavior uses Offboard/raw control.
- [x] Checkpoint E — After Task 11: ensure takeoff/TransitIn/Search/Direct/PrecisionLand are internal strategies, live-lock gates descent, and all earlier camera/plan/persistence/authority slices still pass.
- [x] Checkpoint F — After Task 12: ensure payload unknown/failure behavior is non-retrying, second takeoff/TransitOut/ReturnStrategy are explicit, and the complete sequence is durably ordered.
- [x] Checkpoint G — After Task 15: run the exact acceptance command, full regression/security/source scans, clean-install test, and confirm only the production package is in the runtime graph.

## Notes

- Tasks marked with `*` are optional test/property-hardening tasks under the workflow convention; core implementation tasks and their runnable smoke criteria remain required. Optional tasks still appear in the dependency graph.
- Task 16.1 is explicitly deferred/optional Raspberry Pi 4 bringup work. It validates the fail-closed branch and manifest boundary only; it does not claim hardware support.
- Every task updates or exercises the same `full_self_driving.launch.py`; no task may introduce a second public launch entry point.
- Test-only replay/fault fixtures may refer to read-only prototype source paths as provenance metadata, but production CMake/package/runtime code must not import, link, install, launch, or expose prototype contracts.
- The prototype behavior baseline is ported, not replaced. Production-only changes are limited to the approved generic configuration, bounded typed ROS contract, lifecycle, authority, freshness, persistence, security, and single-mode ownership constraints and must carry a test-visible `safety_change_id`.
- A `Correctness Properties` section exists in `design.md`; therefore each universal property is represented by its own property-test sub-task. Unit, integration, launch, security, and hardware-fault tests remain complementary.
- No task asks an implementation agent to modify `design.md`, `requirements.md`, the prototype packages, or `gazebo_models`.

## Task Dependency Graph

```json
{
  "waves": [
    { "id": 0, "tasks": ["1.1"] },
    { "id": 1, "tasks": ["1.2"] },
    { "id": 2, "tasks": ["1.3"] },
    { "id": 3, "tasks": ["2.1"] },
    { "id": 4, "tasks": ["2.2"] },
    { "id": 5, "tasks": ["2.3"] },
    { "id": 6, "tasks": ["3.1"] },
    { "id": 7, "tasks": ["3.2"] },
    { "id": 8, "tasks": ["3.3", "3.4"] },
    { "id": 9, "tasks": ["4.1"] },
    { "id": 10, "tasks": ["4.2"] },
    { "id": 11, "tasks": ["4.3"] },
    { "id": 12, "tasks": ["4.4", "4.5", "4.6", "4.7", "4.8"] },
    { "id": 13, "tasks": ["5.1"] },
    { "id": 14, "tasks": ["5.2"] },
    { "id": 15, "tasks": ["5.3", "5.4", "5.5"] },
    { "id": 16, "tasks": ["6.1"] },
    { "id": 17, "tasks": ["6.2"] },
    { "id": 18, "tasks": ["6.3"] },
    { "id": 19, "tasks": ["6.4", "6.5", "6.6", "6.7"] },
    { "id": 20, "tasks": ["7.1"] },
    { "id": 21, "tasks": ["7.2"] },
    { "id": 22, "tasks": ["7.3"] },
    { "id": 23, "tasks": ["7.4", "7.5", "7.6"] },
    { "id": 24, "tasks": ["8.1"] },
    { "id": 25, "tasks": ["8.2"] },
    { "id": 26, "tasks": ["9.1"] },
    { "id": 27, "tasks": ["9.2"] },
    { "id": 28, "tasks": ["10.1"] },
    { "id": 29, "tasks": ["10.2", "10.3"] },
    { "id": 30, "tasks": ["11.1"] },
    { "id": 31, "tasks": ["11.2", "11.3"] },
    { "id": 32, "tasks": ["12.1"] },
    { "id": 33, "tasks": ["12.2"] },
    { "id": 34, "tasks": ["12.3"] },
    { "id": 35, "tasks": ["12.4"] },
    { "id": 36, "tasks": ["12.5", "12.6", "12.7"] },
    { "id": 37, "tasks": ["13.1"] },
    { "id": 38, "tasks": ["13.2"] },
    { "id": 39, "tasks": ["13.3"] },
    { "id": 40, "tasks": ["13.4", "13.5", "13.6"] },
    { "id": 41, "tasks": ["14.1"] },
    { "id": 42, "tasks": ["14.2"] },
    { "id": 43, "tasks": ["14.3"] },
    { "id": 44, "tasks": ["14.4"] },
    { "id": 45, "tasks": ["15.1"] },
    { "id": 46, "tasks": ["15.2"] },
    { "id": 47, "tasks": ["15.3"] },
    { "id": 48, "tasks": ["16.1"] }
  ]
}
```
