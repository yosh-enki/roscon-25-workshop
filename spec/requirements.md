# Requirements Document

## Introduction

This document defines the requirements for **Full Self-Driving**, a generic autonomous-sortie product implemented as one standalone ROS 2 package named `full_self_driving`. The product supports multiple configured maps, scenarios, vehicles, payload adapters, routes, target identities, and deployment profiles without embedding site-specific mission values in flight code.

The product has two separated authority planes. Node-RED/MQTT is a typed preparation and inspection plane that operates only while the vehicle is disarmed. QGroundControl and PX4 remain the authorities for selecting the registered `Full Self-Driving` external mode and arming. Once PX4 grants that mode, one companion executor conducts the committed sortie under PX4 safety authority.

The requirements preserve the approved design boundary: the existing prototype, old launch files, old scripts, and `gazebo_models` assets are read-only behavioral or asset references and are never production runtime dependencies. Integrated simulation is the initial launch scope. Raspberry Pi 4 hardware bringup remains deferred until a complete approved hardware manifest and its validation evidence exist.

## Glossary

- **Full_Self_Driving_Product**: The complete production product delivered by the `full_self_driving` package, including runtime nodes, domain libraries, interfaces, launch integration, persistence, evidence, adapters, and operator gateway.
- **ROS_2**: The selected Robot Operating System 2 distribution and its runtime, lifecycle, interface-generation, security, and communication facilities.
- **full_self_driving package**: The one standalone ROS 2 package that owns the production launch entry point, concrete ROS interfaces, runtime components, domain libraries, tests, and approved simulation integration metadata.
- **Integrated_Launch_Entry_Point**: The single public launch entry point `full_self_driving.launch.py` that starts and supervises the selected production profile and its declared dependencies.
- **Engineering_Configuration**: The one administrator-owned configuration document containing policy, route, safety, adapter, storage, resource, QoS, observability, security, and launch values for a deployment.
- **Configuration_Manager**: The component that loads, validates, canonicalizes, hashes, and exposes the read-only resolved `Engineering_Configuration`.
- **Resolved_Configuration**: The validated and normalized set of engineering configuration values used by runtime policy decisions.
- **Canonical_Hash**: The lowercase SHA-256 digest of the stable canonical serialization of a `Resolved_Configuration` or other immutable artifact.
- **Operator_Selection**: The disarmed, revisioned selection containing map, scenario, plan, working plan, target identity, and approved payload-preparation state.
- **Mission_Snapshot**: The durable committed record containing the complete resolved configuration, canonical hash, operator selection, plan and checkpoint state, target identity, payload state, and executor checkpoint.
- **Locked_Snapshot**: A `Mission_Snapshot` latched after PX4 grants the registered mode and arming begins; a locked snapshot is immutable for the active sortie.
- **Preparation_Gateway**: The typed `fsd_gateway` boundary between Node-RED/MQTT and authoritative ROS components. The gateway validates, limits, and forwards preparation or inspection operations only.
- **Node-RED**: The operator-console technology used to select and inspect managed mission context and perform approved disarmed payload preparation.
- **MQTT**: The broker protocol used by the Preparation_Gateway under the configured TLS, ACL, age, size, rate, revision, and non-retained-command policies.
- **QGroundControl**: The ground-control application that observes PX4 and selects the registered external mode through PX4; QGroundControl does not receive an arm or flight command from Node-RED.
- **PX4**: The flight controller that owns arming checks, mode authority, estimator and envelope protections, geofence, failsafes, and higher-priority takeover behavior.
- **Flight_Runtime**: The regular runtime component that hosts the Mission_Coordinator, FullSelfDrivingMode, FullSelfDrivingModeExecutor, adapters, readiness gates, and committed-sortie action boundary.
- **Perception_Service**: The perception component that publishes All_ID_Observation and Live_Target_Lock data and events without selecting a flight mode or invoking an executor transition.
- **FullSelfDrivingMode**: The one production registered external PX4 mode derived from the version-matched Auterion `px4_ros2_cpp`/`px4_ros2_interface_lib` `ModeBase` API or exact version-matched equivalent.
- **ModeBase**: The version-matched PX4 ROS 2 base-mode API from which FullSelfDrivingMode is derived.
- **FullSelfDrivingModeExecutor**: The one production owner of `FullSelfDrivingMode`, derived from the version-matched `ModeExecutorBase` API or exact version-matched equivalent, and the only companion flight scheduler.
- **ModeExecutorBase**: The version-matched PX4 ROS 2 executor API used by FullSelfDrivingModeExecutor for the registered mode and documented top-level actions.
- **Registered_Mode**: The dynamically registered `Full Self-Driving` external mode visible to QGroundControl after the runtime passes registration prerequisites.
- **Mission_Coordinator**: The domain component that evaluates mission, perception, health, policy, and persistence guards and returns typed strategy decisions; it does not publish flight setpoints or create a second scheduler.
- **Internal_Strategy**: A behavior state within the one registered mode, including `TransitIn`, `TransitOut`, `Search`, `Direct`, and `PrecisionLand`; an internal strategy is not an independently registered PX4 mode or executor.
- **TransitIn**: The configured inbound route strategy.
- **TransitOut**: The configured outbound route strategy; outbound geometry is not assumed to be the reverse of the inbound route.
- **Search**: The working-plan route strategy used to search for the selected target.
- **Direct**: The map-assisted navigation strategy that may navigate to a trusted registry coordinate but cannot establish a live target lock or authorize descent.
- **PrecisionLand**: The hierarchical landing strategy containing `Search`, `Approach`, `Descend`, and `Landed_Verify` substates.
- **Approach**: The PrecisionLand substate that holds a configured safe altitude and centers over a fresh live target lock before descent.
- **Descend**: The PrecisionLand substate that performs bounded descent using a fresh live target lock and stops descent when lock or landing gates fail.
- **Landed_Verify**: The PrecisionLand substate that verifies vehicle stability, landing state, target identity, and configured landing dwell before payload eligibility.
- **ReturnStrategy**: The snapshot-selected route, PX4/ModeExecutor RTL action, or other approved recovery behavior used after outbound travel or an unsafe payload outcome.
- **All_ID_Observation**: A bounded perception observation for any accepted marker identity, including marker ID, dictionary, namespace, pose, scope, timestamp, covariance, calibration, and quality.
- **Selected_Target**: The target identity stored in the operator selection and locked snapshot.
- **Live_Target_Lock**: A fresh, qualified perception result for the selected target after identity, scope, freshness, quality, covariance, transform, consecutive-observation, and spatial-consistency gates pass.
- **Pad_Registry**: The durable map/scenario-scoped collection of trusted marker observations and coordinates.
- **Map_Scenario_Scope**: The pair of configured map and scenario identifiers used to isolate registry, plan, target, and mission data.
- **Plan_Artifact**: An immutable, managed upload of an approved bounded QGroundControl `.plan` subset, identified by a managed ID and canonical hash rather than an operator filesystem path.
- **Working_Plan**: A generated search record separate from the immutable Plan_Artifact, containing source hash, map/scenario scope, generation, canonical route, checkpoint, and progress.
- **Plan_Manager**: The component that validates, stores, parses, prints, hashes, selects, generates, checkpoints, and resets Plan_Artifact and Working_Plan records.
- **Payload_Controller**: The approved adapter boundary for named payload preparation and internal payload operations with commanded state, hardware feedback, idempotency, durable results, and bounded failure handling.
- **Named_Payload_Operation**: An allowlisted semantic payload action such as a configured release or preparation operation; a Named_Payload_Operation does not contain raw GPIO, servo, pulse, or command bytes.
- **Persistence_Manager**: The component that performs durable snapshots, journals, checkpoints, registry records, payload records, recovery markers, and evidence boundaries.
- **Durable_Boundary**: The configured sequence of validation, temporary write, flush/fsync or equivalent, atomic replacement, directory durability, journal update, and backup handling required before state is reported durable.
- **RECOVERY_REQUIRED**: The explicit state entered when restart, corruption, interruption, or contradiction prevents safe reconciliation of safety-relevant state.
- **Lifecycle_Node**: A ROS 2 `LifecycleNode` whose configure, activate, deactivate, cleanup, and shutdown states are supervised independently.
- **Lifecycle_Supervisor**: The launch and runtime supervision responsible for lifecycle ordering, readiness withdrawal, reverse-order cleanup, and child-process supervision.
- **Regular_Runtime_Node**: The stable non-lifecycle `rclcpp::Node` that hosts the registered mode and executor and does not recreate them during lifecycle transitions.
- **ROS_Interface_Boundary**: The package-owned bounded `.msg`, `.srv`, and `.action` contract set under the `/full_self_driving` namespace.
- **Status_Read_Model**: A complete, authoritative-looking-but-read-only projection for operator display; a status projection cannot authorize a mutation or flight action.
- **Observability_Plane**: The bounded asynchronous logging, diagnostics, metrics, tracing, health, status, and evidence read plane.
- **Gazebo**: The simulation process used by the approved Simulation_Profile to provide the configured world and simulated sensors.
- **PX4_SITL**: The manifest-selected PX4 software-in-the-loop process and transport used by the integrated Simulation_Profile.
- **MicroXRCE_DDS_Agent**: The configured DDS transport agent required by the simulation or approved PX4 ROS 2 integration manifest.
- **TF_Transform**: The configured coordinate-frame transform information used by camera, perception, and bridge components.
- **TLS_mTLS**: Transport Layer Security, including mutual certificate authentication when configured, for protected MQTT and approved service connections.
- **MQTT_ACL**: The broker access-control list that restricts authenticated MQTT identities to approved topics and operations.
- **DDS_Security**: The ROS 2 DDS governance, permissions, enclave, and participant-authentication controls selected by deployment policy.
- **QoS_Profile**: The configured ROS 2 quality-of-service settings for reliability, durability, queue depth, deadline, and related delivery behavior.
- **Security_Plane**: The implementation controls for TLS/mTLS, MQTT ACLs, DDS-Security/SROS2 permissions, secrets, input validation, resource limits, launch trust, and evidence integrity.
- **Simulation_Profile**: An approved launch profile that resolves Gazebo, a selected world and resources, PX4 SITL, MicroXRCE-DDS transport, bridges, and production nodes.
- **Hardware_Profile**: A launch profile backed by a complete approved external adapter and process manifest; the initial product does not claim Raspberry Pi 4 bringup.
- **Hardware_Manifest**: The validated declaration of external PX4, camera, payload, telemetry, clock/TF, process, executable, resource, and permission dependencies.
- **KMITL_Simulation_Fixture**: The configuration/catalog fixture identified by `kmitl_airfield` for the initial integrated simulation acceptance launch; the identifier is not flight-code policy.
- **Prototype_Reference**: The existing prototype package, public contracts, launch files, scripts, or source used only as read-only behavioral reference.
- **Offboard_Control**: The forbidden PX4 companion control path using Offboard symbols or direct offboard/setpoint topics instead of the registered mode API.
- **Raw_Control_Bridge**: Any generic or direct interface that exposes PX4 flight commands, setpoints, topic/service names, GPIO, servo, executable paths, or arbitrary filesystem paths to an operator or alternate component.
- **Version_Matched_PX4_Library**: The pinned compatible Auterion `px4_ros2_cpp`/`px4_ros2_interface_lib` and `px4_msgs` release selected and checked by the deployment.
- **Revision_Guard**: An expected monotonic revision check required before an authoritative mutation is applied.
- **Idempotency_Key**: A bounded request or operation identity whose repeated use returns the retained durable result without repeating a side effect.
- **Bounded_Contract**: A message, service, action, queue, or payload with explicit size, count, enum, numeric, timing, and ownership limits.
- **Disarmed**: A vehicle state in which PX4 reports that the vehicle is not armed; disarmed-only operations are rejected in any other state.
- **Armed**: A vehicle state in which PX4 reports that the vehicle is armed; selection, registry-clear, working-plan-reset, and other preparation mutations are not permitted.
- **Flight_Authority**: The authority currently controlling flight behavior; PX4, QGroundControl, RC, and PX4 failsafes have priority over the companion runtime.
- **Unknown_Payload_Result**: A payload operation outcome that cannot be confirmed as success or explicit failure after timeout, interruption, contradictory feedback, or restart.

## Requirements

### Requirement 1: Generic standalone product and authoritative deployment configuration

**User Story:** As a deployment administrator, I want one generic product whose deployment behavior is selected by approved configuration, so that multiple maps, scenarios, vehicles, payloads, and profiles can use the same production package without site-specific flight literals.

#### Acceptance Criteria

1.1 THE Full_Self_Driving_Product SHALL derive deployment-specific routes, maps, scenarios, vehicles, target catalogs, payload adapters, safety thresholds, resource limits, and launch adapters from the Engineering_Configuration and approved catalogs, and SHALL preserve common domain behavior across approved deployment profiles rather than embedding hard-coded mission values in flight code.

1.2 WHEN the Configuration_Manager loads an Engineering_Configuration, THE Configuration_Manager SHALL validate the schema and relationships, produce a deterministic Resolved_Configuration, calculate its Canonical_Hash, and publish a read-only configuration status containing the revision, hash, and field-level violations.

1.3 THE Full_Self_Driving_Product SHALL be delivered as one standalone ROS_2 package named `full_self_driving` with one Integrated_Launch_Entry_Point named `full_self_driving.launch.py`.

1.4 IF a requested production launch depends on a Prototype_Reference, an old prototype launch file, an old prototype script, or `gazebo_models`, THEN THE Lifecycle_Supervisor SHALL reject startup with a dependency-boundary error before readiness is reported.

1.5 WHEN an operator invokes `ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:=kmitl_airfield headless:=false`, THE Simulation_Profile SHALL start the KMITL_Simulation_Fixture, manifest-configured PX4 SITL transport, required MicroXRCE-DDS agent, configured clock/camera/TF bridges, all required production nodes, a readiness summary, and reverse dependency shutdown supervision.

1.6 IF `simulation:=false` is selected without a complete approved Hardware_Manifest, THEN THE Integrated_Launch_Entry_Point SHALL fail with `HARDWARE_PROFILE_NOT_CONFIGURED` and SHALL not start simulated or fake PX4, camera, payload, telemetry, or Pi 4 components.

1.7 WHILE a Hardware_Profile lacks approval and validation evidence for Raspberry Pi 4 bringup, THE Full_Self_Driving_Product SHALL report hardware bringup as deferred and SHALL not report hardware readiness.

1.8 WHERE an approved Simulation_Profile or Hardware_Profile is selected, THE Full_Self_Driving_Product SHALL preserve the same domain safety rules, ROS_Interface_Boundary, persistence protocol, and Node-RED preparation contract while changing only declared integration adapters, process manifests, and resource paths.

1.9 WHEN a mission context is committed and later locked for an armed sortie, THE Mission_Snapshot SHALL contain the complete Resolved_Configuration, Canonical_Hash, Operator_Selection, plan and checkpoint state, Selected_Target, payload state, and executor checkpoint, and THE Locked_Snapshot SHALL reject mutation for the active sortie.

1.10 IF Node-RED/MQTT requests an Engineering_Configuration or Resolved_Configuration mutation, THEN THE Configuration_Manager SHALL reject the request and SHALL preserve the administrator-owned configuration and Canonical_Hash.

### Requirement 2: Disarmed preparation, managed plans, and working-plan state

**User Story:** As an operator, I want to prepare a safe mission context using managed plans and revisioned selections, so that the flight runtime receives an explicit, reproducible, and reviewable sortie context.

#### Acceptance Criteria

2.1 WHILE the vehicle is Disarmed and no Locked_Snapshot exists, THE Preparation_Gateway SHALL allow Node-RED/MQTT to mutate only Operator_Selection and approved pre-arm payload-preparation state.

2.2 WHEN the Plan_Manager receives plan bytes through the typed upload contract, THE Plan_Manager SHALL enforce the configured size and JSON bounds, reject unsafe names or paths, validate the approved QGroundControl `.plan` subset, calculate the Plan_Artifact Canonical_Hash, and store the accepted artifact under a managed immutable ID.

2.3 WHEN a disarmed operator resets or resumes a Working_Plan, THE Plan_Manager SHALL create a new generation with empty checkpoint state and `0%` progress for a reset, preserve the source Plan_Artifact hash, and begin a valid resume at the checkpoint position or next source index rather than silently restarting from the first source waypoint.

2.4 WHEN the Plan_Manager parses a valid Plan_Artifact, THE Plan_Manager SHALL walk supported nested mission items, preserve source indexes, reject unsupported safety-relevant constructs with a stable error, and generate a canonical Search route with a route hash.

2.5 THE Plan_Manager SHALL provide a bounded canonical printer for every accepted plan representation so that the printed representation is valid input for the approved plan parser.

2.6 WHEN a valid accepted plan representation is printed and parsed again, THE Plan_Manager SHALL produce an equivalent supported navigation structure, source-index mapping, and canonical route hash.

2.7 WHEN the Plan_Manager records a Search waypoint or safe interruption checkpoint, THE Plan_Manager SHALL persist the Working_Plan generation, source hash, next source index, position when valid, completed count, total count, progress, reason, and durable checkpoint sequence.

2.8 WHEN the Preparation_Gateway receives an allowed mutation, THE Preparation_Gateway SHALL require a bounded typed payload, a request Idempotency_Key, a current Revision_Guard, an allowed command name, a valid request age, and a non-retained MQTT command.

2.9 IF a gateway request names `arm`, `disarm`, `select_ownmode`, `takeoff`, `land`, `rtl`, `goto`, `setpoint`, `raw_setpoint`, `raw_gpio`, `raw_servo`, `release`, `release_cargo`, an arbitrary ROS operation, or an arbitrary filesystem path, THEN THE Preparation_Gateway SHALL reject the request with a stable error and SHALL produce no control side effect.

2.10 IF a preparation mutation is received while the vehicle is Armed, a Locked_Snapshot is active, recovery is unresolved, or the expected revision is stale, THEN THE Preparation_Gateway SHALL reject the mutation and SHALL preserve the authoritative state.

2.11 IF an upload has the same managed artifact identity as an existing immutable Plan_Artifact but a different Canonical_Hash, THEN THE Plan_Manager SHALL reject the upload without replacing the existing artifact.

### Requirement 3: Scoped registry, perception, and selected-target live lock

**User Story:** As a flight operator, I want known marker records and live target observations kept separate and scoped to the active mission context, so that map knowledge assists navigation without authorizing an unsafe landing or payload action.

#### Acceptance Criteria

3.1 THE Pad_Registry SHALL scope every durable record by Map_Scenario_Scope, target namespace, marker dictionary, and marker ID, and SHALL reject a lookup from returning a record outside the requested scope.

3.2 WHEN the Perception_Service publishes All_ID_Observations, THE Perception_Service SHALL keep all-ID observations separate from the Selected_Target Live_Target_Lock stream and SHALL not treat registry existence as a qualified live lock.

3.3 THE Direct strategy SHALL use a trusted Pad_Registry coordinate only for navigation assistance and SHALL not create a Live_Target_Lock, authorize PrecisionLand descent, verify the landing target, or authorize a payload operation.

3.4 WHEN the Pad_Registry accepts an All_ID_Observation, THE Pad_Registry SHALL apply configured timestamp, transform, calibration, quality, covariance, outlier, and scope checks before durably updating the corresponding scoped record.

3.5 IF a Pad_Registry lookup requests a different map, scenario, namespace, dictionary, or marker ID from a stored record, THEN THE Pad_Registry SHALL exclude the stored record from the trusted lookup result.

3.6 WHEN a disarmed operator requests `clear_pad_registry`, THE Pad_Registry SHALL require the active map and scenario, the current registry revision, the configured confirmation, and a successful durable backup before committing the cleared revision.

3.7 WHEN an observation matches the Selected_Target and passes configured identity, scope, frame, freshness, quality, covariance, consecutive-observation, transform, and spatial-consistency gates, THE Perception_Service SHALL publish a qualified Live_Target_Lock containing the selected identity and evidence timestamps.

3.8 IF an All_ID_Observation fails the Selected_Target identity, Map_Scenario_Scope, freshness, quality, covariance, transform, or calibration gate, THEN THE Mission_Coordinator SHALL reject the observation as a selected-target lock while preserving permitted all-ID diagnostic or registry behavior.

3.9 IF a Live_Target_Lock becomes stale or lost during PrecisionLand, THEN THE Mission_Coordinator SHALL stop or reverse descent according to the configured target-loss policy and SHALL record the lock-loss transition.

3.10 WHILE the vehicle is Armed or a Locked_Snapshot exists, THE Preparation_Gateway SHALL reject a target-identity mutation and SHALL preserve the target identity in the Locked_Snapshot.

### Requirement 4: Authority gates and preparation-plane boundary

**User Story:** As a safety operator, I want PX4 and QGroundControl to remain authoritative for mode selection and arming while the preparation plane is strictly limited, so that stale or compromised operator interfaces cannot control flight.

#### Acceptance Criteria

4.1 WHEN PX4 and QGroundControl request activation or arming of the Registered_Mode, THE Flight_Runtime SHALL report readiness only when the committed context, map/scenario, Plan_Artifact, valid Working_Plan, Selected_Target, payload readiness, persistence health, recovery state, configuration hash, PX4 transport, and policy-required health gates all pass, SHALL report every failed gate with a stable reason, and SHALL treat PX4/QGroundControl state as authoritative for Registered_Mode selection and arming.

4.2 THE Preparation_Gateway SHALL never arm, disarm, select the Registered_Mode, take off, land, command RTL, publish flight setpoints, invoke raw PX4 flight control, issue raw actuator commands, or trigger an in-flight Named_Payload_Operation on behalf of Node-RED/MQTT, and SHALL reject any security-invalid gateway request without a PX4, flight, payload, filesystem, or state-mutation side effect.

4.3 IF a retained MQTT command, stale status projection, cached dashboard value, disconnected gateway response, or replayed request is used as an authorization source, THEN THE Preparation_Gateway SHALL reject the authorization and SHALL consult the authoritative runtime state instead.

4.4 WHILE the vehicle is Armed or a Locked_Snapshot exists, THE Preparation_Gateway SHALL permit only read-only inspection operations and SHALL reject all preparation mutations.

4.5 WHEN a readiness gate fails, THE Flight_Runtime SHALL keep the current authoritative state unchanged or enter an explicit safety state and SHALL not make the failed gate pass from a Node-RED status or parameter update.

### Requirement 5: Registered PX4 mode, sortie execution, and payload safety

**User Story:** As a flight-system owner, I want one version-matched PX4 external mode to execute the complete configured sortie under explicit guards, so that navigation, perception, landing, payload, and return behavior remain ordered and safety-controlled.

#### Acceptance Criteria

5.1 THE FullSelfDrivingModeExecutor SHALL execute a committed sortie only in the configured order: takeoff, TransitIn, target acquisition, Direct or Search, live-lock-gated PrecisionLand, landed-target verification, payload operation, second takeoff, TransitOut or configured outbound route, ReturnStrategy, and recovery landing.

5.2 WHEN required Lifecycle_Nodes are active, Version_Matched_PX4_Library transport is compatible, durable recovery is clear, storage is healthy, and policy-required health signals are fresh, THE Flight_Runtime SHALL register exactly one FullSelfDrivingMode and exactly one owning FullSelfDrivingModeExecutor derived from the version-matched ModeExecutorBase API, and THE Mission_Coordinator SHALL own strategy decisions applied through that one mode/executor path without a perception-owned transition or competing scheduler.

5.3 WHILE PrecisionLand is active, THE FullSelfDrivingMode SHALL execute Search, Approach, Descend, and Landed_Verify using only a fresh qualified Live_Target_Lock and configured altitude, velocity, quality, covariance, target-loss, stability, dwell, and landing-verification gates, and SHALL stop or reverse descent when live-lock freshness fails.

5.4 WHEN a payload action is requested after landing verification, THE Payload_Controller SHALL issue a Named_Payload_Operation only after landing, stability, target identity/live-lock, policy, operation-count, idempotency, adapter-health, and feedback gates pass, SHALL persist the result, and SHALL not automatically retry an Unknown_Payload_Result.

5.5 WHEN TransitOut completes or a safe return is required, THE FullSelfDrivingModeExecutor SHALL select and verify the snapshot's explicit ReturnStrategy, including configured route or approved PX4/ModeExecutor return behavior, before finalizing recovery landing.

5.6 THE Full_Self_Driving_Product SHALL use no Offboard_Control, direct `/fmu/in/offboard_control_mode` publication, direct `/fmu/in/trajectory_setpoint` publication for flight control, generic setpoint service, Raw_Control_Bridge, alternate flight-control library, or equivalent fallback under any launch profile, test fixture, or recovery path.

5.7 THE FullSelfDrivingMode SHALL implement TransitIn, TransitOut, Search, Direct, and PrecisionLand as Internal_Strategies owned by the one registered mode, and THE FullSelfDrivingModeExecutor SHALL remain the only companion flight scheduler unless a later pinned-release design explicitly replaces this arrangement.

5.8 WHEN the TransitIn or TransitOut strategy follows a configured route, THE FullSelfDrivingMode SHALL apply the snapshot's altitude, speed, acceleration, heading, arrival, settle, geofence, clearance, energy, and checkpoint policies before advancing to the next route boundary.

5.9 WHEN target acquisition begins, THE Mission_Coordinator SHALL select Direct only when the trusted Pad_Registry record, scope, age, quality, path, clearance, and energy gates pass, and SHALL select Search when Direct is unavailable and a valid Working_Plan exists.

5.10 WHEN Direct reaches its configured safe navigation position, THE FullSelfDrivingMode SHALL enter PrecisionLand search behavior and SHALL require a fresh qualified Live_Target_Lock before Approach or Descend.

5.11 WHEN a Named_Payload_Operation succeeds, THE FullSelfDrivingModeExecutor SHALL persist the result before starting the second takeoff and SHALL select TransitOut or the configured outbound strategy only after the second-takeoff gates pass.

5.12 IF PX4, QGroundControl, RC, a PX4 failsafe, mode loss, watchdog expiry, or transport loss removes companion Flight_Authority, THEN THE FullSelfDrivingModeExecutor SHALL stop library-managed mode work, persist the safest available checkpoint, yield to PX4, and SHALL not fight or mask the takeover.

### Requirement 6: Lifecycle ownership, durability, and recovery

**User Story:** As a system operator, I want safety-relevant state, lifecycle transitions, and recovery decisions to be durable and explicit, so that interruption or restart cannot cause an automatic unsafe action.

#### Acceptance Criteria

6.1 WHEN the Persistence_Manager reports any safety-relevant state, including a Mission_Snapshot, as durable, THE Persistence_Manager SHALL complete the configured Durable_Boundary for that state, including validation, temporary write, flush/fsync or equivalent, atomic replacement, directory durability where supported, journal sequence, and backup policy, before publishing the state as committed or durable.

6.2 WHEN the product restarts with an ambiguous snapshot, journal, Working_Plan, payload outcome, executor action, registry record, configuration hash, or evidence sequence, THE Persistence_Manager SHALL enter RECOVERY_REQUIRED, publish the ambiguity codes and last valid durable sequence, and disable automatic arm, resume, strategy switching, and payload operation.

6.3 IF a snapshot, journal, registry, Working_Plan, payload record, executor checkpoint, or evidence write fails, THEN THE Persistence_Manager SHALL preserve the last valid durable state, report the failed boundary and durability state, and block any progression that requires the failed state.

6.4 WHEN the Regular_Runtime_Node starts, THE Lifecycle_Supervisor SHALL start it without registering FullSelfDrivingMode, configure required Lifecycle_Nodes, and activate registry, perception, evidence, and gateway components in the approved dependency order before permitting mode registration.

6.5 WHEN a required lifecycle configure or activate transition fails, THE Lifecycle_Supervisor SHALL withdraw readiness, deactivate already-active components in reverse order, preserve durable state, shut down child processes in reverse dependency order, and SHALL not substitute an unapproved simulator or control path.

6.6 WHEN the Regular_Runtime_Node deactivates or shuts down, THE Flight_Runtime SHALL stop library-managed updates, persist the safe checkpoint and deactivation state, yield to PX4, flush required persistence and evidence, and SHALL not destroy an active mode object while PX4 may still hold mode authority.

6.7 WHEN a route waypoint, strategy boundary, target-lock transition, payload intent/result, safe deactivation, or recovery decision completes, THE Persistence_Manager SHALL record the configured checkpoint or event before the runtime reports the boundary as durable.

6.8 WHILE RECOVERY_REQUIRED is active, THE Preparation_Gateway SHALL allow only explicit disarmed recovery inspection and resolution and SHALL not permit an ambiguous payload operation to resume without a new explicitly authorized sortie context.

6.9 WHEN an operator resolves RECOVERY_REQUIRED while Disarmed, THE Persistence_Manager SHALL durably record the decision, require fresh validation and preflight, and SHALL not resume a previously ambiguous payload operation without a new explicitly authorized sortie context.

### Requirement 7: Bounded interfaces, truthful observability, evidence, safety authority, and security

**User Story:** As an operator, developer, and safety reviewer, I want bounded interfaces, truthful status, correlated evidence, and implementation-level security, so that the product can be integrated, monitored, audited, and rejected safely under malformed or unauthorized input.

#### Acceptance Criteria

7.1 THE ROS_Interface_Boundary and Observability_Plane SHALL expose complete, truthful, read-only status snapshots and typed ROS 2 `.msg`, `.srv`, and `.action` contracts with bounded strings/sequences, explicit optional-field flags, validated enums, finite numeric values, owner fields, revision fields, and no raw PX4, Offboard, setpoint, filesystem, GPIO, servo, executable, or arbitrary JSON control fields, and SHALL deliver observability data asynchronously without blocking the registered-mode update path.

7.2 WHEN a safety-relevant event, route checkpoint, target lock, landing verification, payload result, recovery decision, or sortie completion is recorded, THE Observability_Plane SHALL correlate the record with mission ID, sortie ID, snapshot hash, event sequence, durable sequence, component, source, and idempotency key as applicable.

7.3 WHEN PX4 safety, QGroundControl, RC, or a PX4 failsafe changes Flight_Authority, THE Full_Self_Driving_Product SHALL expose the higher-priority state in the Status_Read_Model and SHALL not allow a lower-priority gateway, UI, cached status, or companion decision to suppress or reinterpret the state.

7.4 THE Status_Read_Model SHALL distinguish PX4/telemetry ground-link health from optional QGroundControl application presence and SHALL expose configuration state, flight phase, ownmode state, armed/landed state, health, plan progress, target registry state, live-lock state, payload feedback, persistence state, recovery state, failures, and safe operator actions.

7.5 WHILE the product is operating, THE Observability_Plane SHALL use bounded asynchronous logging, diagnostics, metrics, and optional tracing so that exporter, evidence, or dashboard failures cannot block the registered-mode update path or create a flight-control path.

7.6 THE Security_Plane SHALL enforce authenticated encrypted MQTT/TLS access, broker ACLs, configured DDS-Security/SROS2 permissions where enabled, protected secret storage, certificate validation and rotation, bounded request age/size/rate, input and manifest sanitization, resource limits, and evidence-integrity checks.

7.7 IF a request, participant, credential, certificate, manifest, plan, configuration, path, enum, numeric value, sequence, resource limit, or authorization fails a Security_Plane check, THEN THE owning component SHALL reject the input, record a bounded audit result, preserve authoritative state, and produce no PX4, flight, payload, filesystem, or alternate-control side effect.

7.8 THE Full_Self_Driving_Product SHALL return stable bounded error codes containing the owning component, severity, current configuration state, current flight phase, relevant revision or durable sequence, operator-safe explanation, and safe next action for every rejected or failed operation.

7.9 THE ROS_Interface_Boundary SHALL provide typed contracts for managed plan upload and selection, map/scenario selection, Working_Plan creation and reset, target selection, named payload preparation, registry inspection and clear, mission validation and commit, recovery resolution, and committed-sortie supervision.

## Design Correctness-Property Traceability

The following mapping preserves the requirement references already attached to the correctness properties in `design.md`. Each property is intended to validate the cited acceptance criterion or criteria; additional acceptance criteria in this document are covered by the unit, property, integration, smoke, lifecycle, launch, security, and soak strategies described by the design.

| Design property | Property focus | Requirement reference |
|---|---|---|
| Property 1 | Authoritative engineering configuration | **Validates: Requirements 1.1** |
| Property 2 | Configuration hash consistency | **Validates: Requirements 1.2** |
| Property 3 | Disarmed operator-selection isolation | **Validates: Requirements 2.1** |
| Property 4 | Plan immutability and safe paths | **Validates: Requirements 2.2** |
| Property 5 | Working-plan generation correctness | **Validates: Requirements 2.3** |
| Property 6 | Map/scenario registry isolation | **Validates: Requirements 3.1** |
| Property 7 | All-ID/live-lock separation | **Validates: Requirements 3.2** |
| Property 8 | Direct never substitutes for visual lock | **Validates: Requirements 3.3** |
| Property 9 | Ownmode readiness is complete and authoritative | **Validates: Requirements 4.1** |
| Property 10 | Gateway command boundary | **Validates: Requirements 4.2** |
| Property 11 | Mission sequence ordering | **Validates: Requirements 5.1** |
| Property 12 | Coordinator-owned mode transitions | **Validates: Requirements 5.2** |
| Property 13 | Precision landing freshness | **Validates: Requirements 5.3** |
| Property 14 | Payload operation safety | **Validates: Requirements 5.4** |
| Property 15 | Return-strategy explicitness | **Validates: Requirements 5.5** |
| Property 16 | Durable boundary integrity | **Validates: Requirements 6.1** |
| Property 17 | Recovery safety | **Validates: Requirements 6.2** |
| Property 18 | Status observability truthfulness | **Validates: Requirements 7.1** |
| Property 19 | Evidence correlation | **Validates: Requirements 7.2** |
| Property 20 | Stronger safety authority wins | **Validates: Requirements 7.3** |
| Property 21 | Concrete ROS interface boundary | **Validates: Requirements 7.1** |
| Property 22 | Lifecycle activation precedes external-mode registration | **Validates: Requirements 5.2** |
| Property 23 | Snapshot commit and recovery ordering | **Validates: Requirements 6.1** |
| Property 24 | Simulation/hardware adapter invariance | **Validates: Requirements 1.1** |
| Property 25 | Observability noninterference and truthfulness | **Validates: Requirements 7.1** |
| Property 26 | Security rejection has no flight side effect | **Validates: Requirements 4.2** |

## Assumptions and Open Implementation Gates

- The exact ROS 2 distribution, PX4 release, `px4_msgs` release, and Auterion `px4_ros2_cpp`/`px4_ros2_interface_lib` release remain to be pinned and compatibility-tested.
- Exact `ModeBase`, `ModeExecutorBase`, activation, watchdog, action, deactivation, and return-strategy APIs are implementation gates. The requirements mandate one owning executor and one registered mode without assuming undocumented signatures.
- The approved QGroundControl `.plan` grammar subset, canonical printer representation, duplicate-key behavior, supported nested items, and safety-relevant unknown-item policy remain to be frozen.
- Engineering-owned catalogs must define map/scenario IDs, vehicle and payload adapters, target dictionaries/namespaces, calibration artifacts, launch manifests, world/resource versions, and hardware-manifest approval evidence.
- Storage-platform behavior for fsync, directory durability, backup retention, protected secret storage, and power-loss testing must be selected before implementation acceptance.
- The exact ROS interface schema version, maximum bounds, enum registries, QoS profiles, DDS-Security governance/permissions, MQTT certificate lifecycle, and evidence-retention policy remain implementation gates.
- The exact KMITL world/resource/package versions and process manifests must be declared by the approved Simulation_Profile. `simulation:=false` remains explicitly deferred until a complete Hardware_Manifest is approved.
- Property-based tests apply to pure domain transformations and bounded in-memory logic; infrastructure, lifecycle, launch, external services, and hardware behavior require example, integration, smoke, or hardware-in-the-loop tests as appropriate.
