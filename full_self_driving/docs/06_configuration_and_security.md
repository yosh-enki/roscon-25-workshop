# Module 06: Configuration & Security Hardening

The `full_self_driving` software suite is hardened with **authoritative parameter schemas**, **SROS2 cryptographic enclaves**, and **negative security boundary gateways** to ensure robust, tamper-resistant operations.

---

## 1. Authoritative Parameter Configuration (`fsd_parameters.yaml`)

All configurable system values are consolidated into a single authoritative parameter file located at [`config/fsd_parameters.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/fsd_parameters.yaml).

### 1.1 Complete Parameter Catalog

#### Perception Parameters (`perception`)
| Parameter Name | Data Type | Default | Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| `dictionary` | `string` | `"DICT_4X4_50"` | - | Active ArUco marker dictionary name |
| `marker_size_m` | `float` | `0.40` | meters | Physical marker edge length |
| `camera_frame` | `string` | `"camera_frame"` | - | Optical frame ID for TF transforms |
| `min_quality` | `float` | `0.10` | $[0.0, 1.0]$ | Minimum detection quality threshold |
| `lock_min_consecutive_obs` | `int` | `1` | count | Consecutive frames required for qualification |
| `lock_spatial_consistency_m` | `float` | `25.0` | meters | Maximum allowed jump between observations |
| `lock_freshness_timeout_s` | `float` | `2.0` | seconds | Target timeout before transitioning to LOST |

#### Flight Route & Strategy Parameters (`routes`)
| Parameter Name | Data Type | Default | Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| `transit_in_speed_m_s` | `float` | `5.0` | $m/s$ | Ingress horizontal cruise speed |
| `transit_out_speed_m_s` | `float` | `5.0` | $m/s$ | Egress horizontal cruise speed |
| `transit_altitude_m` | `float` | `20.0` | meters AGL | Ingress and egress transit corridor altitude |
| `search_altitude_m` | `float` | `12.0` | meters AGL | Nominal search pattern altitude |
| `approach_altitude_m` | `float` | `5.0` | meters AGL | Intermediate precision land altitude |
| `max_horizontal_velocity_m_s`| `float` | `5.0` | $m/s$ | Global horizontal velocity limit |
| `landing_descent_rate_m_s` | `float` | `0.5` | $m/s$ | Final visual descent speed |
| `acceptance_radius_m` | `float` | `4.0` | meters | Waypoint arrival sphere radius |
| `max_yaw_rate_deg_s` | `float` | `45.0` | $\text{deg}/s$ | Maximum angular yaw rate limit |
| `search_policy` | `string` | `"complete_grid_first"` | - | Search completion policy (`complete_grid_first` / `interrupt_on_target`) |

#### Safety & Power Limits (`safety`)
| Parameter Name | Data Type | Default | Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| `max_altitude_m` | `float` | `30.0` | meters AGL | Hard geofence altitude ceiling |
| `min_battery_percentage` | `float` | `20.0` | $\%$ | Battery floor required for Direct flight |
| `target_loss_timeout_s` | `float` | `3.0` | seconds | Precision land timeout before abort/hold |
| `emergency_stop_enabled` | `bool` | `true` | - | Master emergency stop flag |

#### Payload & Actuator Parameters (`payload`)
| Parameter Name | Data Type | Default | Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| `adapter_type` | `string` | `"simulation_payload_stub"` | - | Active HAL adapter ID |
| `gripper_instance` | `int` | `1` | - | PX4 actuator instance index (1–8) |
| `actuation_dwell_s` | `float` | `0.5` | seconds | Post-actuation dwell time |

---

## 2. JSON Schema Validation (`config/schemas/`)

Runtime configuration and external command envelopes are validated against JSON Schemas:
- [`config/schemas/engineering_config.schema.json`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/schemas/engineering_config.schema.json): Validates overall system configuration on startup.
- [`config/schemas/command_envelope.schema.json`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/schemas/command_envelope.schema.json): Validates all external REST/WebSocket commands ingested by `fsd_gateway`.
- [`config/schemas/snapshot.schema.json`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/schemas/snapshot.schema.json): Schema for persistent mission snapshots.
- [`config/schemas/journal.schema.json`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/schemas/journal.schema.json): Schema for append-only mission journals.

---

## 3. SROS2 DDS Cryptographic Security (PKI Enclaves)

To secure multi-machine robotics deployments (e.g. companion computer to ground station over Wi-Fi/LTE), the package supports **SROS2 (Secure ROS 2)** with DDS encryption and access control.

```mermaid
graph TD
    CA[Root Identity & Permissions CA] --> KEYSTORE[sample_keystore/enclaves]
    
    KEYSTORE --> E1[/full_self_driving/flight_runtime]
    KEYSTORE --> E2[/full_self_driving/perception]
    KEYSTORE --> E3[/full_self_driving/pad_registry]
    KEYSTORE --> E4[/full_self_driving/gateway]
    KEYSTORE --> E5[/full_self_driving/evidence]
```

### 3.1 Keystore Structure
- Tooling: [`scripts/generate_sros2_keystore.py`](file:///home/yosh/roscon-25-workshop/full_self_driving/scripts/generate_sros2_keystore.py) and [`scripts/manage_sros2_keystore.sh`](file:///home/yosh/roscon-25-workshop/full_self_driving/scripts/manage_sros2_keystore.sh).
- Security Artifacts per Enclave:
  - `identity_ca.cert.pem`: Identity Certificate Authority.
  - `cert.pem` & `key.pem`: Node X.509 certificate and private RSA/ECDSA key.
  - `governance.p7s` & `governance.xml`: Signed DDS domain security rules.
  - `permissions.p7s` & `permissions.xml`: Signed publish/subscribe topic whitelist.

### 3.2 DDS Profiles Supported
- **CycloneDDS Security**: [`config/security/cyclonedds_security.xml`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/security/cyclonedds_security.xml)
- **FastDDS Security**: [`config/security/fastdds_security.xml`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/security/fastdds_security.xml)

---

## 4. Gateway Security & Negative Boundaries

The [`FsdGateway`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/gateway/fsd_gateway.hpp) acts as the single boundary gatekeeper between untrusted external clients (Web UIs, Node-RED, external APIs) and the internal flight stack.

### 4.1 Negative Security Boundary Enforcement
To protect physical flight integrity, the Gateway actively rejects any command attempting to bypass the FSM:
- **Forbidden Commands**: Direct motor RPM setting, direct trajectory overriding, raw actuator pulsing, or forced disarm in flight.
- **Allowed Commands**: High-level declarative actions only (`select_map_scenario`, `select_target_identity`, `prepare_payload`, `select_plan_artifact`, `emergency_stop`).

### 4.2 Denial-of-Service (DoS) Protection
1. **Rate Limiting**: Clamped at 120 commands per minute per client.
2. **Payload Size Limit**: Hard ceiling of 8 MiB (8,388,608 bytes) to prevent memory exhaustion attacks during plan uploads.
3. **Idempotency Caching**: Re-transmitted requests with duplicate `request_id` values return cached results without re-executing state transitions.
