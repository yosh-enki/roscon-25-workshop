# PX4 Native Gripper Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate PX4-native gripper peripheral control via `px4_msgs::msg::VehicleCommand` (`VEHICLE_CMD_DO_GRIPPER` / Command 211) and `PayloadAdapter` HAL interface, enabling direct FMU actuator management with full backward compatibility and safety gate enforcement.

**Architecture:** Implement a dedicated `Px4GripperPayloadAdapter` adhering to `full_self_driving::payload::PayloadAdapter`. Bridge commanded states to `px4_msgs::msg::VehicleCommand` published to `/fmu/in/vehicle_command`, process execution telemetry from `/fmu/out/vehicle_command_ack`, and validate manifest declarations via `HardwareManifestValidator`.

**Tech Stack:** ROS 2 (Humble/Jazzy), C++17, `px4_msgs`, `rclcpp`, `ament_cmake`, GoogleTest, Gmock.

## Global Constraints

- Fail-closed error handling: on unknown outcome (`RESULT_UNKNOWN`) or command ACK failure, transitions must abort to recovery landing (`RETURN_STRATEGY`) and must never re-trigger payload actuation (Design Property 14).
- Zero mock or synthetic fallback code in production paths; deferred physical bringup must report explicit health status.
- Maintain full ABI and API compatibility with existing `PreparePayload.srv` and `PayloadStatus.msg` interfaces.
- Comply with SROS2 access control policies for `/fmu/in/vehicle_command` and `/fmu/out/vehicle_command_ack`.
- Follow strict TDD: Unit test first -> Minimal implementation -> Test verification -> Commit.

---

### Task 1: PX4 Gripper Payload Adapter Header & Unit Tests

**Files:**
- Create: `full_self_driving/src/payload/px4_gripper_payload_adapter.hpp`
- Create: `full_self_driving/test/payload/px4_gripper_payload_adapter_test.cpp`

**Interfaces:**
- Consumes: `full_self_driving::payload::PayloadAdapter`, `full_self_driving::msg::PayloadStatus`, `px4_msgs::msg::VehicleCommand`, `px4_msgs::msg::VehicleCommandAck`
- Produces: `full_self_driving::payload::Px4GripperPayloadAdapter`

- [ ] **Step 1: Create the Px4GripperPayloadAdapter class definition**

```cpp
// full_self_driving/src/payload/px4_gripper_payload_adapter.hpp
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include "payload/payload_adapter.hpp"
#include "full_self_driving/msg/payload_status.hpp"

namespace full_self_driving::payload
{

class Px4GripperPayloadAdapter : public PayloadAdapter
{
public:
  struct Config
  {
    std::string adapter_id{"px4_uorb_gripper_actuator"};
    uint8_t gripper_instance{1};
    uint64_t command_timeout_ms{1500};
    uint8_t target_system{1};
    uint8_t target_component{1};
  };

  explicit Px4GripperPayloadAdapter(
    rclcpp::Node & node,
    Config config = Config{});

  ~Px4GripperPayloadAdapter() override = default;

  std::string get_adapter_id() const override { return config_.adapter_id; }
  bool is_healthy() const override;

  bool execute_command(
    uint8_t commanded_state,
    const std::string & operation_id,
    full_self_driving::msg::PayloadStatus & out_status) override;

  full_self_driving::msg::PayloadStatus get_status() const override;

  void handle_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr ack);

private:
  rclcpp::Node & node_;
  Config config_;
  mutable std::mutex mutex_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

  bool healthy_{true};
  full_self_driving::msg::PayloadStatus current_status_;
  uint64_t last_command_timestamp_us_{0};
  uint8_t expected_command_id_{px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER};
  bool pending_ack_{false};
};

}  // namespace full_self_driving::payload
```

- [ ] **Step 2: Write unit test for Px4GripperPayloadAdapter**

```cpp
// full_self_driving/test/payload/px4_gripper_payload_adapter_test.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "payload/px4_gripper_payload_adapter.hpp"

class Px4GripperPayloadAdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("px4_gripper_adapter_test_node");
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(Px4GripperPayloadAdapterTest, PublishesVehicleCommandOnRelease)
{
  full_self_driving::payload::Px4GripperPayloadAdapter::Config cfg;
  cfg.adapter_id = "px4_uorb_gripper_actuator";
  cfg.gripper_instance = 1;

  auto adapter = std::make_shared<full_self_driving::payload::Px4GripperPayloadAdapter>(*node_, cfg);
  EXPECT_EQ(adapter->get_adapter_id(), "px4_uorb_gripper_actuator");
  EXPECT_TRUE(adapter->is_healthy());

  // Subscribe to vehicle command
  bool command_received = false;
  uint16_t received_cmd = 0;
  float received_param2 = -1.0f;

  auto sub = node_->create_subscription<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10,
    [&](const px4_msgs::msg::VehicleCommand::SharedPtr msg) {
      command_received = true;
      received_cmd = msg->command;
      received_param2 = msg->param2;
    });

  full_self_driving::msg::PayloadStatus status;
  bool ok = adapter->execute_command(
    full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED,
    "op_test_001",
    status);

  EXPECT_TRUE(ok);
  EXPECT_EQ(status.last_operation_id, "op_test_001");
  EXPECT_EQ(status.commanded_state, full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED);

  // Spin to receive
  rclcpp::spin_some(node_);
  EXPECT_TRUE(command_received);
  EXPECT_EQ(received_cmd, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER);
  EXPECT_FLOAT_EQ(received_param2, 0.0f); // 0.0 = Release
}

TEST_F(Px4GripperPayloadAdapterTest, ProcessesAckFeedback)
{
  full_self_driving::payload::Px4GripperPayloadAdapter adapter(*node_);

  full_self_driving::msg::PayloadStatus status;
  adapter.execute_command(
    full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED,
    "op_test_002",
    status);

  // Simulate ACK from PX4
  auto ack = std::make_shared<px4_msgs::msg::VehicleCommandAck>();
  ack->command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER;
  ack->result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  ack->timestamp = node_->get_clock()->now().nanoseconds() / 1000;

  adapter.handle_command_ack(ack);

  auto updated = adapter.get_status();
  EXPECT_EQ(updated.last_operation_result, full_self_driving::msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_EQ(updated.feedback_state, full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED);
  EXPECT_FALSE(updated.cargo_loaded);
  EXPECT_FALSE(updated.secured);
}
```

- [ ] **Step 3: Run unit tests to verify failure before implementation**

Run: `colcon test --packages-select full_self_driving --ctest-args -R px4_gripper_payload_adapter_test`
Expected: FAIL / build missing `px4_gripper_payload_adapter.cpp`

---

### Task 2: Px4GripperPayloadAdapter Implementation & Build Integration

**Files:**
- Create: `full_self_driving/src/payload/px4_gripper_payload_adapter.cpp`
- Modify: `full_self_driving/CMakeLists.txt:183-205,1075-1115`

**Interfaces:**
- Consumes: `full_self_driving/src/payload/px4_gripper_payload_adapter.hpp`
- Produces: `fsd_payload_core` library target containing `Px4GripperPayloadAdapter`

- [ ] **Step 1: Implement Px4GripperPayloadAdapter source**

```cpp
// full_self_driving/src/payload/px4_gripper_payload_adapter.cpp
#include "payload/px4_gripper_payload_adapter.hpp"

namespace full_self_driving::payload
{

Px4GripperPayloadAdapter::Px4GripperPayloadAdapter(
  rclcpp::Node & node,
  Config config)
: node_(node), config_(std::move(config))
{
  vehicle_command_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", rclcpp::SystemDefaultsQoS());

  vehicle_command_ack_sub_ = node_.create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", rclcpp::SystemDefaultsQoS(),
    [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr ack) {
      handle_command_ack(ack);
    });

  current_status_.header.frame_id = "px4_gripper";
  current_status_.adapter_id = config_.adapter_id;
  current_status_.commanded_state = full_self_driving::msg::PayloadStatus::COMMAND_SECURED;
  current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED;
  current_status_.cargo_loaded = true;
  current_status_.secured = true;
  current_status_.successful_operation_count = 0;
  current_status_.has_last_operation_id = false;
  current_status_.last_operation_id = "";
  current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_NONE;
  current_status_.unknown_result = false;
  current_status_.feedback_latency_us = 0;
  current_status_.updated_monotonic_ns = 0;
}

bool Px4GripperPayloadAdapter::is_healthy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return healthy_;
}

bool Px4GripperPayloadAdapter::execute_command(
  uint8_t commanded_state,
  const std::string & operation_id,
  full_self_driving::msg::PayloadStatus & out_status)
{
  std::lock_guard<std::mutex> lock(mutex_);

  current_status_.has_last_operation_id = !operation_id.empty();
  current_status_.last_operation_id = operation_id;
  current_status_.commanded_state = commanded_state;

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  last_command_timestamp_us_ = cmd.timestamp;

  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER;
  cmd.param1 = static_cast<float>(config_.gripper_instance);

  switch (commanded_state) {
    case full_self_driving::msg::PayloadStatus::COMMAND_OPEN:
      cmd.param2 = 0.0f; // Release / Open
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN;
      break;
    case full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED:
      cmd.param2 = 0.0f; // Release
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED;
      break;
    case full_self_driving::msg::PayloadStatus::COMMAND_SECURED:
    default:
      cmd.param2 = 1.0f; // Grab / Close
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED;
      break;
  }

  cmd.target_system = config_.target_system;
  cmd.target_component = config_.target_component;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  if (vehicle_command_pub_) {
    vehicle_command_pub_->publish(cmd);
  }

  pending_ack_ = true;
  current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_SUCCESS;
  current_status_.updated_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  out_status = current_status_;
  return true;
}

full_self_driving::msg::PayloadStatus Px4GripperPayloadAdapter::get_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_status_;
}

void Px4GripperPayloadAdapter::handle_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr ack)
{
  if (!ack || ack->command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pending_ack_ = false;

  uint64_t now_us = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  if (now_us >= last_command_timestamp_us_) {
    current_status_.feedback_latency_us = now_us - last_command_timestamp_us_;
  }

  if (ack->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
    current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_SUCCESS;
    if (current_status_.commanded_state == full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED) {
      current_status_.cargo_loaded = false;
      current_status_.secured = false;
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED;
    } else if (current_status_.commanded_state == full_self_driving::msg::PayloadStatus::COMMAND_SECURED) {
      current_status_.cargo_loaded = true;
      current_status_.secured = true;
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED;
    }
    current_status_.successful_operation_count++;
  } else {
    current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
    current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_FAULT;
  }

  current_status_.updated_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace full_self_driving::payload
```

- [ ] **Step 2: Update CMakeLists.txt to register Px4GripperPayloadAdapter and tests**

In `full_self_driving/CMakeLists.txt`:
```cmake
add_library(fsd_payload_core
  src/payload/simulation_payload_adapter.cpp
  src/payload/hardware_payload_adapter.cpp
  src/payload/px4_gripper_payload_adapter.cpp
  src/payload/payload_controller.cpp
)

ament_target_dependencies(fsd_payload_core
  rclcpp
  px4_msgs
)

if(BUILD_TESTING)
  ament_add_gtest(px4_gripper_payload_adapter_test
    test/payload/px4_gripper_payload_adapter_test.cpp
  )
  target_link_libraries(px4_gripper_payload_adapter_test
    fsd_payload_core
  )
  ament_target_dependencies(px4_gripper_payload_adapter_test
    rclcpp
    px4_msgs
  )
endif()
```

- [ ] **Step 3: Run unit test to verify it passes**

Run: `colcon build --packages-select full_self_driving && colcon test --packages-select full_self_driving --ctest-args -R px4_gripper_payload_adapter_test`
Expected: PASS

- [ ] **Step 4: Commit Task 2 changes**

```bash
git add full_self_driving/src/payload/px4_gripper_payload_adapter.* full_self_driving/test/payload/px4_gripper_payload_adapter_test.cpp full_self_driving/CMakeLists.txt
git commit -m "feat(payload): implement Px4GripperPayloadAdapter with VehicleCommand bridge"
```

---

### Task 3: Hardware Manifest Schema & Validator Extension

**Files:**
- Modify: `full_self_driving/simulation/manifests/hardware_schema.yaml:34-44`
- Modify: `full_self_driving/src/launch/hardware_manifest_validator.hpp:74-85`
- Modify: `full_self_driving/src/launch/hardware_manifest_validator.cpp:245-265`
- Test: `full_self_driving/test/launch/launch_manifest_test.cpp`

**Interfaces:**
- Consumes: `HardwareManifestValidator`, `PayloadConfig`
- Produces: Validated `px4_uorb_gripper_actuator` configuration in hardware manifests

- [ ] **Step 1: Update HardwareManifestValidator to accept `px4_uorb_gripper_actuator`**

In `full_self_driving/src/launch/hardware_manifest_validator.hpp`:
```cpp
struct PayloadConfig
{
  std::string adapter_id;
  std::string device_path;
  std::string transport_interface{"vehicle_command"};
  int gripper_instance{1};
  int pwm_pin{18};
  int pwm_frequency_hz{50};
  int disarmed_pwm_us{1000};
  int armed_pwm_us{1500};
  int release_pwm_us{2000};
  int feedback_sense_pin{24};
  int max_pulse_duration_ms{2500};
};
```

In `full_self_driving/src/launch/hardware_manifest_validator.cpp`:
```cpp
if (adapter_id != "gpio_pwm_payload_actuator" &&
    adapter_id != "px4_uorb_gripper_actuator" &&
    adapter_id != "simulation_payload_stub") {
  result.is_valid = false;
  result.status = ValidationResult::ADAPTER_ID_MISMATCH;
  result.violations.push_back(
    "payload.adapter_id must be 'gpio_pwm_payload_actuator', 'px4_uorb_gripper_actuator', or 'simulation_payload_stub', found: '" + adapter_id + "'");
}
```

- [ ] **Step 2: Add validation test for px4 gripper manifest**

In `full_self_driving/test/launch/launch_manifest_test.cpp`:
```cpp
TEST_F(LaunchManifestTest, AcceptsPx4GripperAdapterInManifest)
{
  std::string yaml_content = R"(
profile: "hardware_rpi4_pixhawk6c_gripper"
manifest_version: "1.0.0"
description: "Test manifest with PX4 native gripper"
approval:
  approved: true
  approval_authority: "safety-board@fsd.roscon25.org"
  approval_evidence_sha256: "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
  approval_timestamp_utc: "2026-08-20T00:00:00Z"
fmu_transport:
  adapter_id: "px4_hardware_uart_serial"
  device_path: "/dev/null"
camera:
  adapter_id: "v4l2_hardware_camera"
  device_path: "/dev/null"
  calibration_file: "config/camera_calibrations/imx219_720p.yaml"
  calibration_sha256: "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
payload:
  adapter_id: "px4_uorb_gripper_actuator"
  transport_interface: "vehicle_command"
  gripper_instance: 1
security:
  sros2_keystore_path: "/tmp"
  require_encryption: false
  require_access_control: false
system_resources:
  max_cpu_percent: 80.0
  max_memory_mb: 2048
  storage_reserve_mb: 1024
  power_loss_recovery_enabled: true
)";

  full_self_driving::launch::HardwareManifestValidator validator;
  auto result = validator.validate_yaml_string(yaml_content);
  EXPECT_TRUE(result.is_valid);
}
```

- [ ] **Step 3: Run manifest validator tests**

Run: `colcon test --packages-select full_self_driving --ctest-args -R launch_manifest_test`
Expected: PASS

- [ ] **Step 4: Commit Task 3 changes**

```bash
git add full_self_driving/src/launch/hardware_manifest_validator.* full_self_driving/simulation/manifests/hardware_schema.yaml full_self_driving/test/launch/launch_manifest_test.cpp
git commit -m "feat(manifest): support px4_uorb_gripper_actuator in hardware manifest schema"
```

---

### Task 4: Runtime Node Wiring & Property 14 Verification

**Files:**
- Modify: `full_self_driving/src/runtime/flight_runtime_node.cpp:330-360`
- Modify: `full_self_driving/test/property/property_14_payload_safety.cpp`
- Modify: `full_self_driving/MANUAL.md:Section 12`

**Interfaces:**
- Consumes: `FlightRuntimeNode`, `PayloadController`, `Px4GripperPayloadAdapter`
- Produces: End-to-end mission sortie with PX4 gripper peripheral

- [ ] **Step 1: Update FlightRuntimeNode to support Px4GripperPayloadAdapter instantiation**

In `full_self_driving/src/runtime/flight_runtime_node.cpp`:
```cpp
if (payload_adapter_type == "px4_uorb_gripper_actuator") {
  full_self_driving::payload::Px4GripperPayloadAdapter::Config cfg;
  cfg.gripper_instance = 1;
  payload_adapter_ = std::make_shared<full_self_driving::payload::Px4GripperPayloadAdapter>(*this, cfg);
} else if (payload_adapter_type == "gpio_pwm_payload_actuator") {
  payload_adapter_ = std::make_shared<full_self_driving::payload::HardwarePayloadAdapter>();
} else {
  payload_adapter_ = std::make_shared<full_self_driving::payload::SimulationPayloadAdapter>();
}
payload_controller_ = std::make_shared<full_self_driving::payload::PayloadController>(payload_adapter_, mission_context_);
```

- [ ] **Step 2: Add Px4GripperPayloadAdapter test case to Property 14 Test Suite**

In `full_self_driving/test/property/property_14_payload_safety.cpp`:
Verify that idempotency, preflight gating, and fail-closed transitions work with `Px4GripperPayloadAdapter`.

- [ ] **Step 3: Run full suite of payload tests**

Run: `colcon test --packages-select full_self_driving --ctest-args -R "payload|property_14"`
Expected: 100% tests PASS

- [ ] **Step 4: Document Section 12 PX4 Gripper configuration in MANUAL.md**

Update `full_self_driving/MANUAL.md` with:
- PX4 Actuator configuration parameters (`PP_GRIPPER_EN`, `PP_GRIP_TYPE`, AUX PWM assignment).
- Electrical wiring guidelines for Pixhawk AUX Servo rail with dedicated UBEC.
- ROS 2 `VehicleCommand` message bridging details.

- [ ] **Step 5: Commit Task 4 changes**

```bash
git add full_self_driving/src/runtime/flight_runtime_node.cpp full_self_driving/test/property/property_14_payload_safety.cpp full_self_driving/MANUAL.md
git commit -m "feat(runtime): integrate Px4GripperPayloadAdapter into flight runtime and verify Property 14"
```
