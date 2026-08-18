#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/utils/message_version.hpp>

#include "domain/live_target_lock.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/internal_strategy.hpp"
#include "flight/strategies/precision_land_strategy.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;
using namespace std::chrono_literals;

class PrecisionLandParityTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    static int node_counter = 0;
    node_ = std::make_shared<rclcpp::Node>("test_precision_land_parity_node_" + std::to_string(++node_counter));
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_precision_land_test");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_);
    coordinator_->bind_executor(executor_, mode_);

    // Setup PX4 publishers matching px4_ros2 subscription topics
    auto qos = rclcpp::QoS(1).best_effort();
    local_pos_pub_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
      "fmu/out/vehicle_local_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(), qos);
    global_pos_pub_ = node_->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
      "fmu/out/vehicle_global_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleGlobalPosition>(), qos);
    home_pos_pub_ = node_->create_publisher<px4_msgs::msg::HomePosition>(
      "fmu/out/home_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::HomePosition>(), qos);
    land_detected_pub_ = node_->create_publisher<px4_msgs::msg::VehicleLandDetected>(
      "fmu/out/vehicle_land_detected" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLandDetected>(), qos);
    vehicle_status_pub_ = node_->create_publisher<px4_msgs::msg::VehicleStatus>(
      "fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), qos);
  }

  void TearDown() override
  {
    executor_.reset();
    mode_.reset();
    coordinator_.reset();
    mission_ctx_.reset();
    state_cache_.reset();
    local_pos_pub_.reset();
    global_pos_pub_.reset();
    home_pos_pub_.reset();
    land_detected_pub_.reset();
    vehicle_status_pub_.reset();
    context_.reset();
    node_.reset();
  }

  void publish_state(
    double lat, double lon, double alt,
    float x = 0.0f, float y = 0.0f, float z = -15.0f,
    float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
    float heading = 0.0f)
  {
    auto now_time = node_->get_clock()->now();
    uint64_t ts_us = now_time.nanoseconds() / 1000;

    px4_msgs::msg::VehicleGlobalPosition global_msg;
    global_msg.timestamp = ts_us;
    global_msg.lat_lon_valid = true;
    global_msg.alt_valid = true;
    global_msg.lat_lon_reset_counter = 0;
    global_msg.alt_reset_counter = 0;
    global_msg.lat = lat;
    global_msg.lon = lon;
    global_msg.alt = static_cast<float>(alt);
    global_pos_pub_->publish(global_msg);

    px4_msgs::msg::VehicleLocalPosition local_msg;
    local_msg.timestamp = ts_us;
    local_msg.xy_valid = true;
    local_msg.z_valid = true;
    local_msg.v_xy_valid = true;
    local_msg.v_z_valid = true;
    local_msg.heading_reset_counter = 0;
    local_msg.x = x;
    local_msg.y = y;
    local_msg.z = z;
    local_msg.vx = vx;
    local_msg.vy = vy;
    local_msg.vz = vz;
    local_msg.heading = heading;
    local_pos_pub_->publish(local_msg);

    px4_msgs::msg::HomePosition home_msg;
    home_msg.timestamp = ts_us;
    home_msg.valid_alt = true;
    home_msg.valid_hpos = true;
    home_msg.valid_lpos = true;
    home_msg.lat = 13.73132845;
    home_msg.lon = 100.78990948;
    home_msg.alt = 2.21f;
    home_msg.x = 0.0f;
    home_msg.y = 0.0f;
    home_msg.z = 0.0f;
    home_msg.yaw = 0.0f;
    home_pos_pub_->publish(home_msg);

    px4_msgs::msg::VehicleLandDetected land_msg;
    land_msg.timestamp = ts_us;
    land_msg.landed = false;
    land_detected_pub_->publish(land_msg);

    px4_msgs::msg::VehicleStatus status_msg;
    status_msg.timestamp = ts_us;
    status_msg.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    status_msg.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    vehicle_status_pub_->publish(status_msg);

    for (int i = 0; i < 5; ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(2ms);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;

  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::HomePosition>::SharedPtr home_pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_pub_;
};

// 1. Strict Prototype Parity Parameters Test
TEST_F(PrecisionLandParityTest, StrictPrototypeParametersParity)
{
  flight::PrecisionLandStrategy strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_);

  // Assert all prototype parameter values are maintained exactly
  EXPECT_FLOAT_EQ(strategy.max_velocity(), 3.0f);        // Prototype max_velocity
  EXPECT_FLOAT_EQ(strategy.descent_vel(), 1.0f);         // Prototype descent_vel
  EXPECT_FLOAT_EQ(strategy.vel_p_gain(), 1.5f);          // Prototype vel_p_gain
  EXPECT_FLOAT_EQ(strategy.vel_i_gain(), 0.0f);          // Prototype vel_i_gain
  EXPECT_FLOAT_EQ(strategy.target_timeout(), 3.0f);      // Prototype target_timeout
  EXPECT_FLOAT_EQ(strategy.delta_position(), 0.25f);     // Prototype delta_position
  EXPECT_FLOAT_EQ(strategy.delta_velocity(), 0.25f);     // Prototype delta_velocity
  EXPECT_FLOAT_EQ(strategy.stabilize_duration(), 1.0f);  // Settle dwell duration
  EXPECT_DOUBLE_EQ(strategy.search_altitude_m(), 15.0);  // Search altitude
  EXPECT_DOUBLE_EQ(strategy.approach_altitude_m(), 5.0); // Approach altitude
}

// 2. Optical to FRD Body to NED World Coordinate Transform Parity Test
TEST_F(PrecisionLandParityTest, CoordinateTransformParity)
{
  adapters::Px4StateSnapshot snapshot;
  snapshot.local_pos_valid = true;
  snapshot.local_position_ned = Eigen::Vector3f(10.0f, 20.0f, -15.0f);
  snapshot.heading = 0.0f;  // Facing North
  snapshot.monotonic_timestamp_ns = 1000000000;

  // Optical frame tag: 0m right, 0m down, 5m straight down along optical axis
  geometry_msgs::msg::Pose tag_pose_optical;
  tag_pose_optical.position.x = 0.0;
  tag_pose_optical.position.y = 0.0;
  tag_pose_optical.position.z = 5.0;
  tag_pose_optical.orientation.w = 1.0;
  tag_pose_optical.orientation.x = 0.0;
  tag_pose_optical.orientation.y = 0.0;
  tag_pose_optical.orientation.z = 0.0;

  auto world_tag = flight::PrecisionLandStrategy::compute_world_tag(tag_pose_optical, snapshot);
  EXPECT_TRUE(world_tag.valid);
  // With drone at NED (10, 20, -15) and optical tag at (0,0,5):
  // Optical (0,0,5) -> FRD (0,0,5) -> NED (10+0, 20+0, -15+5 = -10)
  EXPECT_NEAR(world_tag.position.x(), 10.0, 1e-3);
  EXPECT_NEAR(world_tag.position.y(), 20.0, 1e-3);
  EXPECT_NEAR(world_tag.position.z(), -10.0, 1e-3);
}

// 3. Spiral Search Waypoint Generation Parity Test
TEST_F(PrecisionLandParityTest, SpiralWaypointGenerationParity)
{
  flight::PrecisionLandStrategy strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_);

  Eigen::Vector3f start_pos(0.0f, 0.0f, -15.0f);
  auto waypoints = strategy.generate_search_waypoints(start_pos);

  EXPECT_FALSE(waypoints.empty());
  // Verify start layer begins at initial altitude (-15.0m)
  EXPECT_NEAR(waypoints.front().z(), -15.0f, 1e-3);
  // Verify layer geometry: max radius 2.0m, layer spacing 0.5m
  for (const auto & wp : waypoints) {
    float r = std::sqrt(wp.x() * wp.x() + wp.y() * wp.y());
    EXPECT_LE(r, 2.01f);
  }
}

// 4. Lateral Velocity Controller & Anti-Windup Clamping Test
TEST_F(PrecisionLandParityTest, LateralControllerClampingParity)
{
  flight::PrecisionLandStrategy strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_);

  Eigen::Vector3d drone_pos(10.0, 10.0, -15.0);
  Eigen::Vector3d tag_pos(11.0, 8.0, -15.0);

  // delta_pos_x = 10 - 11 = -1.0 -> vx = -1 * (-1.0 * 1.5) = +1.5 m/s
  // delta_pos_y = 10 - 8 = +2.0 -> vy = -1 * (2.0 * 1.5) = -3.0 m/s
  auto vel = strategy.calculate_velocity_setpoint_xy(drone_pos, tag_pos);
  EXPECT_NEAR(vel.x(), 1.5f, 1e-3);
  EXPECT_NEAR(vel.y(), -3.0f, 1e-3);

  // Large displacement clamping: delta_x = 100m -> clamped to 3.0 m/s max velocity
  Eigen::Vector3d far_drone_pos(100.0, 0.0, -15.0);
  Eigen::Vector3d far_tag_pos(0.0, 0.0, -15.0);
  auto clamped_vel = strategy.calculate_velocity_setpoint_xy(far_drone_pos, far_tag_pos);
  EXPECT_FLOAT_EQ(clamped_vel.x(), -3.0f);
  EXPECT_FLOAT_EQ(clamped_vel.y(), 0.0f);
}

// 5. Target Acquisition Triggers HOVER_BRAKE (Zero-Velocity Position Hold)
TEST_F(PrecisionLandParityTest, TargetAcquisitionTriggersHoverBrake)
{
  publish_state(13.73132845, 100.78990948, 17.21, 0.0f, 0.0f, -15.0f, 4.0f, 0.0f, 0.0f);

  auto strategy = std::make_unique<flight::PrecisionLandStrategy>(
    *node_, mode_->goto_global_setpoint(), state_cache_);
  auto * raw_strat = strategy.get();
  mode_->set_strategy(std::move(strategy));
  raw_strat->on_enter();

  EXPECT_EQ(raw_strat->get_sub_phase(), flight::PrecisionLandSubPhase::SEARCH);

  // Send qualified live target lock
  domain::LiveTargetLock lock;
  lock.identity.marker_id = 42;
  lock.identity.dictionary = "DICT_5X5_100";
  lock.identity.target_namespace = "landing_pad";
  lock.lock_state = domain::LockState::QUALIFIED;
  lock.quality = 0.95f;
  lock.received_monotonic_ns = node_->get_clock()->now().nanoseconds();
  lock.pose.position.x = 0.0;
  lock.pose.position.y = 0.0;
  lock.pose.position.z = 5.0;
  lock.pose.orientation.w = 1.0;

  raw_strat->update_target_lock(lock);

  // Assert that strategy transitioned to HOVER_BRAKE
  EXPECT_EQ(raw_strat->get_sub_phase(), flight::PrecisionLandSubPhase::HOVER_BRAKE);
  EXPECT_FALSE(raw_strat->is_hover_stabilized());
}

// 6. Braking Dynamics and Settle Gates (Velocity decay and dwell time)
TEST_F(PrecisionLandParityTest, BrakingDynamicsAndSettleGates)
{
  // 1. Moving at 4.0 m/s when target acquired
  publish_state(13.73132845, 100.78990948, 17.21, 0.0f, 0.0f, -15.0f, 4.0f, 0.0f, 0.0f);

  auto strategy = std::make_unique<flight::PrecisionLandStrategy>(
    *node_, mode_->goto_global_setpoint(), state_cache_);
  auto * raw_strat = strategy.get();
  mode_->set_strategy(std::move(strategy));
  raw_strat->on_enter();

  domain::LiveTargetLock lock;
  lock.identity.marker_id = 42;
  lock.identity.dictionary = "DICT_5X5_100";
  lock.identity.target_namespace = "landing_pad";
  lock.lock_state = domain::LockState::QUALIFIED;
  lock.quality = 0.95f;
  lock.received_monotonic_ns = node_->get_clock()->now().nanoseconds();
  lock.pose.position.z = 5.0;
  lock.pose.orientation.w = 1.0;

  raw_strat->update_target_lock(lock);
  EXPECT_EQ(raw_strat->get_sub_phase(), flight::PrecisionLandSubPhase::HOVER_BRAKE);

  // While still moving at 4.0 m/s: update setpoint -> cannot settle
  raw_strat->on_update(0.1f);
  EXPECT_FALSE(raw_strat->is_hover_stabilized());
  EXPECT_FLOAT_EQ(raw_strat->hover_settle_duration(), 0.0f);

  // 2. Velocity decays to 0.1 m/s (< delta_velocity 0.25 m/s)
  // Keep publishing fresh settled telemetry and updating
  for (int i = 0; i < 5; ++i) {
    publish_state(13.73132845, 100.78990948, 17.21, 0.0f, 0.0f, -15.0f, 0.1f, 0.0f, 0.0f);
    raw_strat->on_update(0.1f);
  }
  EXPECT_NEAR(raw_strat->hover_settle_duration(), 0.5f, 1e-2);
  EXPECT_FALSE(raw_strat->is_hover_stabilized());

  // Run update for another 6 cycles -> total dwell >= 1.0s -> stabilized!
  for (int i = 0; i < 6; ++i) {
    publish_state(13.73132845, 100.78990948, 17.21, 0.0f, 0.0f, -15.0f, 0.1f, 0.0f, 0.0f);
    raw_strat->on_update(0.1f);
  }
  EXPECT_GE(raw_strat->hover_settle_duration(), 1.0f);
  EXPECT_TRUE(raw_strat->is_hover_stabilized());
}

// 7. Disturbance / Gust Resets Settle Duration during Hover Brake
TEST_F(PrecisionLandParityTest, GustResetsHoverSettleTimer)
{
  publish_state(13.73132845, 100.78990948, 17.21, 0.0f, 0.0f, -15.0f, 0.05f, 0.0f, 0.0f);

  auto strategy = std::make_unique<flight::PrecisionLandStrategy>(
    *node_, mode_->goto_global_setpoint(), state_cache_);
  auto * raw_strat = strategy.get();
  mode_->set_strategy(std::move(strategy));
  raw_strat->on_enter();

  domain::LiveTargetLock lock;
  lock.identity.marker_id = 42;
  lock.identity.dictionary = "DICT_5X5_100";
  lock.identity.target_namespace = "landing_pad";
  lock.lock_state = domain::LockState::QUALIFIED;
  lock.quality = 0.95f;
  lock.received_monotonic_ns = node_->get_clock()->now().nanoseconds();
  lock.pose.position.z = 5.0;
  lock.pose.orientation.w = 1.0;

  raw_strat->update_target_lock(lock);
  EXPECT_EQ(raw_strat->get_sub_phase(), flight::PrecisionLandSubPhase::HOVER_BRAKE);

  // Settle for 0.6s
  for (int i = 0; i < 6; ++i) {
    publish_state(13.73132845, 100.78990948, 17.21, 0.0f, 0.0f, -15.0f, 0.05f, 0.0f, 0.0f);
    raw_strat->on_update(0.1f);
  }
  EXPECT_NEAR(raw_strat->hover_settle_duration(), 0.6f, 1e-2);

  // Gust occurs: velocity spikes to 1.5 m/s (> 0.25 m/s)
  publish_state(13.73132845, 100.78990948, 17.21, 0.0f, 0.0f, -15.0f, 1.5f, 0.0f, 0.0f);
  raw_strat->on_update(0.1f);

  // Settle timer must reset to 0.0s
  EXPECT_FLOAT_EQ(raw_strat->hover_settle_duration(), 0.0f);
  EXPECT_FALSE(raw_strat->is_hover_stabilized());
}
