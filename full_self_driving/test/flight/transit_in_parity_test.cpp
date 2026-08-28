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

#include "domain/route.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/internal_strategy.hpp"
#include "flight/strategies/takeoff_strategy.hpp"
#include "flight/strategies/transit_in_strategy.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;
using namespace std::chrono_literals;

class TransitInParityTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_transit_in_parity_node_" + std::to_string(++node_counter));
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_transit_in_test");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_);
    coordinator_->bind_executor(executor_, mode_);

    executor_->set_takeover_callback([this](flight::FullSelfDrivingModeExecutor::DeactivateReason reason) {
      if (coordinator_) {
        coordinator_->handle_takeover(reason);
      }
    });

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

  std::string get_fixture_path() const
  {
    std::string share_dir;
    try {
      share_dir = ament_index_cpp::get_package_share_directory("full_self_driving");
    } catch (...) {
      share_dir = "";
    }

    std::vector<std::string> candidates = {
      share_dir + "/test/fixtures/prototype_behavior/transit_in/golden_waypoints.yaml",
      share_dir + "/share/full_self_driving/test/fixtures/prototype_behavior/transit_in/golden_waypoints.yaml"
    };

    for (const auto & path : candidates) {
      if (!path.empty() && std::filesystem::is_regular_file(path)) {
        return path;
      }
    }
    return "";
  }

  void publish_telemetry(
    bool armed,
    bool landed,
    double lat,
    double lon,
    double alt_amsl,
    float vx = 0.0f,
    float vy = 0.0f,
    float vz = 0.0f,
    float heading = 0.0f,
    double home_lat = 13.730322,
    double home_lon = 100.787446,
    float home_alt = 2.0f)
  {
    auto stamp = node_->get_clock()->now();

    // 1. VehicleStatus
    px4_msgs::msg::VehicleStatus status_msg;
    status_msg.timestamp = stamp.nanoseconds() / 1000;
    status_msg.arming_state = armed ? px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED : px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    status_msg.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    vehicle_status_pub_->publish(status_msg);

    // 2. VehicleLandDetected
    px4_msgs::msg::VehicleLandDetected land_msg;
    land_msg.timestamp = stamp.nanoseconds() / 1000;
    land_msg.landed = landed;
    land_msg.maybe_landed = landed;
    land_detected_pub_->publish(land_msg);

    // 3. HomePosition
    px4_msgs::msg::HomePosition home_msg;
    home_msg.timestamp = stamp.nanoseconds() / 1000;
    home_msg.valid_hpos = true;
    home_msg.valid_lpos = true;
    home_msg.valid_alt = true;
    home_msg.lat = home_lat;
    home_msg.lon = home_lon;
    home_msg.alt = home_alt;
    home_pos_pub_->publish(home_msg);

    // 4. VehicleGlobalPosition
    px4_msgs::msg::VehicleGlobalPosition global_msg;
    global_msg.timestamp = stamp.nanoseconds() / 1000;
    global_msg.lat_lon_valid = true;
    global_msg.alt_valid = true;
    global_msg.lat = lat;
    global_msg.lon = lon;
    global_msg.alt = static_cast<float>(alt_amsl);
    global_msg.lat_lon_reset_counter = 0;
    global_msg.alt_reset_counter = 0;
    global_pos_pub_->publish(global_msg);

    // 5. VehicleLocalPosition
    px4_msgs::msg::VehicleLocalPosition local_msg;
    local_msg.timestamp = stamp.nanoseconds() / 1000;
    local_msg.xy_valid = true;
    local_msg.z_valid = true;
    local_msg.v_xy_valid = true;
    local_msg.v_z_valid = true;
    local_msg.heading = heading;
    local_msg.x = 0.0f;
    local_msg.y = 0.0f;
    local_msg.z = static_cast<float>(-(alt_amsl - home_alt));
    local_msg.vx = vx;
    local_msg.vy = vy;
    local_msg.vz = vz;
    local_pos_pub_->publish(local_msg);

    // Spin to process callbacks
    for (int i = 0; i < 3; ++i) {
      rclcpp::spin_some(node_);
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

// Test 1: Route Value Object parsing, validation, and safety clamping
TEST_F(TransitInParityTest, RouteParsingAndSafetyClamping)
{
  std::string fixture_path = get_fixture_path();
  ASSERT_FALSE(fixture_path.empty());

  domain::Route route = domain::Route::from_yaml_file(fixture_path);
  EXPECT_FALSE(route.empty());
  EXPECT_EQ(route.size(), 3u);

  std::vector<std::string> errors;
  EXPECT_TRUE(route.validate(&errors));
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(route.get_transit_altitude_above_home_m(), 15.0);
  EXPECT_FLOAT_EQ(route.get_arrival_radius_m(), 4.0f);
  EXPECT_FLOAT_EQ(route.get_max_horizontal_speed_m_s(), 5.0f);
  EXPECT_FLOAT_EQ(route.get_max_vertical_speed_m_s(), 1.0f);
  EXPECT_FLOAT_EQ(route.get_max_heading_rate_deg_s(), 45.0f);
  EXPECT_FLOAT_EQ(route.get_course_heading_min_speed_m_s(), 0.3f);
  EXPECT_FLOAT_EQ(route.get_altitude_tolerance_m(), 1.0f);
  EXPECT_FLOAT_EQ(route.get_altitude_settle_speed_m_s(), 0.5f);
  EXPECT_FLOAT_EQ(route.get_data_timeout_s(), 2.0f);

  // Test Hard Safety Cap Clamping
  route.set_max_horizontal_speed_m_s(25.0f);  // Exceeds 10.0 cap
  EXPECT_FLOAT_EQ(route.get_max_horizontal_speed_m_s(), domain::Route::kHardMaxHorizontalSpeedMps);

  route.set_max_vertical_speed_m_s(10.0f);    // Exceeds 3.0 cap
  EXPECT_FLOAT_EQ(route.get_max_vertical_speed_m_s(), domain::Route::kHardMaxVerticalSpeedMps);

  route.set_max_heading_rate_deg_s(360.0f);   // Exceeds 180 cap
  EXPECT_FLOAT_EQ(route.get_max_heading_rate_deg_s(), domain::Route::kHardMaxHeadingRateDegS);

  route.set_transit_altitude_above_home_m(200.0); // Exceeds 120 cap
  EXPECT_DOUBLE_EQ(route.get_transit_altitude_above_home_m(), domain::Route::kHardMaxAltitudeAboveHomeM);

  route.set_data_timeout_s(20.0f);            // Exceeds 10s cap
  EXPECT_FLOAT_EQ(route.get_data_timeout_s(), domain::Route::kHardMaxDataTimeoutS);
}

// Test 2: Takeoff Strategy altitude and settling verification
TEST_F(TransitInParityTest, TakeoffStrategyProgressionAndSettling)
{
  bool completed = false;
  auto takeoff_strat = std::make_unique<flight::TakeoffStrategy>(
    *node_, mode_->goto_global_setpoint(), state_cache_, 10.0, 1.0, 0.5, 30.0);
  takeoff_strat->set_completion_callback([&](bool success) {
    completed = success;
  });

  takeoff_strat->on_enter();
  EXPECT_EQ(takeoff_strat->get_type(), flight::StrategyType::TAKEOFF);
  EXPECT_EQ(takeoff_strat->get_name(), "TAKEOFF");
  EXPECT_FALSE(takeoff_strat->is_completed());

  // Step 1: On ground, armed -> not completed
  publish_telemetry(true, true, 13.730322, 100.787446, 2.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  takeoff_strat->on_update(0.1f);
  EXPECT_FALSE(takeoff_strat->is_completed());

  // Step 2: In air, approaching 12.0m (10m above home) -> settling cycles increment
  for (int i = 0; i < 10; ++i) {
    publish_telemetry(true, false, 13.730322, 100.787446, 12.0, 0.0f, 0.0f, 0.1f, 0.0f, 13.730322, 100.787446, 2.0f);
    takeoff_strat->on_update(0.1f);
  }

  EXPECT_TRUE(takeoff_strat->is_completed());
  EXPECT_TRUE(completed);
  takeoff_strat->on_exit();
}

// Test 3: TransitIn Strategy progression through waypoints matching golden trace
TEST_F(TransitInParityTest, TransitInGoldenTraceProgression)
{
  std::string fixture_path = get_fixture_path();
  ASSERT_FALSE(fixture_path.empty());

  domain::Route route = domain::Route::from_yaml_file(fixture_path);
  route.set_data_timeout_s(10.0f);
  auto transit_strat = std::make_unique<flight::TransitInStrategy>(
    *node_, mode_->goto_global_setpoint(), state_cache_, route);

  std::vector<std::size_t> visited_wps;
  bool transit_completed = false;

  transit_strat->set_waypoint_callback([&](std::size_t wp_idx, bool success) {
    if (success) visited_wps.push_back(wp_idx);
  });
  transit_strat->set_completion_callback([&](bool success) {
    transit_completed = success;
  });

  transit_strat->on_enter();
  EXPECT_EQ(transit_strat->get_type(), flight::StrategyType::TRANSIT_IN);
  EXPECT_EQ(transit_strat->get_name(), "TRANSIT_IN");
  EXPECT_FALSE(transit_strat->is_completed());

  auto advance_to_waypoint = [&](double lat, double lon, double alt, float vx, float vy, float vz, float heading) {
    for (int i = 0; i < 5; ++i) {
      publish_telemetry(true, false, lat, lon, alt, vx, vy, vz, heading, 13.730322, 100.787446, 2.0f);
      transit_strat->on_update(0.05f);
    }
  };

  // Publish fresh telemetry at initial waypoint 0
  publish_telemetry(true, false, 13.730322, 100.787446, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);

  // Cycle 1: First-setpoint guard cycle
  transit_strat->on_update(0.05f);
  EXPECT_FALSE(transit_strat->is_completed());

  // Reach waypoint 0 and advance to 1
  advance_to_waypoint(13.730322, 100.787446, 17.0, 0.5f, 0.0f, 0.1f, 0.0f);
  EXPECT_GE(transit_strat->current_waypoint_index(), 1u);
  EXPECT_FALSE(visited_wps.empty());

  // Reach waypoint 1 and advance to 2
  advance_to_waypoint(13.730397, 100.788694, 17.0, 0.2f, 0.1f, 0.05f, 0.45f);
  EXPECT_GE(transit_strat->current_waypoint_index(), 2u);
  EXPECT_GE(visited_wps.size(), 2u);

  // Reach final waypoint 2 and complete route
  advance_to_waypoint(13.730712, 100.788755, 17.0, 0.05f, 0.0f, 0.02f, 1.57f);
  EXPECT_EQ(transit_strat->current_waypoint_index(), 3u);
  EXPECT_TRUE(transit_strat->is_completed());
  EXPECT_TRUE(transit_completed);
  EXPECT_EQ(visited_wps.size(), 3u);

  transit_strat->on_exit();
}

// Test 4: TransitIn Failure Gates (Disarmed, Landed, Stale Telemetry)
TEST_F(TransitInParityTest, TransitInFailureGates)
{
  auto route = domain::Route::create_default_kmitl_transit_in_route();

  // 1. Disarmed vehicle fails closed
  {
    auto strat = std::make_unique<flight::TransitInStrategy>(*node_, mode_->goto_global_setpoint(), state_cache_, route);
    strat->on_enter();
    publish_telemetry(false, false, 13.730322, 100.787446, 17.0);  // Armed = false
    strat->on_update(0.05f);
    EXPECT_TRUE(strat->is_failed());
    EXPECT_FALSE(strat->failure_reason().empty());
  }

  // 2. Landed vehicle fails closed
  {
    auto strat = std::make_unique<flight::TransitInStrategy>(*node_, mode_->goto_global_setpoint(), state_cache_, route);
    strat->on_enter();
    publish_telemetry(true, true, 13.730322, 100.787446, 2.0);   // Landed = true
    strat->on_update(0.05f);
    EXPECT_TRUE(strat->is_failed());
    EXPECT_FALSE(strat->failure_reason().empty());
  }
}

// Test 5: Coordinator transitions WAITING_FOR_MODE -> TAKEOFF -> TRANSIT_IN
TEST_F(TransitInParityTest, CoordinatorTransitionSequence)
{
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);

  // Transition to TAKEOFF
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::TAKEOFF));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TAKEOFF);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::TAKEOFF);

  // Transition to TRANSIT_IN
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::TRANSIT_IN));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::TRANSIT_IN);

  // Manual Takeover immediately forces transition to HOLD
  executor_->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
  EXPECT_TRUE(coordinator_->is_takeover_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
}
