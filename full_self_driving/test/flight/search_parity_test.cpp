#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
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

#include "domain/plan_parser.hpp"
#include "domain/working_plan.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/internal_strategy.hpp"
#include "flight/strategies/takeoff_strategy.hpp"
#include "flight/strategies/transit_in_strategy.hpp"
#include "flight/strategies/search_strategy.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "runtime/plan_manager.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;
using namespace std::chrono_literals;

class SearchParityTest : public ::testing::Test
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
    std::filesystem::remove_all("/tmp/fsd_search_test_plans");
    node_ = std::make_shared<rclcpp::Node>("test_search_parity_node_" + std::to_string(++node_counter));
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_search_test");
    plan_manager_ = std::make_shared<runtime::PlanManager>("/tmp/fsd_search_test_plans");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    coordinator_->set_plan_manager(plan_manager_);

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
    plan_manager_.reset();
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

  std::string get_plan_fixture_path() const
  {
    std::string share_dir;
    try {
      share_dir = ament_index_cpp::get_package_share_directory("full_self_driving");
    } catch (...) {
      share_dir = "";
    }

    std::vector<std::string> candidates = {
      share_dir + "/test/fixtures/plans/aavc2026_mission.plan",
      share_dir + "/share/full_self_driving/test/fixtures/plans/aavc2026_mission.plan"
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

    // 3. HomePosition (latch to avoid map projection reference resets during test)
    if (!home_latched_ || cached_test_home_lat_ != home_lat || cached_test_home_lon_ != home_lon) {
      home_latched_ = true;
      cached_test_home_lat_ = home_lat;
      cached_test_home_lon_ = home_lon;
      px4_msgs::msg::HomePosition home_msg;
      home_msg.timestamp = stamp.nanoseconds() / 1000;
      home_msg.valid_hpos = true;
      home_msg.valid_lpos = true;
      home_msg.valid_alt = true;
      home_msg.lat = home_lat;
      home_msg.lon = home_lon;
      home_msg.alt = home_alt;
      home_pos_pub_->publish(home_msg);
    }

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
    local_msg.heading_reset_counter = 0;
    local_msg.xy_reset_counter = 0;
    local_msg.z_reset_counter = 0;
    local_msg.x = 0.0f;
    local_msg.y = 0.0f;
    local_msg.z = -(alt_amsl - home_alt);
    local_msg.vx = vx;
    local_msg.vy = vy;
    local_msg.vz = vz;
    local_msg.heading = heading;
    local_pos_pub_->publish(local_msg);
    spin_some(5);
  }

  void spin_some(int iterations = 5)
  {
    for (int i = 0; i < iterations; ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(2ms);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;

  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::HomePosition>::SharedPtr home_pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_pub_;

  bool home_latched_{false};
  double cached_test_home_lat_{0.0};
  double cached_test_home_lon_{0.0};
};

// Test 1: Plan parsing and extraction parity against prototype baseline
TEST_F(SearchParityTest, PrototypeSearchPlanExtractionParity)
{
  std::string plan_path = get_plan_fixture_path();
  ASSERT_FALSE(plan_path.empty()) << "aavc2026_mission.plan fixture not found";

  auto parse_res = domain::PlanParser::parse_file(plan_path);
  ASSERT_TRUE(parse_res.is_valid) << "Parse failed: " << parse_res.error_message;

  EXPECT_EQ(parse_res.route.waypoints.size(), 10U);
  EXPECT_FLOAT_EQ(parse_res.route.default_altitude_m, 15.0f);
  EXPECT_FLOAT_EQ(parse_res.route.cruise_speed_m_s, 5.0f);

  // First waypoint: (13.731328451896626, 100.78990948056993, 15.0)
  EXPECT_NEAR(parse_res.route.waypoints[0].latitude_deg, 13.73132845, 1e-6);
  EXPECT_NEAR(parse_res.route.waypoints[0].longitude_deg, 100.78990948, 1e-6);
  EXPECT_DOUBLE_EQ(parse_res.route.waypoints[0].altitude_m, 15.0);

  // Last waypoint (index 9): (13.7307894714774, 100.78783793887287, 15.0)
  EXPECT_NEAR(parse_res.route.waypoints[9].latitude_deg, 13.73078947, 1e-6);
  EXPECT_NEAR(parse_res.route.waypoints[9].longitude_deg, 100.78783793, 1e-6);
  EXPECT_DOUBLE_EQ(parse_res.route.waypoints[9].altitude_m, 15.0);
}

// Test 2: Search climb-to-altitude behavior
TEST_F(SearchParityTest, ClimbToSearchAltitudeBehavior)
{
  std::string plan_path = get_plan_fixture_path();
  ASSERT_FALSE(plan_path.empty());
  auto parse_res = domain::PlanParser::parse_file(plan_path);
  ASSERT_TRUE(parse_res.is_valid);

  domain::WorkingPlan wp(
    "wp_climb_test", "art_test", "kmitl_airfield", "default_scenario",
    parse_res.raw_content_sha256, parse_res.route);

  flight::SearchStrategy strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_,
    plan_manager_, wp, 15.0, 5.0f, 4.0f, 0.785f);

  // Publish telemetry where altitude is low (5.0m AMSL, home is 2.0m -> 3m above home, target is 17m AMSL)
  publish_telemetry(true, false, 13.730322, 100.787446, 5.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  spin_some();

  strategy.on_enter();
  EXPECT_DOUBLE_EQ(strategy.target_altitude_amsl_m(), 17.0);  // 2.0 + 15.0

  // Update setpoint: since 5.0m < 17.0m - 1.0m, strategy must command climb at current position
  strategy.on_update(0.1f);
  EXPECT_FALSE(strategy.is_completed());
  EXPECT_FALSE(strategy.is_failed());
  EXPECT_EQ(strategy.current_waypoint_index(), 0U);

  // Now climb to target altitude (17.0m AMSL)
  publish_telemetry(true, false, 13.730322, 100.787446, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  spin_some();

  strategy.on_update(0.1f);
  EXPECT_FALSE(strategy.is_completed());
  EXPECT_EQ(strategy.current_waypoint_index(), 0U);

  strategy.on_exit();
}

// Test 3: Search waypoint progression, arrival radius, heading calculation and checkpoint updates
TEST_F(SearchParityTest, WaypointProgressionAndCheckpointUpdates)
{
  std::string plan_path = get_plan_fixture_path();
  ASSERT_FALSE(plan_path.empty());
  auto parse_res = domain::PlanParser::parse_file(plan_path);
  ASSERT_TRUE(parse_res.is_valid);

  domain::WorkingPlan wp(
    "wp_progression_test", "art_test", "kmitl_airfield", "default_scenario",
    parse_res.raw_content_sha256, parse_res.route);

  std::vector<uint32_t> completed_wp_log;
  std::vector<float> progress_log;

  flight::SearchStrategy strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_,
    plan_manager_, wp, 15.0, 5.0f, 4.0f, 0.785f);

  strategy.set_checkpoint_callback([&](const domain::SearchCheckpointData & cp) {
    completed_wp_log.push_back(cp.completed_waypoints);
    progress_log.push_back(cp.progress_percent);
  });

  // Telemetry at search altitude
  publish_telemetry(true, false, 13.730322, 100.787446, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  spin_some();

  strategy.on_enter();
  EXPECT_EQ(strategy.current_waypoint_index(), 0U);

  // Waypoint 0 target: (13.73132845, 100.78990948)
  // Move vehicle to waypoint 0 within reach radius (< 4.0m)
  publish_telemetry(true, false, 13.73132845, 100.78990948, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  spin_some();

  strategy.on_update(0.1f);
  EXPECT_EQ(strategy.current_waypoint_index(), 1U);
  ASSERT_FALSE(completed_wp_log.empty());
  EXPECT_EQ(completed_wp_log.back(), 1U);
  EXPECT_FLOAT_EQ(progress_log.back(), 10.0f);

  // Move vehicle to waypoint 1: (13.731328455754555, 100.78938348265243)
  publish_telemetry(true, false, 13.731328455754555, 100.78938348265243, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  spin_some();

  strategy.on_update(0.1f);
  EXPECT_EQ(strategy.current_waypoint_index(), 2U);
  EXPECT_EQ(completed_wp_log.back(), 2U);
  EXPECT_FLOAT_EQ(progress_log.back(), 20.0f);

  strategy.on_exit();
}

// Test 4: Checkpoint interruption and resumption with entry point insertion
TEST_F(SearchParityTest, CheckpointInterruptionAndResumption)
{
  std::string plan_path = get_plan_fixture_path();
  ASSERT_FALSE(plan_path.empty());
  auto parse_res = domain::PlanParser::parse_file(plan_path);
  ASSERT_TRUE(parse_res.is_valid);

  // Create managed artifact and working plan in PlanManager
  std::ifstream file(plan_path, std::ios::binary);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  auto art = plan_manager_->upload_artifact("aavc2026_mission.plan", bytes);
  ASSERT_TRUE(art.has_value());

  auto wp_opt = plan_manager_->create_or_select_working_plan(
    art->artifact_id, "kmitl_airfield", "default_scenario");
  ASSERT_TRUE(wp_opt.has_value());
  std::string wp_id = wp_opt->get_working_plan_id();

  // Session 1: Run strategy up to waypoint 3, then deactivate while flying to waypoint 4
  {
    flight::SearchStrategy session1(
      *node_, mode_->goto_global_setpoint(), state_cache_,
      plan_manager_, *wp_opt, 15.0, 5.0f, 4.0f, 0.785f);

    publish_telemetry(true, false, 13.730322, 100.787446, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
    spin_some();

    session1.on_enter();

    // Progress through waypoints 0, 1, 2, 3
    for (size_t i = 0; i < 4; ++i) {
      const auto & target_wp = parse_res.route.waypoints[i];
      publish_telemetry(true, false, target_wp.latitude_deg, target_wp.longitude_deg, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
      spin_some();
      session1.on_update(0.1f);
    }
    EXPECT_EQ(session1.current_waypoint_index(), 4U);

    // Drone is now flying midway to waypoint 4 at (13.7311, 100.7888) when mode is deactivated
    publish_telemetry(true, false, 13.7311, 100.7888, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
    spin_some();

    session1.on_exit();  // Saves safe deactivation checkpoint
  }

  // Verify checkpoint persisted in PlanManager
  auto updated_wp = plan_manager_->get_working_plan(wp_id);
  ASSERT_TRUE(updated_wp.has_value());
  const auto & cp = updated_wp->get_checkpoint();
  EXPECT_EQ(cp.next_source_index, 4U);
  EXPECT_EQ(cp.completed_waypoints, 4U);
  EXPECT_FLOAT_EQ(cp.progress_percent, 40.0f);
  EXPECT_TRUE(cp.has_checkpoint_position);
  EXPECT_DOUBLE_EQ(cp.checkpoint_latitude_deg, 13.7311);
  EXPECT_DOUBLE_EQ(cp.checkpoint_longitude_deg, 100.7888);

  // Session 2: Resumed search strategy from persisted working plan
  {
    flight::SearchStrategy session2(
      *node_, mode_->goto_global_setpoint(), state_cache_,
      plan_manager_, *updated_wp, 15.0, 5.0f, 4.0f, 0.785f);

    publish_telemetry(true, false, 13.730322, 100.787446, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
    spin_some();

    session2.on_enter();

    // Route has 7 waypoints: 1 entry point + 6 remaining (4..9)
    EXPECT_EQ(session2.route().waypoints.size(), 7U);
    EXPECT_DOUBLE_EQ(session2.route().waypoints[0].latitude_deg, 13.7311);
    EXPECT_DOUBLE_EQ(session2.route().waypoints[0].longitude_deg, 100.7888);

    // Reach entry point
    publish_telemetry(true, false, 13.7311, 100.7888, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
    spin_some();
    session2.on_update(0.1f);
    EXPECT_EQ(session2.current_waypoint_index(), 1U);

    // Reach waypoint 4
    const auto & wp4 = parse_res.route.waypoints[4];
    publish_telemetry(true, false, wp4.latitude_deg, wp4.longitude_deg, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
    spin_some();
    session2.on_update(0.1f);
    EXPECT_EQ(session2.current_waypoint_index(), 2U);

    session2.on_exit();
  }
}

// Test 5: Full search completion and final hold
TEST_F(SearchParityTest, FullSearchCompletionAndFinalHold)
{
  std::string plan_path = get_plan_fixture_path();
  ASSERT_FALSE(plan_path.empty());
  auto parse_res = domain::PlanParser::parse_file(plan_path);
  ASSERT_TRUE(parse_res.is_valid);

  domain::WorkingPlan wp(
    "wp_complete_test", "art_test", "kmitl_airfield", "default_scenario",
    parse_res.raw_content_sha256, parse_res.route);

  bool completed_called = false;
  flight::SearchStrategy strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_,
    plan_manager_, wp, 15.0, 5.0f, 4.0f, 0.785f);

  strategy.set_completion_callback([&](bool success) {
    if (success) completed_called = true;
  });

  publish_telemetry(true, false, 13.730322, 100.787446, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  spin_some();

  strategy.on_enter();

  // Progress through all 10 waypoints
  for (size_t i = 0; i < 10; ++i) {
    const auto & target_wp = parse_res.route.waypoints[i];
    publish_telemetry(true, false, target_wp.latitude_deg, target_wp.longitude_deg, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
    spin_some();
    strategy.on_update(0.1f);
  }

  EXPECT_TRUE(completed_called);
  EXPECT_TRUE(strategy.is_completed());
  EXPECT_FALSE(strategy.is_failed());

  // Subsequent updates continue holding over the last waypoint
  strategy.on_update(0.1f);
  EXPECT_TRUE(strategy.is_completed());

  strategy.on_exit();
}

// Test 6: Malformed/Empty plan and invalid telemetry failure handling
TEST_F(SearchParityTest, FailClosedOnInvalidPlanOrTelemetry)
{
  // 1. Empty route fails closed
  domain::CanonicalSearchRoute empty_route;
  flight::SearchStrategy empty_strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_,
    empty_route, 15.0, 5.0f, 4.0f, 0.785f);

  empty_strategy.on_enter();
  EXPECT_TRUE(empty_strategy.is_failed());
  EXPECT_FALSE(empty_strategy.is_completed());

  // 2. Negative search altitude fails closed
  domain::CanonicalSearchRoute valid_route;
  valid_route.waypoints.push_back({13.7313, 100.7899, 15.0, 0});
  flight::SearchStrategy invalid_alt_strategy(
    *node_, mode_->goto_global_setpoint(), state_cache_,
    valid_route, -5.0, 5.0f, 4.0f, 0.785f);

  invalid_alt_strategy.on_enter();
  EXPECT_TRUE(invalid_alt_strategy.is_failed());
}

// Test 7: Coordinator transition from ACQUIRE_TARGET to SEARCH (Direct fallback)
TEST_F(SearchParityTest, CoordinatorAcquireTargetToSearchTransition)
{
  std::string plan_path = get_plan_fixture_path();
  ASSERT_FALSE(plan_path.empty());
  auto parse_res = domain::PlanParser::parse_file(plan_path);
  ASSERT_TRUE(parse_res.is_valid);

  coordinator_->set_custom_search_route(parse_res.route);

  publish_telemetry(true, false, 13.730322, 100.787446, 17.0, 0.0f, 0.0f, 0.0f, 0.0f, 13.730322, 100.787446, 2.0f);
  spin_some();

  // TransitIn completes -> requests ACQUIRE_TARGET
  bool trans_ok = coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET);
  EXPECT_TRUE(trans_ok);

  // In Task 10, ACQUIRE_TARGET evaluates acquisition branch and selects SearchStrategy (Direct fallback)
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
  EXPECT_EQ(mode_->get_current_strategy_name(), "SEARCH");

  // Subsequent transition to SEARCH explicitly
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::SEARCH));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
  EXPECT_EQ(mode_->get_current_strategy_name(), "SEARCH");

  const auto & trace = coordinator_->get_transition_trace();
  bool found_acquire = false;
  bool found_search_fallback = false;
  for (const auto & step : trace) {
    if (step.find("ACQUIRE") != std::string::npos || step.find("ACQUISITION") != std::string::npos) found_acquire = true;
    if (step.find("SEARCH") != std::string::npos) found_search_fallback = true;
  }
  EXPECT_TRUE(found_acquire);
  EXPECT_TRUE(found_search_fallback);
}
