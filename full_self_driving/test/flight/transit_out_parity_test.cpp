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
#include "flight/strategies/transit_out_strategy.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;
using namespace std::chrono_literals;

class TransitOutParityTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_transit_out_parity_node_" + std::to_string(++node_counter));
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_transit_out_test");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_);
    coordinator_->bind_executor(executor_, mode_);

    executor_->set_takeover_callback([this](flight::FullSelfDrivingModeExecutor::DeactivateReason reason) {
      if (coordinator_) {
        coordinator_->handle_takeover(reason);
      }
    });

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
      share_dir + "/test/fixtures/prototype_behavior/transit_out/golden_transit_out_waypoints.yaml",
      share_dir + "/share/full_self_driving/test/fixtures/prototype_behavior/transit_out/golden_transit_out_waypoints.yaml"
    };

    for (const auto & path : candidates) {
      if (std::filesystem::exists(path)) {
        return path;
      }
    }
    return candidates.front();
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

// 1. Test Parameter Loading & Route Validation Parity
TEST_F(TransitOutParityTest, ParameterLoadingAndValidationParity)
{
  std::string fixture_path = get_fixture_path();
  ASSERT_TRUE(std::filesystem::exists(fixture_path)) << "Fixture missing at: " << fixture_path;

  auto route = domain::Route::from_yaml_file(fixture_path);
  EXPECT_TRUE(route.validate());
  EXPECT_EQ(route.size(), 3u);
  EXPECT_DOUBLE_EQ(route.get_transit_altitude_above_home_m(), 15.0);
  EXPECT_FLOAT_EQ(route.get_arrival_radius_m(), 4.0f);
  EXPECT_FLOAT_EQ(route.get_max_horizontal_speed_m_s(), 5.0f);
  EXPECT_FLOAT_EQ(route.get_max_vertical_speed_m_s(), 1.0f);
  EXPECT_FLOAT_EQ(route.get_max_heading_rate_deg_s(), 45.0f);
}

// 2. Test Outbound Route Waypoints Construction
TEST_F(TransitOutParityTest, DefaultKmitlTransitOutRoute)
{
  auto route = domain::Route::create_default_kmitl_transit_out_route();
  EXPECT_TRUE(route.validate());
  EXPECT_EQ(route.size(), 3u);
  EXPECT_DOUBLE_EQ(route.get_transit_altitude_above_home_m(), 15.0);
  EXPECT_DOUBLE_EQ(route[0].latitude_deg, 13.730712);
  EXPECT_DOUBLE_EQ(route[0].longitude_deg, 100.788755);
}

// 3. Test Strategy Execution Flow & Waypoint Progression
TEST_F(TransitOutParityTest, StrategyExecutionProgression)
{
  auto route = domain::Route::create_default_kmitl_transit_out_route();
  flight::TransitOutStrategy strategy(*node_, *context_, state_cache_, route);

  strategy.on_enter();
  EXPECT_EQ(strategy.get_type(), flight::StrategyType::TRANSIT_OUT);
  EXPECT_EQ(strategy.current_waypoint_index(), 0u);
  EXPECT_FALSE(strategy.is_completed());
}

// 4. Test Coordinator Transition to TransitOut
TEST_F(TransitOutParityTest, CoordinatorTransitionsToTransitOut)
{
  std::string err;
  bool ok = coordinator_->request_transition(flight::StrategyType::TRANSIT_OUT, &err);
  EXPECT_TRUE(ok) << "Error: " << err;
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_OUT);
}
