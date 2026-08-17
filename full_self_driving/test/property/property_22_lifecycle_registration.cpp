#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "persistence/persistence_manager.hpp"
#include "runtime/lifecycle_supervisor.hpp"
#include "adapters/px4_api_capabilities.hpp"

using namespace full_self_driving;

class Property22LifecycleRegistrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
    supervisor_ = std::make_shared<runtime::LifecycleSupervisor>();

    persistence::StoragePaths paths;
    paths.state_directory = "/tmp/fsd_state_p22";
    paths.evidence_directory = "/tmp/fsd_evidence_p22";
    paths.plan_directory = "/tmp/fsd_plans_p22";
    paths.backup_directory = "/tmp/fsd_backups_p22";
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);
  }

  std::shared_ptr<domain::EngineeringConfig> config_;
  std::shared_ptr<runtime::LifecycleSupervisor> supervisor_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
};

// Property 22.1: Unconfigured / inactive lifecycle nodes strictly block mode registration readiness
TEST_F(Property22LifecycleRegistrationTest, Property22_InactiveLifecycleNodesBlockRegistration)
{
  std::vector<std::string> missing;
  bool ready = supervisor_->evaluate_runtime_readiness(
    true,   // config ok
    true,   // persistence ok
    true,   // recovery clear
    true,   // px4 transport ok
    &missing);

  EXPECT_FALSE(ready);
  EXPECT_FALSE(missing.empty());

  bool found_lifecycle_gate = false;
  for (const auto & gate : missing) {
    if (gate.find("LIFECYCLE") != std::string::npos || gate.find("lifecycle") != std::string::npos) {
      found_lifecycle_gate = true;
    }
  }
  EXPECT_TRUE(found_lifecycle_gate);
}

// Property 22.2: Partial lifecycle activation still blocks registration
TEST_F(Property22LifecycleRegistrationTest, Property22_PartialLifecycleActivationBlocksRegistration)
{
  // Configure all
  EXPECT_TRUE(supervisor_->configure_all());

  // Activate only first 2 nodes: fsd_pad_registry, fsd_perception
  EXPECT_TRUE(supervisor_->activate_node("fsd_pad_registry"));
  EXPECT_TRUE(supervisor_->activate_node("fsd_perception"));
  EXPECT_FALSE(supervisor_->is_all_active());

  std::vector<std::string> missing;
  bool ready = supervisor_->evaluate_runtime_readiness(true, true, true, true, &missing);
  EXPECT_FALSE(ready);
  EXPECT_FALSE(missing.empty());
}

// Property 22.3: Transport and recovery failures block registration even when all lifecycle nodes are active
TEST_F(Property22LifecycleRegistrationTest, Property22_TransportAndRecoveryFailuresBlockRegistration)
{
  EXPECT_TRUE(supervisor_->configure_all());
  EXPECT_TRUE(supervisor_->activate_all());
  EXPECT_TRUE(supervisor_->is_all_active());

  // 1. PX4 transport failure
  {
    std::vector<std::string> missing;
    bool ready = supervisor_->evaluate_runtime_readiness(true, true, true, false, &missing);
    EXPECT_FALSE(ready);
    EXPECT_EQ(missing.size(), 1u);
    EXPECT_EQ(missing[0], "PX4_TRANSPORT_NOT_READY");
  }

  // 2. Recovery required failure
  {
    std::vector<std::string> missing;
    bool ready = supervisor_->evaluate_runtime_readiness(true, true, false, true, &missing);
    EXPECT_FALSE(ready);
    EXPECT_EQ(missing.size(), 1u);
    EXPECT_EQ(missing[0], "RECOVERY_REQUIRED");
  }

  // 3. Storage failure
  {
    std::vector<std::string> missing;
    bool ready = supervisor_->evaluate_runtime_readiness(true, false, true, true, &missing);
    EXPECT_FALSE(ready);
    EXPECT_EQ(missing.size(), 1u);
    EXPECT_EQ(missing[0], "PERSISTENCE_NOT_HEALTHY");
  }
}

// Property 22.4: All gates passing enables external mode registration
TEST_F(Property22LifecycleRegistrationTest, Property22_AllGatesPassEnablesRegistration)
{
  EXPECT_TRUE(supervisor_->configure_all());
  EXPECT_TRUE(supervisor_->activate_all());
  EXPECT_TRUE(supervisor_->is_all_active());

  std::vector<std::string> missing;
  bool ready = supervisor_->evaluate_runtime_readiness(true, true, true, true, &missing);
  EXPECT_TRUE(ready);
  EXPECT_TRUE(missing.empty());

  // If a node subsequently fails or deactivates, readiness is immediately withdrawn
  EXPECT_TRUE(supervisor_->deactivate_node("fsd_gateway"));
  EXPECT_FALSE(supervisor_->is_all_active());

  ready = supervisor_->evaluate_runtime_readiness(true, true, true, true, &missing);
  EXPECT_FALSE(ready);
  EXPECT_FALSE(missing.empty());
}
