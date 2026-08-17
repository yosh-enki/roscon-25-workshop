#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "domain/working_plan.hpp"

namespace full_self_driving::runtime
{

class WorkingPlanStore
{
public:
  explicit WorkingPlanStore(const std::string & working_directory = "");
  ~WorkingPlanStore() = default;

  const std::string & get_working_directory() const { return working_directory_; }

  bool save_working_plan(
    const domain::WorkingPlan & working_plan,
    std::string * out_error = nullptr);

  std::optional<domain::WorkingPlan> load_working_plan(
    const std::string & working_plan_id,
    std::string * out_error = nullptr) const;

  std::vector<std::string> list_working_plan_ids() const;

private:
  std::string working_directory_;
};

}  // namespace full_self_driving::runtime
