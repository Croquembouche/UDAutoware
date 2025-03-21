// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__GOAL_PLANNER__DEFAULT_FIXED_GOAL_PLANNER_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__GOAL_PLANNER__DEFAULT_FIXED_GOAL_PLANNER_HPP_

#include "behavior_path_planner/utils/goal_planner/fixed_goal_planner_base.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{

class DefaultFixedGoalPlanner : public FixedGoalPlannerBase
{
public:
  DefaultFixedGoalPlanner() = default;
  BehaviorModuleOutput plan(const std::shared_ptr<const PlannerData> & planner_data) const override;

protected:
  boost::optional<BehaviorModuleOutput> getReferencePath(
    const std::shared_ptr<const PlannerData> & planner_data) const;
  PathWithLaneId modifyPathForSmoothGoalConnection(
    const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data) const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__GOAL_PLANNER__DEFAULT_FIXED_GOAL_PLANNER_HPP_
