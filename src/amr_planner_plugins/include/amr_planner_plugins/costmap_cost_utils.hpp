// Copyright 2026 Himanshu
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

#ifndef AMR_PLANNER_PLUGINS__COSTMAP_COST_UTILS_HPP_
#define AMR_PLANNER_PLUGINS__COSTMAP_COST_UTILS_HPP_

#include "nav2_costmap_2d/cost_values.hpp"

namespace amr_planner_plugins
{

// Shared by the grid-search planners (via cell cost) and RRT (via
// point-sampled cost) so the "what counts as blocked" rule lives in one place.
inline bool isLethalCost(unsigned char cost, bool allow_unknown)
{
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return true;
  }
  return cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown;
}

}  // namespace amr_planner_plugins

#endif  // AMR_PLANNER_PLUGINS__COSTMAP_COST_UTILS_HPP_
