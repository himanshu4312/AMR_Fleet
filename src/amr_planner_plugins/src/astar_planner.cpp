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

#include "amr_planner_plugins/astar_planner.hpp"

#include <cmath>
#include <algorithm>

namespace amr_planner_plugins
{

double AStarPlanner::heuristic(const CellIndex & a, const CellIndex & b) const
{
  // Octile distance — admissible and consistent for an 8-connected grid:
  // it never overestimates the true remaining cost (there's no cheaper way
  // to cover a diagonal than a diagonal step), which is exactly what
  // GridSearchPlanner::searchPath() requires to guarantee an optimal path.
  double dx = std::abs(a.x - b.x);
  double dy = std::abs(a.y - b.y);
  double res = costmap_->getResolution();
  return res * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
}

}  // namespace amr_planner_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_planner_plugins::AStarPlanner, nav2_core::GlobalPlanner)
