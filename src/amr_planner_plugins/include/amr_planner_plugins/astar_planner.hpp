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

#ifndef AMR_PLANNER_PLUGINS__ASTAR_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__ASTAR_PLANNER_HPP_

#include <string>

#include "amr_planner_plugins/grid_search_planner.hpp"

namespace amr_planner_plugins
{

// A* Strategy: GridSearchPlanner's shared search loop plus an octile-distance
// heuristic, so the search is biased toward the goal instead of expanding
// uniformly in every direction like Dijkstra does.
class AStarPlanner : public GridSearchPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

protected:
  double heuristic(const CellIndex & a, const CellIndex & b) const override;
  std::string plannerTypeName() const override {return "AStarPlanner";}
};

}  // namespace amr_planner_plugins

#endif  // AMR_PLANNER_PLUGINS__ASTAR_PLANNER_HPP_
