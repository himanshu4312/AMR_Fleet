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

#ifndef AMR_PLANNER_PLUGINS__DIJKSTRA_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__DIJKSTRA_PLANNER_HPP_

#include <string>

#include "amr_planner_plugins/grid_search_planner.hpp"

namespace amr_planner_plugins
{

// Dijkstra Strategy: identical GridSearchPlanner search loop to AStarPlanner,
// with the heuristic zeroed out. Deliberately included to demonstrate the
// A*/Dijkstra relationship directly in code, not because it's expected to
// outperform A* — with no goal-directed lookahead it expands a growing
// "circle" of cells around the start instead of a goal-directed cone, so it
// typically visits more cells than A* for the same result.
class DijkstraPlanner : public GridSearchPlanner
{
public:
  DijkstraPlanner() = default;
  ~DijkstraPlanner() override = default;

protected:
  double heuristic(const CellIndex &, const CellIndex &) const override {return 0.0;}
  std::string plannerTypeName() const override {return "DijkstraPlanner";}
};

}  // namespace amr_planner_plugins

#endif  // AMR_PLANNER_PLUGINS__DIJKSTRA_PLANNER_HPP_
