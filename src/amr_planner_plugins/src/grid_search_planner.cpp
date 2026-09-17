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

#include "amr_planner_plugins/grid_search_planner.hpp"

#include <cmath>
#include <algorithm>

#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "amr_planner_plugins/costmap_cost_utils.hpp"

namespace amr_planner_plugins
{

void GridSearchPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_weight", rclcpp::ParameterValue(0.8));
  node->get_parameter(name + ".cost_weight", cost_weight_);

  RCLCPP_INFO(
    logger_, "Configured %s '%s' (allow_unknown=%s, cost_weight=%.2f)",
    plannerTypeName().c_str(), name_.c_str(), allow_unknown_ ? "true" : "false", cost_weight_);
}

void GridSearchPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s '%s'", plannerTypeName().c_str(), name_.c_str());
}
void GridSearchPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s '%s'", plannerTypeName().c_str(), name_.c_str());
}
void GridSearchPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s '%s'", plannerTypeName().c_str(), name_.c_str());
}

nav_msgs::msg::Path GridSearchPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  unsigned int start_mx, start_my, goal_mx, goal_my;

  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    throw nav2_core::PlannerException("Start pose is outside the costmap bounds.");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    throw nav2_core::PlannerException("Goal pose is outside the costmap bounds.");
  }

  CellIndex start_cell{static_cast<int>(start_mx), static_cast<int>(start_my)};
  CellIndex goal_cell{static_cast<int>(goal_mx), static_cast<int>(goal_my)};

  if (isLethal(goal_cell)) {
    throw nav2_core::PlannerException("Goal cell is occupied by a lethal obstacle.");
  }

  std::vector<CellIndex> cell_path;

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  bool found = searchPath(start_cell, goal_cell, cell_path);
  lock.unlock();

  if (!found) {
    throw nav2_core::PlannerException("No valid path could be found between start and goal.");
  }

  path.poses.reserve(cell_path.size());
  for (const auto & cell : cell_path) {
    double wx, wy;
    costmap_->mapToWorld(cell.x, cell.y, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }

  return path;
}

// Best-first grid search over an 8-connected costmap grid. This is the one
// piece of code AStarPlanner and DijkstraPlanner share: both push cells into
// `open_set` ordered by g_cost (accumulated cost from start) + heuristic(),
// and both accept the first pop of the goal cell as optimal — that's only
// true because heuristic() is required to be admissible (never overestimates
// the true remaining cost) and consistent (h(a) <= cost(a,b) + h(b) for every
// edge a->b). AStarPlanner's octile-distance heuristic satisfies both;
// DijkstraPlanner's heuristic() returning a constant 0 trivially satisfies
// both too, which is exactly what turns this into plain Dijkstra: with no
// lookahead, cells are expanded strictly in order of distance from the
// start, so it explores a growing "circle" instead of A*'s goal-directed
// cone, but is still provably optimal for the same reason.
bool GridSearchPlanner::searchPath(
  const CellIndex & start_cell, const CellIndex & goal_cell,
  std::vector<CellIndex> & result_path)
{
  std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, OpenSetCompare> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_cost;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  g_cost[start_cell] = 0.0;
  open_set.push({start_cell, heuristic(start_cell, goal_cell)});

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (closed[current]) {continue;}  // stale entry, cheaper path already processed it
    closed[current] = true;

    if (current == goal_cell) {
      result_path.clear();
      CellIndex step = current;
      result_path.push_back(step);
      while (!(step == start_cell)) {
        step = came_from.at(step);
        result_path.push_back(step);
      }
      std::reverse(result_path.begin(), result_path.end());
      return true;
    }

    for (const auto & next : neighbors(current)) {
      if (closed[next]) {continue;}

      double tentative_g = g_cost[current] + traversalCost(current, next);
      auto it = g_cost.find(next);
      if (it == g_cost.end() || tentative_g < it->second) {
        g_cost[next] = tentative_g;
        came_from[next] = current;
        open_set.push({next, tentative_g + heuristic(next, goal_cell)});
      }
    }
  }
  return false;
}

double GridSearchPlanner::traversalCost(const CellIndex & from, const CellIndex & to) const
{
  double res = costmap_->getResolution();
  bool diagonal = (from.x != to.x) && (from.y != to.y);
  double base_cost = diagonal ? res * std::sqrt(2.0) : res;

  unsigned char cost_value = costmap_->getCost(
    static_cast<unsigned int>(to.x), static_cast<unsigned int>(to.y));
  double normalized_cost = static_cast<double>(cost_value) / 255.0;

  // Folding inflation cost into edge weight biases the search toward corridor
  // centers instead of skimming obstacle edges.
  return base_cost * (1.0 + cost_weight_ * normalized_cost);
}

std::vector<GridSearchPlanner::CellIndex> GridSearchPlanner::neighbors(const CellIndex & idx) const
{
  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  std::vector<CellIndex> result;
  result.reserve(8);
  for (int i = 0; i < 8; ++i) {
    CellIndex next{idx.x + dx[i], idx.y + dy[i]};
    if (inBounds(next) && !isLethal(next)) {result.push_back(next);}
  }
  return result;
}

bool GridSearchPlanner::inBounds(const CellIndex & idx) const
{
  return idx.x >= 0 && idx.y >= 0 &&
         static_cast<unsigned int>(idx.x) < costmap_->getSizeInCellsX() &&
         static_cast<unsigned int>(idx.y) < costmap_->getSizeInCellsY();
}

bool GridSearchPlanner::isLethal(const CellIndex & idx) const
{
  unsigned char cost = costmap_->getCost(
    static_cast<unsigned int>(idx.x), static_cast<unsigned int>(idx.y));
  return isLethalCost(cost, allow_unknown_);
}

}  // namespace amr_planner_plugins
