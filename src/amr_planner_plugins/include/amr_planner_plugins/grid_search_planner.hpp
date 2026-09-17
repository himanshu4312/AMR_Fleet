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

#ifndef AMR_PLANNER_PLUGINS__GRID_SEARCH_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__GRID_SEARCH_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace amr_planner_plugins
{

// Shared 8-connected grid-search planner: holds the priority-queue search
// loop, costmap bookkeeping, and Nav2 lifecycle plumbing common to any
// planner that differs only in how it weighs "distance still to go".
// Subclasses (AStarPlanner, DijkstraPlanner) supply the heuristic.
//
// This class, together with RRTPlanner, is this package's Strategy pattern:
// nav2_core::GlobalPlanner is the Strategy interface Nav2 itself defines,
// and AStarPlanner / DijkstraPlanner / RRTPlanner are three interchangeable
// concrete strategies behind it — planner_server picks one per `planner_id`
// at runtime (see nav2_params.yaml) with no code on either side aware of
// which concrete algorithm it's talking to.
class GridSearchPlanner : public nav2_core::GlobalPlanner
{
public:
  GridSearchPlanner() = default;
  ~GridSearchPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  struct CellIndex
  {
    int x;
    int y;
    bool operator==(const CellIndex & other) const {return x == other.x && y == other.y;}
  };

  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex & idx) const
    {
      return (static_cast<std::size_t>(idx.x) << 16) ^ static_cast<std::size_t>(idx.y);
    }
  };

  struct OpenSetEntry
  {
    CellIndex index;
    double f_cost;
  };

  struct OpenSetCompare
  {
    bool operator()(const OpenSetEntry & a, const OpenSetEntry & b) const
    {
      return a.f_cost > b.f_cost;  // min-heap: smallest f_cost served first
    }
  };

  // Distance-to-go estimate from a cell to the goal, in the same units as
  // traversalCost(). Return 0.0 everywhere to turn this into Dijkstra.
  virtual double heuristic(const CellIndex & a, const CellIndex & b) const = 0;

  // Used only in log messages so runtime output identifies which concrete
  // planner is running.
  virtual std::string plannerTypeName() const = 0;

  bool searchPath(
    const CellIndex & start_cell, const CellIndex & goal_cell,
    std::vector<CellIndex> & result_path);

  double traversalCost(const CellIndex & from, const CellIndex & to) const;
  std::vector<CellIndex> neighbors(const CellIndex & idx) const;
  bool inBounds(const CellIndex & idx) const;
  bool isLethal(const CellIndex & idx) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("GridSearchPlanner")};
  std::string global_frame_, name_;

  bool allow_unknown_{true};
  double cost_weight_{0.8};  // scales inflated costmap cost into edge weight
};

}  // namespace amr_planner_plugins

#endif  // AMR_PLANNER_PLUGINS__GRID_SEARCH_PLANNER_HPP_
