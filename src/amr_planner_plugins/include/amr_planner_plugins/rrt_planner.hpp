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

#ifndef AMR_PLANNER_PLUGINS__RRT_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__RRT_PLANNER_HPP_

#include <memory>
#include <random>
#include <string>
#include <vector>

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

// RRT Strategy: goal-biased Rapidly-exploring Random Tree over continuous
// world coordinates, not a grid search — this is why it can't share
// GridSearchPlanner's search loop the way AStarPlanner/DijkstraPlanner do.
// Each iteration samples a random point (or the goal itself, with
// probability goal_bias_, to pull the tree toward it instead of relying on
// pure chance), finds the nearest existing tree node, steers from it toward
// the sample by at most step_size_, and accepts the new node only if the
// straight-line segment to it is collision-free. Deliberately no RRT*
// rewiring and no path smoothing — this returns the first valid path found,
// which can be visibly jagged; that's a known, accepted tradeoff for keeping
// the implementation simple and easy to verify first.
class RRTPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTPlanner() = default;
  ~RRTPlanner() override = default;

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

private:
  struct TreeNode
  {
    double x;
    double y;
    int parent;  // index into the tree, -1 for the root (start)
  };

  bool isStateValid(double x, double y) const;
  bool isSegmentFree(double x0, double y0, double x1, double y1) const;
  // Plain O(n) linear scan — see the design notes for why this is an
  // accepted tradeoff rather than a kd-tree.
  int nearestNode(const std::vector<TreeNode> & tree, double x, double y) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RRTPlanner")};
  std::string global_frame_, name_;

  bool allow_unknown_{true};
  int max_iterations_{5000};
  double step_size_{0.3};
  double goal_tolerance_{0.3};
  double goal_bias_{0.1};
  double planning_timeout_{1.0};  // seconds — Humble's GlobalPlanner has no cancel callback
  // -1 (default) seeds from std::random_device for real randomness in
  // production; set to a fixed value to make a run reproducible, which is
  // how the unit tests get deterministic RRT behavior in CI.
  int rng_seed_{-1};

  mutable std::mt19937 rng_;
};

}  // namespace amr_planner_plugins

#endif  // AMR_PLANNER_PLUGINS__RRT_PLANNER_HPP_
