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

#ifndef PLANNER_TEST_UTILS_HPP_
#define PLANNER_TEST_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "amr_planner_plugins/costmap_cost_utils.hpp"

// Shared helpers for building a headless Costmap2DROS (no Gazebo/Nav2
// bringup needed) and for independently verifying planner output, reused
// across the GTest suite and the benchmark harness.
namespace amr_planner_plugins_test
{

struct CostmapOptions
{
  int width_m = 10;
  int height_m = 10;
  double resolution = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  bool track_unknown_space = false;
};

inline std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeTestCostmap(
  const std::string & node_name, const CostmapOptions & opts = CostmapOptions())
{
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(node_name);

  // No layer plugins: the master grid is only ever touched by the test, so
  // hand-placed obstacles can't be overwritten by a layer update.
  costmap_ros->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  costmap_ros->set_parameter(rclcpp::Parameter("global_frame", std::string("map")));
  costmap_ros->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_link")));
  costmap_ros->set_parameter(rclcpp::Parameter("width", opts.width_m));
  costmap_ros->set_parameter(rclcpp::Parameter("height", opts.height_m));
  costmap_ros->set_parameter(rclcpp::Parameter("resolution", opts.resolution));
  costmap_ros->set_parameter(rclcpp::Parameter("origin_x", opts.origin_x));
  costmap_ros->set_parameter(rclcpp::Parameter("origin_y", opts.origin_y));
  costmap_ros->set_parameter(rclcpp::Parameter("rolling_window", false));
  costmap_ros->set_parameter(rclcpp::Parameter("track_unknown_space", opts.track_unknown_space));
  costmap_ros->set_parameter(rclcpp::Parameter("use_sim_time", false));

  // Deliberately not calling on_activate(): the GlobalPlanner interface only
  // needs getCostmap()/getGlobalFrameID(), both available after configure.
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  return costmap_ros;
}

inline geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

// Straight-line collision check sampled at half the costmap resolution,
// using the exact same "what's blocked" rule the planners themselves use
// (amr_planner_plugins::isLethalCost) — this lets tests verify a path
// independently, without reaching into any planner's private methods.
inline bool segmentIsCollisionFree(
  nav2_costmap_2d::Costmap2D * costmap, double x0, double y0, double x1, double y1,
  bool allow_unknown)
{
  double dist = std::hypot(x1 - x0, y1 - y0);
  double check_step = costmap->getResolution() * 0.5;
  int steps = std::max(1, static_cast<int>(std::ceil(dist / check_step)));

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(steps);
    double x = x0 + t * (x1 - x0);
    double y = y0 + t * (y1 - y0);
    unsigned int mx, my;
    if (!costmap->worldToMap(x, y, mx, my)) {
      return false;
    }
    if (amr_planner_plugins::isLethalCost(costmap->getCost(mx, my), allow_unknown)) {
      return false;
    }
  }
  return true;
}

inline bool pathIsCollisionFree(
  nav2_costmap_2d::Costmap2D * costmap, const nav_msgs::msg::Path & path, bool allow_unknown)
{
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    if (!segmentIsCollisionFree(costmap, a.x, a.y, b.x, b.y, allow_unknown)) {
      return false;
    }
  }
  return true;
}

// Closed-form minimum cost of an 8-connected path between two grid cells on
// a free costmap (straight step = resolution, diagonal step =
// resolution*sqrt(2)) — the same formula AStarPlanner uses as its heuristic.
// Used to verify A*/Dijkstra return a truly optimal path, not just "a" path.
inline double octileDistance(int dx, int dy, double resolution)
{
  dx = std::abs(dx);
  dy = std::abs(dy);
  return resolution * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
}

inline double pathLength(const nav_msgs::msg::Path & path)
{
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

// Vertical wall of lethal cells at world-x `wall_x`, leaving gaps below
// `gap_before` and above `height_m - gap_after` (in meters) so a path around
// it must exist.
inline void addWallWithGaps(
  nav2_costmap_2d::Costmap2D * costmap, double wall_x, double gap_before, double gap_after)
{
  unsigned int wall_mx, dummy_my;
  if (!costmap->worldToMap(wall_x, 0.0, wall_mx, dummy_my)) {
    return;
  }
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    double wx, wy;
    costmap->mapToWorld(wall_mx, my, wx, wy);
    if (wy > gap_before && wy < (costmap->getSizeInMetersY() - gap_after)) {
      costmap->setCost(wall_mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

// Seals a rectangular ring of lethal cells around a world point so nothing
// outside it can reach the (still-free) center — used for the
// goal-unreachable test, distinct from the goal-cell-itself-lethal test.
inline void sealBoxAround(
  nav2_costmap_2d::Costmap2D * costmap, double cx, double cy, double half_size)
{
  unsigned int mx0, my0, mx1, my1;
  if (!costmap->worldToMap(cx - half_size, cy - half_size, mx0, my0)) {return;}
  if (!costmap->worldToMap(cx + half_size, cy + half_size, mx1, my1)) {return;}
  for (unsigned int mx = mx0; mx <= mx1; ++mx) {
    for (unsigned int my = my0; my <= my1; ++my) {
      if (mx == mx0 || mx == mx1 || my == my0 || my == my1) {
        costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

}  // namespace amr_planner_plugins_test

#endif  // PLANNER_TEST_UTILS_HPP_
