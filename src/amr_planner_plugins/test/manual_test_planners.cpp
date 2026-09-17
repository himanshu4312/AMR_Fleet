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

// Throwaway standalone test harness (not a gtest) for amr_planner_plugins.
// Builds a small real Costmap2DROS (zero layers, so nothing but us writes to
// the costmap), hand-adds a wall obstacle, and calls each planner directly.
// Run with: ./build/amr_planner_plugins/manual_test_planners

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "amr_planner_plugins/astar_planner.hpp"
#include "amr_planner_plugins/dijkstra_planner.hpp"
#include "amr_planner_plugins/rrt_planner.hpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> buildTestCostmap()
{
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  costmap_ros->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  costmap_ros->set_parameter(rclcpp::Parameter("global_frame", std::string("map")));
  costmap_ros->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_link")));
  costmap_ros->set_parameter(rclcpp::Parameter("width", 10));
  costmap_ros->set_parameter(rclcpp::Parameter("height", 10));
  costmap_ros->set_parameter(rclcpp::Parameter("resolution", 0.1));
  costmap_ros->set_parameter(rclcpp::Parameter("origin_x", 0.0));
  costmap_ros->set_parameter(rclcpp::Parameter("origin_y", 0.0));
  costmap_ros->set_parameter(rclcpp::Parameter("rolling_window", false));
  costmap_ros->set_parameter(rclcpp::Parameter("track_unknown_space", false));
  costmap_ros->set_parameter(rclcpp::Parameter("use_sim_time", false));

  costmap_ros->on_configure(rclcpp_lifecycle::State());

  return costmap_ros;
}

void addWallWithGaps(nav2_costmap_2d::Costmap2D * costmap)
{
  unsigned int wall_mx = 50;
  for (unsigned int my = 20; my < 80; ++my) {
    costmap->setCost(wall_mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

void printPath(const std::string & planner_name, const nav_msgs::msg::Path & path)
{
  std::cout << "=== " << planner_name << " ===\n";
  std::cout << "poses: " << path.poses.size() << "\n";
  std::cout << std::fixed << std::setprecision(4);
  for (const auto & pose : path.poses) {
    std::cout << "  (" << pose.pose.position.x << ", " << pose.pose.position.y << ")\n";
  }
  std::cout << std::endl;
}

void printFailure(const std::string & planner_name, const std::exception & e)
{
  std::cout << "=== " << planner_name << " ===\n";
  std::cout << "FAILED: " << e.what() << "\n" << std::endl;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto costmap_ros = buildTestCostmap();
  addWallWithGaps(costmap_ros->getCostmap());

  auto start = makePose(1.0, 5.0);
  auto goal = makePose(9.0, 5.0);

  {
    amr_planner_plugins::AStarPlanner astar;
    astar.configure(
      std::weak_ptr<rclcpp_lifecycle::LifecycleNode>(costmap_ros), "GridBased",
      costmap_ros->getTfBuffer(), costmap_ros);
    astar.activate();
    try {
      auto path = astar.createPlan(start, goal);
      printPath("AStarPlanner (GridBased)", path);
    } catch (const std::exception & e) {
      printFailure("AStarPlanner (GridBased)", e);
    }
    astar.deactivate();
    astar.cleanup();
  }

  {
    amr_planner_plugins::DijkstraPlanner dijkstra;
    dijkstra.configure(
      std::weak_ptr<rclcpp_lifecycle::LifecycleNode>(costmap_ros), "DijkstraGridBased",
      costmap_ros->getTfBuffer(), costmap_ros);
    dijkstra.activate();
    try {
      auto path = dijkstra.createPlan(start, goal);
      printPath("DijkstraPlanner (DijkstraGridBased)", path);
    } catch (const std::exception & e) {
      printFailure("DijkstraPlanner (DijkstraGridBased)", e);
    }
    dijkstra.deactivate();
    dijkstra.cleanup();
  }

  {
    amr_planner_plugins::RRTPlanner rrt;
    rrt.configure(
      std::weak_ptr<rclcpp_lifecycle::LifecycleNode>(costmap_ros), "RRTGlobal",
      costmap_ros->getTfBuffer(), costmap_ros);
    rrt.activate();
    try {
      auto path = rrt.createPlan(start, goal);
      printPath("RRTPlanner (RRTGlobal)", path);
    } catch (const std::exception & e) {
      printFailure("RRTPlanner (RRTGlobal)", e);
    }
    rrt.deactivate();
    rrt.cleanup();
  }

  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
  return 0;
}
