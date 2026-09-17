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

// RRT is not optimal by construction (goal-biased random sampling), so
// unlike the grid-search suite, these tests check collision-free
// connectivity and endpoint correctness rather than path cost.
//
// rng_seed is fixed in every success-path test below so CI is deterministic
// — see RRTPlanner::configure()'s handling of the rng_seed parameter.

#include <atomic>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "amr_planner_plugins/rrt_planner.hpp"
#include "planner_test_utils.hpp"

namespace
{
using amr_planner_plugins_test::CostmapOptions;
using amr_planner_plugins_test::makeTestCostmap;
using amr_planner_plugins_test::makePose;

std::atomic<int> g_node_counter{0};

class RRTPlannerTest : public ::testing::Test
{
protected:
  static constexpr const char * kPlannerName = "test_rrt";

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  amr_planner_plugins::RRTPlanner planner_;

  void configurePlanner(
    const CostmapOptions & opts,
    bool allow_unknown = true,
    int max_iterations = 5000,
    double step_size = 0.3,
    double goal_tolerance = 0.3,
    double goal_bias = 0.2,
    double planning_timeout = 2.0,
    int rng_seed = 42)
  {
    costmap_ros_ = makeTestCostmap("rrt_test_costmap_" + std::to_string(g_node_counter++), opts);
    auto declare = [this](const std::string & suffix, auto value) {
        costmap_ros_->declare_parameter(
          std::string(kPlannerName) + "." + suffix, rclcpp::ParameterValue(value));
      };
    declare("allow_unknown", allow_unknown);
    declare("max_iterations", max_iterations);
    declare("step_size", step_size);
    declare("goal_tolerance", goal_tolerance);
    declare("goal_bias", goal_bias);
    declare("planning_timeout", planning_timeout);
    declare("rng_seed", rng_seed);

    planner_.configure(
      std::weak_ptr<rclcpp_lifecycle::LifecycleNode>(costmap_ros_), kPlannerName,
      costmap_ros_->getTfBuffer(), costmap_ros_);
    planner_.activate();
  }

  void TearDown() override
  {
    if (costmap_ros_) {
      planner_.deactivate();
      planner_.cleanup();
      costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
    }
  }

  // Every interior tree edge is bounded by step_size_, and only the final
  // edge connecting to the goal is bounded by goal_tolerance_ instead —
  // this checks that structural invariant directly against the actual
  // returned waypoints, independent of RRTPlanner's internals.
  static void expectValidStepStructure(
    const nav_msgs::msg::Path & path, double step_size, double goal_tolerance)
  {
    ASSERT_GE(path.poses.size(), 1u);
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      double dist = std::hypot(b.x - a.x, b.y - a.y);
      bool is_last_edge = (i + 2 == path.poses.size());
      double bound = is_last_edge ? goal_tolerance : step_size;
      EXPECT_LE(dist, bound + 1e-6)
        << "edge " << i << " length " << dist << " exceeds bound " << bound;
    }
  }
};

TEST_F(RRTPlannerTest, ValidPathOnOpenCostmapIsCollisionFreeAndConnected)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto start = makePose(1.0, 5.0);
  auto goal = makePose(9.0, 5.0);

  nav_msgs::msg::Path path;
  ASSERT_NO_THROW(path = planner_.createPlan(start, goal));
  ASSERT_FALSE(path.poses.empty());

  auto * costmap = costmap_ros_->getCostmap();
  EXPECT_TRUE(amr_planner_plugins_test::pathIsCollisionFree(costmap, path, true));
  expectValidStepStructure(path, 0.3, 0.3);

  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y, start.pose.position.y);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
}

TEST_F(RRTPlannerTest, ValidPathAvoidsWallObstacle)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto * costmap = costmap_ros_->getCostmap();
  amr_planner_plugins_test::addWallWithGaps(costmap, 5.0, 2.0, 2.0);

  auto start = makePose(1.0, 5.0);
  auto goal = makePose(9.0, 5.0);

  nav_msgs::msg::Path path;
  ASSERT_NO_THROW(path = planner_.createPlan(start, goal));
  EXPECT_TRUE(amr_planner_plugins_test::pathIsCollisionFree(costmap, path, true));
}

TEST_F(RRTPlannerTest, ThrowsWhenGoalFullyBlocked)
{
  // Small iteration/time budget: with the goal fully sealed off, RRT should
  // exhaust its budget quickly and fail cleanly rather than search forever.
  configurePlanner(
    CostmapOptions{10, 10, 0.1, 0.0, 0.0, false}, true, /*max_iterations=*/ 500,
    0.3, 0.3, 0.2, /*planning_timeout=*/ 2.0, /*rng_seed=*/ 7);
  auto * costmap = costmap_ros_->getCostmap();
  amr_planner_plugins_test::sealBoxAround(costmap, 5.0, 5.0, 0.3);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(5.0, 5.0);
  EXPECT_THROW(planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TEST_F(RRTPlannerTest, ThrowsWhenGoalCellIsLethal)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto * costmap = costmap_ros_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(5.0, 5.0, mx, my));
  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(5.0, 5.0);
  EXPECT_THROW(planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TEST_F(RRTPlannerTest, ThrowsWhenStartOutsideCostmapBounds)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto start = makePose(-1000.0, -1000.0);
  auto goal = makePose(5.0, 5.0);
  EXPECT_THROW(planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TEST_F(RRTPlannerTest, ThrowsWhenGoalOutsideCostmapBounds)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto start = makePose(5.0, 5.0);
  auto goal = makePose(1000.0, 1000.0);
  EXPECT_THROW(planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TEST_F(RRTPlannerTest, HandlesStartEqualsGoal)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto pose = makePose(5.0, 5.0);

  nav_msgs::msg::Path path;
  ASSERT_NO_THROW(path = planner_.createPlan(pose, pose));
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, pose.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, pose.pose.position.x);
}

TEST_F(RRTPlannerTest, AllUnknownCostmapSucceedsWhenAllowUnknownTrue)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, true}, /*allow_unknown=*/ true);
  auto start = makePose(1.0, 1.0);
  auto goal = makePose(9.0, 9.0);
  EXPECT_NO_THROW(planner_.createPlan(start, goal));
}

TEST_F(RRTPlannerTest, AllUnknownCostmapThrowsWhenAllowUnknownFalse)
{
  configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, true}, /*allow_unknown=*/ false);
  auto start = makePose(1.0, 1.0);
  auto goal = makePose(9.0, 9.0);
  EXPECT_THROW(planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TEST_F(RRTPlannerTest, MinimalOneCellCostmapDoesNotCrash)
{
  configurePlanner(CostmapOptions{1, 1, 1.0, 0.0, 0.0, false});
  auto pose = makePose(0.5, 0.5);
  EXPECT_NO_THROW(planner_.createPlan(pose, pose));
}

}  // namespace
