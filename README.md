# AMR_Fleet

Multi-robot autonomous mobile robot (AMR) simulation on ROS 2 Humble. The end
goal is 3-4 differential-drive robots, each running its own Nav2 stack,
navigating independently to separate goals inside one shared Gazebo world,
coordinated by a central **fleet manager** node.

## Project working (end goal)

- Each robot runs its own full Nav2 navigation stack (planner, controller,
  localization) inside its own namespace, all inside one Gazebo world.
- A central `amr_fleet_manager` node subscribes to every robot's planned path
  and live position.
- It detects when two robots' planned paths are about to cross at
  close-enough times (a predicted conflict), and pauses the lower-priority
  robot (the one farther from its own goal) until the conflict clears.
- Custom interfaces in `amr_fleet_msgs` carry status information about the
  fleet manager's view of each robot for observability.

This is being built in stages, committing a clean, buildable workspace at
the end of each stage.

## Workspace layout

```
AMR_Fleet/
├── src/
│   ├── amr_description/            # robot URDF/xacro/meshes/world, namespace-parameterized
│   │   ├── urdf/                   # robot_name xacro arg threads through every link/joint/topic
│   │   ├── world/maze.sdf          # shared maze world (Sensors system plugin lives here, world-level)
│   │   └── launch/                 # single-robot display/spawn launch files
│   ├── amr_description_bringup/    # launch files, maps, nav2/slam config
│   │   ├── launch/
│   │   │   ├── amr_bringup.launch.py       # single-robot bringup (unmodified, still works as-is)
│   │   │   ├── nav2_bringup.launch.py      # single-robot Nav2 only (unmodified)
│   │   │   ├── slam.launch.py              # SLAM mapping (unmodified)
│   │   │   └── multi_robot_bringup.launch.py  # two robots, one world, two Nav2 stacks, RViz, TF relay
│   │   ├── rviz/multi_robot_rviz_config.rviz  # both robots + both plans + per-robot goal/pose-estimate buttons
│   │   ├── scripts/tf_merge_relay.py          # mirrors both robots' TF onto shared /tf for RViz
│   │   └── config/nav2_params.yaml            # shared Nav2 params (per-robot frames/topics rewritten at launch)
│   ├── amr_planner_plugins/        # custom Nav2 planner plugins (unmodified)
│   ├── amr_fleet_msgs/             # custom interfaces shared across the fleet
│   │   └── msg/FleetRobotState.msg # fleet manager's per-robot belief, observability only
│   └── amr_fleet_manager/          # central fleet coordinator node (rclpy)
│       └── amr_fleet_manager/fleet_manager_node.py  # conflict detection + pause/resume logic
├── build/, install/, log/          # colcon-generated, not hand-edited, not committed
```

## Status: what has been done

### Step 1 — workspace & package scaffolding

- Created the `AMR_Fleet` colcon workspace (`src/` layout).
- Pulled `amr_description`, `amr_description_bringup`, and
  `amr_planner_plugins` in unmodified from
  [AMR_Robot](https://github.com/himanshu4312/AMR_Robot) into `src/`.
- Created `amr_fleet_msgs` (`ament_cmake`, empty `msg/`) and
  `amr_fleet_manager` (`ament_python`, placeholder node only). No
  coordination logic yet.

### Step 2 — multi-robot plumbing (namespacing, dual Nav2 stacks, validated)

Proved two robots can run fully independent Nav2 stacks in one shared world
before any fleet-coordination logic was written. `amr_fleet_msgs` and
`amr_fleet_manager` were **not** touched in this step.

- **Namespaced URDF**: `amr_description`'s xacro takes a `robot_name`
  argument that prefixes every link, joint, TF frame, and Gazebo plugin topic
  (`scan`, `cmd_vel`, `odom`, `tf`). Default is empty, so the original
  single-robot launch files (`amr_bringup.launch.py`, `nav2_bringup.launch.py`,
  `slam.launch.py`) are completely untouched and behave exactly as before.
- **`multi_robot_bringup.launch.py`**: one Gazebo (Ignition Fortress)
  instance, two robots (`robot1` at `(-12.2, 9.95)`, `robot2` at
  `(2.65, -13.3)` — picked from the map's clear space, ~32m apart), each with
  its own namespaced `robot_state_publisher`, `ros_gz_bridge`, AMCL instance,
  and full Nav2 stack (plain nodes, not composed — Humble has a known bug
  where composable costmap nodes break namespace propagation). One shared
  `map_server` for both robots.
- **RViz visualization**: a multi-robot RViz config (`RobotModel` + colored
  global-plan `Path` per robot) plus a small `tf_merge_relay` node so
  generic tools that only know the plain global `/tf` (RViz, `rqt_tf_tree`)
  can see both robots' frames — Nav2/AMCL themselves keep using each robot's
  own namespaced TF topic directly and are unaffected.
- **Bugs found and fixed along the way** (all affect correctness, not just
  multi-robot mode):
  - `gz-sim-sensors-system` was declared per-robot in the xacro; two copies
    fighting over one Gazebo rendering scene segfaulted the simulator. Moved
    to `maze.sdf` (world-level, loaded once).
  - `RewrittenYaml` calls were missing `root_key=<robot_name>`, so every
    per-robot AMCL/Nav2 parameter override was silently ignored (AMCL fell
    back to non-existent `base_footprint`/`odom` frames).
  - `nav2_costmap_2d`'s `StaticLayer` resolves `map_topic` to an absolute
    namespaced path internally, bypassing normal remap-rule resolution —
    needed an explicit `map_topic` parameter, not just a `SetRemap`.
  - `FollowPath.min_vel_x: 0.2` in `nav2_params.yaml` prevented the DWB
    controller from ever generating a pure in-place rotation, so robots
    would stall trying to align final heading right next to the goal.
    Changed to `0.0` (this fix also applies to single-robot mode).
- **Validated**: both robots localize correctly at their spawn poses via
  AMCL, and independently completed `navigate_to_pose` goals to different
  points in the maze with no TF/topic cross-talk and zero crashes.

### Step 3 — fleet manager: conflict detection and pause/resume (validated)

Gives `amr_fleet_manager` real logic, using the two robots from Step 2.
`amr_description`, `amr_description_bringup`, and `amr_planner_plugins` were
**not** touched in this step.

**Principle**: the fleet manager doesn't need robots to tell it anything
special — it reads what Nav2 already publishes for each robot
(`/<robot>/plan` and `/<robot>/amcl_pose`) and predicts, ahead of time,
whether two robots' remaining paths will bring them close together *at
around the same time*. Spatial closeness alone is not enough to call it a
conflict — two paths can cross the same point in the maze many seconds
apart with no real risk, and flagging that as a conflict would make the
system pause robots constantly in a maze full of corridor intersections.

**Method** (`amr_fleet_manager/fleet_manager_node.py`):

- Subscribes directly to each robot's native `nav_msgs/Path` (`/<robot>/plan`)
  and `geometry_msgs/PoseWithCovarianceStamped` (`/<robot>/amcl_pose`) — no
  custom message carries path or pose data.
- On a 2 Hz timer, for every pair of robots, checks every pair of path
  segments (one from each robot's remaining path) using a proper
  segment-to-segment closest-distance algorithm (not just endpoint checks,
  which would miss real crossings). A pair only counts as a conflict if
  **both**:
  - the closest distance is under `safety_distance_m` (default **1.0 m** =
    2× the 0.22 m costmap footprint radius + margin), **and**
  - each robot's estimated arrival time at that point (cumulative path
    distance ÷ `nominal_speed_mps`, default **0.26 m/s** = `max_vel_x` from
    `nav2_params.yaml`) is within `time_window_s` (default **7.0 s**) of the
    other's.
- **Priority**: whichever robot has the shorter total remaining path keeps
  moving; the other is paused. This decision is locked in once per conflict
  *episode* (tracked in a dict keyed by robot pair) and is **not**
  re-evaluated every tick while the episode stays active — only when the
  conflict condition stops holding does the paused robot resume.
- **Pause/resume**: publishes `nav2_msgs/msg/SpeedLimit` to
  `/<robot>/speed_limit` (which `controller_server` already subscribes to).
  Per that message's own definition, `speed_limit=0.0` (with
  `percentage=true`) means *no limit* — so pausing uses `speed_limit=1.0`
  (1% of max speed, effectively stopped), and resuming uses `0.0`. Getting
  this backwards was flagged as an easy mistake and is called out explicitly
  in the code at the publish site.
- Publishes one `FleetRobotState` per robot on `/fleet_manager/robot_states`
  every tick, purely for observability (nothing subscribes to it).

**Validated**: relaunched `multi_robot_bringup.launch.py`, started
`fleet_manager_node`, then sent each robot to the *other's* original spawn
point (a full diagonal swap across the maze, guaranteeing their paths
cross):
- `robot1`: `(-12.2, 9.95)` → `(2.65, -13.3)`
- `robot2`: `(2.65, -13.3)` → `(-12.2, 9.95)`

Both robots reached their goals. Verified end to end: `controller_server`
was confirmed as the actual subscriber on `/robot1/speed_limit`; a pause was
confirmed real (not just a log line) by checking `/robot1/odom` velocity
dropped to ~0 during an active episode; both `bt_navigator`s logged `Goal
succeeded`.

**What did *not* go as expected, and why**: instead of one clean
pause → pass → resume cycle, the conflict fired **21 separate short episodes**
(1-3 seconds each) in quick succession, almost all pausing the same robot
(`robot1`, 20 of 21). Root cause: sending both robots to a full swap means
they travel through much of the *same* corridor system in opposite
directions for an extended stretch (closer to a head-on encounter than a
single perpendicular crossing), and Nav2 replans roughly once a second,
slightly reshaping each path every time — enough to flip the spatial+time
condition in and out of threshold repeatedly rather than settling into one
sustained episode. There's also a self-reinforcing effect: once `robot1`
loses an episode and stalls, its remaining path becomes relatively longer,
making it more likely to lose the *next* episode too. This was not unsafe
(actual separation stayed well clear throughout, and the predictive design
worked as intended — it triggered while they were still ~3.5 m apart, well
before any real close encounter) but is a real tuning gap between the
current thresholds and sustained parallel-corridor travel, as opposed to a
single clean crossing. No threshold or hysteresis changes have been made to
address this yet.

**RViz**: added a second **"2D Goal Pose"** and **"2D Pose Estimate"** button
pair to the multi-robot RViz config, one bound to each robot's namespace
(`/robot1/goal_pose` + `/robot1/initialpose`, `/robot2/goal_pose` +
`/robot2/initialpose`), confirmed both are correctly received by each
robot's `bt_navigator`/`amcl`. The pose-estimate buttons are not needed for
normal runs (AMCL is auto-seeded with the correct spawn pose at launch) —
only useful if a robot's localization ever visibly drifts.

## What's required to do next

- [ ] Investigate/tune the conflict-flickering behavior found in Step 3
      (e.g. hysteresis so a cleared conflict doesn't immediately re-open, or
      a minimum episode duration) - only if it turns out to matter in
      practice.
- [ ] Scale from 2 to 3-4 robots once the above is settled.
- [ ] Anything beyond observability for `FleetRobotState` (currently nothing
      subscribes to it) if a use for it emerges.

## Build instructions

```bash
cd ~/AMR_Fleet
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running it

### 1. Launch the simulation

```bash
ros2 launch amr_description_bringup multi_robot_bringup.launch.py
```

Useful launch arguments:

```bash
# no Gazebo GUI (server + offscreen sensor rendering only) - use if two
# robots' worth of Nav2 + rendering is too heavy to run in real time
ros2 launch amr_description_bringup multi_robot_bringup.launch.py headless:=true

# skip RViz entirely
ros2 launch amr_description_bringup multi_robot_bringup.launch.py use_rviz:=false
```

Watch the terminal for five separate `Managed nodes are active` lines
(map_server, robot1 localization, robot1 navigation, robot2 localization,
robot2 navigation) before doing anything else.

If Gazebo is already running without RViz, attach RViz to it separately:

```bash
ros2 run rviz2 rviz2 -d install/amr_description_bringup/share/amr_description_bringup/rviz/multi_robot_rviz_config.rviz --ros-args -p use_sim_time:=true
```

### 2. Start the fleet manager (separate terminal)

```bash
ros2 run amr_fleet_manager fleet_manager_node
```

Watch this terminal for `Conflict detected between ... : pausing ...` /
`Conflict cleared ...` lines while robots are navigating - this is the main
signal that coordination is actually working.

### 3. Send goals

**Via RViz**: click the **"2D Goal Pose (robot1)"** or
**"2D Goal Pose (robot2)"** toolbar button, then click-and-drag on the map at
the target position/heading. Each button only ever targets its own robot;
send one at a time, one robot at a time.

**Via CLI** (run concurrently so both robots move at once):

```bash
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.65, y: -13.3, z: 0.0}, orientation: {z: -0.4805, w: 0.8770}}}}"
```

```bash
ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -12.2, y: 9.95, z: 0.0}, orientation: {z: 0.8770, w: 0.4805}}}}"
```

(the pair above is a full spawn-to-spawn swap, guaranteed to cross paths -
useful specifically for testing conflict detection)

### 4. Checks along the way

```bash
# each robot's localized pose
ros2 topic echo /robot1/amcl_pose --once
ros2 topic echo /robot2/amcl_pose --once

# confirm the fleet manager is actually wired to controller_server
ros2 topic info /robot1/speed_limit -v

# confirm a pause is real, not just a log line (check during an active episode)
ros2 topic echo /robot1/odom --once   # twist.twist.linear.x should be ~0 if paused

# fleet manager's observability view of both robots
ros2 topic echo /fleet_manager/robot_states
```

### Single robot (unchanged from Step 1)

```bash
ros2 launch amr_description_bringup amr_bringup.launch.py
```
