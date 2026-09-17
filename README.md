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
- Custom interfaces in `amr_fleet_msgs` will carry the path/status
  information between the robots and the fleet manager.

This is being built in stages, committing a clean, buildable workspace at
the end of each stage.

## Workspace layout

```
AMR_Fleet/
├── src/
│   ├── amr_description/            # robot URDF/meshes/RViz config (from AMR_Robot repo)
│   ├── amr_description_bringup/    # launch files, maps, bringup config (from AMR_Robot repo)
│   ├── amr_planner_plugins/        # custom Nav2 planner plugin (from AMR_Robot repo)
│   ├── amr_fleet_msgs/             # custom interfaces (msg/srv) shared across the fleet
│   └── amr_fleet_manager/          # central fleet coordinator node (rclpy)
├── build/, install/, log/          # colcon-generated, not hand-edited
```

## Status: what has been done (Step 1 — workspace & package scaffolding)

- Created the `AMR_Fleet` colcon workspace (`src/` layout).
- Pulled `amr_description`, `amr_description_bringup`, and
  `amr_planner_plugins` in unmodified from
  [AMR_Robot](https://github.com/himanshu4312/AMR_Robot) into `src/`.
- Created `amr_fleet_msgs`: an `ament_cmake` package wired for
  `rosidl_generate_interfaces()`, with an empty `msg/` folder. The
  `rosidl_generate_interfaces()` call is guarded (`if(msg_files)`) since the
  macro errors out on zero interface files — it will start generating
  automatically once `.msg`/`.srv` files are added.
- Created `amr_fleet_manager`: an `ament_python` (rclpy) package with a
  single placeholder node, `fleet_manager_node`, that starts up and logs
  `"fleet_manager alive"` on a 1-second timer. No coordination logic yet.
- Verified: `rosdep install --from-paths src --ignore-src -r -y` and
  `colcon build` both succeed with all 5 packages building cleanly, and the
  placeholder node runs and logs correctly.

No namespacing, no multi-robot launch files, and no coordinator logic exist
yet — deliberately deferred to later steps.

## What's required to do next

- [ ] **Namespacing**: parametrize `amr_description` / `amr_description_bringup`
      launch files so each robot instance can be spun up under its own
      namespace (e.g. `robot1`, `robot2`, ...) with unique TF frames and
      topics.
- [ ] **Multi-robot Gazebo + Nav2 launch**: a launch file that spawns 3-4
      robots into one shared Gazebo world, each with its own Nav2 stack
      (map server can be shared, but planner/controller/localization must be
      per-robot).
- [ ] **Interface definitions**: design and add the actual `.msg`/`.srv`
      files to `amr_fleet_msgs` (e.g. a per-robot planned-path + live-pose
      status message, and any request/response needed to pause/resume a
      robot).
- [ ] **Fleet manager logic**:
  - Subscribe to every robot's planned path and live position.
  - Detect predicted path crossings (two robots' paths intersecting at
    close-enough times).
  - Resolve conflicts by priority (pause the robot farther from its own
    goal) and resume it once the conflict clears.
- [ ] **Integration testing**: multi-robot scenarios with deliberately
      crossing goals to validate conflict detection and pause/resume
      behavior.

## Build instructions

```bash
cd ~/AMR_Fleet
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run the placeholder fleet manager node:

```bash
ros2 run amr_fleet_manager fleet_manager_node
```
