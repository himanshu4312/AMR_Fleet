#!/usr/bin/env python3
"""
Multi-robot bringup: two independent AMR instances (robot1, robot2) sharing
one Gazebo (Ignition Fortress) world and one static map, each running its own
completely separate Nav2 stack under its own ROS namespace.

There is deliberately NO coordination logic between the two robots at this
stage - each Nav2 stack is unaware the other robot exists. This file only
proves the multi-robot plumbing (namespacing, TF, bridging, per-robot
localization/navigation) works.

Does NOT modify or replace amr_bringup.launch.py, nav2_bringup.launch.py or
slam.launch.py, which remain the single-robot entry points.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

# Two spawn poses inside src/amr_description/world/maze.sdf, chosen by
# scanning maps/maze_map_v3.yaml for cells with >2m clearance from the
# nearest wall/obstacle: robot1 sits in the open room near the top-left of
# the maze, robot2 in the open room near the bottom-right, ~32m apart.
ROBOTS = [
    {'name': 'robot1', 'x': '-12.2', 'y': '9.95', 'yaw': '0.0'},
    {'name': 'robot2', 'x': '2.65', 'y': '-13.3', 'yaw': '0.0'},
]

WORLD_NAME = 'empty'  # <world name='empty'> in maze.sdf


def make_robot_group(robot, urdf_path, nav2_params_path, pkg_nav2_bringup, use_sim_time):
    name = robot['name']
    x, y, yaw = robot['x'], robot['y'], robot['yaw']

    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=', name]),
        value_type=str,
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        remappings=tf_remap,
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=name,
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', name,
            '-x', x,
            '-y', y,
            '-z', '0.0',
            '-Y', yaw,
        ],
    )

    # Each robot's DiffDrive/lidar plugins (amr_gazebo.xacro, lidar.xacro)
    # publish on gz topics named /<robot_name>/scan, /<robot_name>/odom,
    # /<robot_name>/tf and /<robot_name>/cmd_vel (set via the robot_name
    # xacro arg), plus the fixed gz-sim joint_state topic path. Bridge each
    # robot's topics with its own parameter_bridge instance so nothing from
    # robot1 ever reaches robot2's topics.
    gz_joint_state_topic = f'/world/{WORLD_NAME}/model/{name}/joint_state'
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        namespace=name,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'{gz_joint_state_topic}@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[(gz_joint_state_topic, f'/{name}/joint_states')],
    )

    # Per-robot parameter overrides layered on top of the shared
    # nav2_params.yaml: frame ids and topics get the robot's namespace
    # prefix; the "map" frame and map_server stay shared/untouched.
    param_rewrites = {
        # AMCL (amcl.ros__parameters.*)
        'base_frame_id': f'{name}/base_link',
        'odom_frame_id': f'{name}/odom',
        'scan_topic': f'/{name}/scan',
        # shared leaf keys used identically by bt_navigator / costmaps / behavior_server
        'robot_base_frame': f'{name}/base_link',
        'odom_topic': f'/{name}/odom',
        'topic': f'/{name}/scan',
        # local costmap and behavior_server operate in the robot's own odom
        # frame - global costmap and bt_navigator keep the shared "map" frame
        'local_costmap.local_costmap.ros__parameters.global_frame': f'{name}/odom',
        'behavior_server.ros__parameters.global_frame': f'{name}/odom',
        # nav2_costmap_2d's StaticLayer resolves map_topic to an absolute
        # namespaced path internally (e.g. "/robot1/map") before subscribing,
        # bypassing normal remap-rule resolution - a plain SetRemap('map',
        # '/map') has no effect on it. Point it at the shared map_server's
        # "/map" directly via its own parameter instead.
        'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': '/map',
    }

    # root_key=name wraps the rewritten YAML so its top-level keys become
    # "<name>: {amcl: ..., bt_navigator: ...}", matching the fully-qualified
    # "/<name>/amcl" node name. Without this, --params-file's un-namespaced
    # "amcl:" key never matches the namespaced node and every override here
    # is silently ignored (nav2_bringup's own launch files rely on the same
    # root_key=namespace trick for exactly this reason).
    robot_params = RewrittenYaml(
        source_file=nav2_params_path,
        root_key=name,
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=name,
        output='screen',
        parameters=[
            ParameterFile(robot_params, allow_substs=True),
            {
                'use_sim_time': use_sim_time,
                # RewrittenYaml can only override keys that already exist in
                # nav2_params.yaml, and set_initial_pose/initial_pose.* are
                # not in it, so seed AMCL's initial pose here directly -
                # otherwise it starts hunting from (0,0) in the shared map.
                'set_initial_pose': True,
                'initial_pose.x': float(x),
                'initial_pose.y': float(y),
                'initial_pose.z': 0.0,
                'initial_pose.yaw': float(yaw),
            },
        ],
        # amcl's map_topic defaults to relative "map", which under this
        # node's namespace would resolve to "/robot1/map" - but the single
        # shared map_server (see generate_launch_description) publishes on
        # the global, unnamespaced "/map". Remap explicitly so both robots'
        # amcl subscribe to that one shared topic.
        remappings=tf_remap + [('map', '/map')],
    )

    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'],
        }],
    )

    # Plain (non-composed) Nav2 navigation stack: planner_server,
    # controller_server, smoother_server, behavior_server, bt_navigator,
    # waypoint_follower, velocity_smoother + its own lifecycle manager.
    # use_composition is explicitly 'False' - Humble has a known bug where
    # namespace propagation breaks for nested costmap paths under composable
    # container nodes.
    navigation_group = GroupAction([
        PushRosNamespace(name),
        # planner_server's global_costmap static_layer subscribes to the
        # relative "map" topic by default, which under this namespace would
        # resolve to "/robot1/map" - remap it to the shared, unnamespaced
        # map_server's "/map" topic (same reasoning as amcl's map remap
        # above). SetRemap applies to every node launched within this scope,
        # including navigation_launch.py's, which we can't edit directly.
        SetRemap(src='map', dst='/map'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            # namespace is deliberately NOT passed here: navigation_launch.py
            # applies its own root_key=namespace wrap around params_file, and
            # our params_file (robot_params) is already root_key=name wrapped
            # above - passing the same namespace again would double-wrap it
            # (e.g. {robot1: {robot1: {...}}}) and break node-name matching.
            # PushRosNamespace(name) above already namespaces every node here.
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': robot_params,
                'use_composition': 'False',
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])

    return GroupAction([
        rsp_node,
        spawn_node,
        bridge_node,
        amcl_node,
        lifecycle_manager_localization_node,
        navigation_group,
    ])


def generate_launch_description():
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_description_bringup = get_package_share_directory('amr_description_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_path = os.path.join(pkg_amr_description, 'urdf', 'amr_body.urdf.xacro.xml')
    world_path = os.path.join(pkg_amr_description, 'world', 'maze.sdf')
    map_yaml_path = os.path.join(pkg_amr_description_bringup, 'maps', 'maze_map_v3.yaml')
    nav2_params_path = os.path.join(pkg_amr_description_bringup, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    headless = LaunchConfiguration('headless')
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description=(
            'Run Gazebo with -s --headless-rendering (server only, offscreen '
            'sensor rendering, no GUI window). Two robots each running a full '
            'Nav2 stack plus GPU lidar rendering can be heavy on constrained '
            'hardware; headless mode avoids the GUI compositing/render cost.'
        ),
    )

    gz_args_gui = f'{world_path} -r'
    gz_args_headless = f'{world_path} -r -s --headless-rendering'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression([f'"{gz_args_headless}" if "', headless, f'" == "true" else "{gz_args_gui}"']),
        }.items(),
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # ONE shared map_server for both robots, "map" frame is global/shared.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
        }],
    )

    lifecycle_manager_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start a single RViz2 instance showing both robots (map, TF, both global plans)',
    )
    rviz_config_path = os.path.join(pkg_amr_description_bringup, 'rviz', 'multi_robot_rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # RViz's TF/RobotModel displays only listen on the plain, unnamespaced
    # /tf and /tf_static - but each robot's frames live on its own
    # /<robot_name>/tf(_static) topic by design (see make_robot_group). This
    # relay mirrors both robots' TF onto the shared topics purely so generic
    # tools (RViz, rqt_tf_tree, ...) can see everything in one place; Nav2/
    # AMCL keep using the per-robot topics directly and are unaffected.
    # Every frame name is namespace-prefixed, so merging is collision-free.
    tf_merge_relay_node = Node(
        package='amr_description_bringup',
        executable='tf_merge_relay.py',
        name='tf_merge_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    robot_groups = [
        make_robot_group(robot, urdf_path, nav2_params_path, pkg_nav2_bringup, use_sim_time)
        for robot in ROBOTS
    ]

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_headless_arg,
        declare_use_rviz_arg,
        gazebo,
        clock_bridge_node,
        map_server_node,
        lifecycle_manager_map_node,
        rviz_node,
        tf_merge_relay_node,
        *robot_groups,
    ])
