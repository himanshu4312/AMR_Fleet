#!/usr/bin/env python3
"""
Combined launch file for the amr_description package.

Starts:
  - robot_state_publisher   (parses the xacro -> robot_description -> TF)
  - Ignition Gazebo Fortress (loads world/maze.sdf)
  - spawn (ros_gz_sim create) -> spawns the robot from /robot_description into Gazebo
  - a /clock bridge           (keeps RViz/TF in sync with simulation time)
  - rviz2                    (loads rviz/amr_urdf_config.rviz)

Equivalent to the original amr_gazebo XML launch file, rewritten in
Python launch syntax so it matches the .launch.py extension.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        pkg_amr_description, 'urdf', 'amr_body.urdf.xacro.xml')
    rviz_config_path = os.path.join(
        pkg_amr_description, 'rviz', 'rviz_config_1.rviz')
    world_path = os.path.join(
        pkg_amr_description, 'world', 'maze.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Namespace / TF prefix / Gazebo model name for this robot instance'
    )
    declare_x_arg = DeclareLaunchArgument(
        'x', default_value='0.0', description='Initial X spawn pose (meters)'
    )
    declare_y_arg = DeclareLaunchArgument(
        'y', default_value='0.0', description='Initial Y spawn pose (meters)'
    )
    declare_yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Initial yaw spawn pose (radians)'
    )

    # xacro output must be wrapped in ParameterValue(..., value_type=str),
    # otherwise robot_state_publisher can throw a YAML/type parsing error.
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=', robot_name]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_path} -r'}.items()
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_name,
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', '0.0',
            '-Y', yaw,
        ]
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_name_arg,
        declare_x_arg,
        declare_y_arg,
        declare_yaw_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_robot_node,
        clock_bridge_node,
        rviz_node,
    ])
