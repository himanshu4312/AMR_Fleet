#   !/usr/bin/env python3

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
    pkg_amr_description_bringup = get_package_share_directory('amr_description_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        pkg_amr_description, 'urdf', 'amr_body.urdf.xacro.xml')
    
    rviz_config_path = os.path.join(
        pkg_amr_description_bringup, 'rviz', 'rviz_config_1.rviz')
    
    world_path = os.path.join(
        pkg_amr_description, 'world', 'maze.sdf')
    
    gazebo_bridge_config_path = os.path.join(
        pkg_amr_description_bringup, 'config', 'gazebo_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
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
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amr',          # was 'autobot'
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ]
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': gazebo_bridge_config_path,
            'use_sim_time': use_sim_time,
        }]
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
        robot_state_publisher_node,
        gazebo,
        spawn_robot_node,
        gz_bridge_node,
        rviz_node,
    ])