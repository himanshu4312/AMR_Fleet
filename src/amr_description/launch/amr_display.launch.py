from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('amr_description'),
                             'urdf', 'amr_body.urdf.xacro.xml')
    rviz_config_path = os.path.join(get_package_share_path('amr_description'),
                                    'rviz', 'rviz_config_1.rviz')

    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Namespace / TF prefix for this robot instance'
    )
    declare_x_arg = DeclareLaunchArgument(
        'x', default_value='0.0', description='Initial X pose (unused for display-only, kept for API parity with the spawn launch)'
    )
    declare_y_arg = DeclareLaunchArgument(
        'y', default_value='0.0', description='Initial Y pose (unused for display-only, kept for API parity with the spawn launch)'
    )
    declare_yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Initial yaw pose (unused for display-only, kept for API parity with the spawn launch)'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=', robot_name]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace=robot_name,
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        declare_robot_name_arg,
        declare_x_arg,
        declare_y_arg,
        declare_yaw_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
