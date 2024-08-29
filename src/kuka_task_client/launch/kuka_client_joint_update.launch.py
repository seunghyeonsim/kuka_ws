import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    kuka_description_dir = get_package_share_directory('kuka_description')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            kuka_description_dir, 'urdf', 'kuka.urdf.xacro'
        ),
        description='Absolute path to robot urdf file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Client Node 추가
    kuka_client_joint_update_node = Node(
        package='kuka_task_client',
        executable='kuka_client_joint_update',
        name='kuka_client_joint_update',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(kuka_description_dir, 'rviz', 'display.rviz')],
    )

    return LaunchDescription([
        model_arg,
        kuka_client_joint_update_node,  # 추가된 클라이언트 노드
        robot_state_publisher_node,
        rviz_node
    ])
