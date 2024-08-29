import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 패키지 디렉토리 설정
    kuka_description = get_package_share_directory('kuka_description')
    kuka_description_share = get_package_prefix('kuka_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # URDF 파일 경로 설정
    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        kuka_description, 'urdf', 'kuka.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file'
    )

    # Gazebo 모델 경로 설정
    model_path = os.path.join(kuka_description, "models")
    model_path += pathsep + os.path.join(kuka_description_share, "share")
    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)

    # 로봇 설명 파라미터 설정
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Gazebo 서버 실행
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        )
    )

    # Gazebo 클라이언트 실행
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Gazebo에 로봇 스폰
    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'kuka',
                                   '-topic', 'robot_description',
                                  ],
                        output='screen',
                        parameters=[{'use_sim_time': True}]
    )

    # Rviz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(kuka_description, 'rviz', 'display.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,  # Gazebo 서버 추가
        start_gazebo_client,  # Gazebo 클라이언트 추가
        robot_state_publisher_node,
        spawn_robot,
        rviz_node
    ])
