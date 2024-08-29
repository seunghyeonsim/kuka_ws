import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # kuka_description 패키지의 경로 가져오기
    kuka_description_dir = get_package_share_directory('kuka_description')
    # kuka_controller 패키지의 경로 가져오기
    kuka_controller_dir = get_package_share_directory('kuka_controller')

    # gazebo_rviz.launch 포함
    gazebo_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kuka_description_dir, 'launch', 'gazebo_rviz.launch.py')
        )
    )

    # controller.launch.py 포함
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kuka_controller_dir, 'launch', 'controller.launch.py')
        )
    )

    # kuka_ik_service_server 노드 추가
    ik_service_server_node = Node(
        package='kuka_control_box',
        executable='kuka_ik_service_server',
        name='kuka_ik_service_server',
        output='screen'
    )

    return LaunchDescription([
        gazebo_rviz_launch,
        controller_launch,
        ik_service_server_node,
    ])

