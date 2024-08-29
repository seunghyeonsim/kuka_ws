import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 첫 번째 launch 파일 실행: gazebo_rviz.launch.py
    launch_kuka_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kuka_description'), 'launch', 'gazebo_rviz.launch.py'))
    )

    # 두 번째 launch 파일 실행: controller.launch.py
    launch_kuka_controller = TimerAction(
        period=1.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('kuka_controller'), 'launch', 'controller.launch.py'))
        )]
    )

    # 세 번째 launch 파일 실행: spawn_env.launch.py
    launch_kuka_apriltag = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('kuka_apriltag'), 'launch', 'spawn_env.launch.py'))
        )]
    )

    # 네 번째로 실행할 kuka_ik_service_server 노드
    launch_kuka_ik_service = TimerAction(
        period=1.0,
        actions=[Node(
            package='kuka_control_box',
            executable='kuka_ik_service_server',
            name='kuka_ik_service_server',
            output='screen'
        )]
    )

    # 다섯 번째 launch 파일 실행: apriltag_gazebo_detect.launch.py
    launch_apriltag_gazebo_detect = TimerAction(
        period=1.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('apriltag_ros'), 'launch', 'apriltag_gazebo_detect.launch.py'))
        )]
    )

    return LaunchDescription([
        # 첫 번째 Launch 실행
        launch_kuka_description,

        # 두 번째 Launch는 첫 번째 Launch 이후 1초 대기 후 실행
        launch_kuka_controller,

        # 세 번째 Launch는 두 번째 Launch 이후 1초 대기 후 실행
        launch_kuka_apriltag,

        # 네 번째 Launch는 세 번째 Launch 이후 1초 대기 후 실행
        launch_kuka_ik_service,

        # 다섯 번째 Launch는 네 번째 Launch 이후 1초 대기 후 실행
        launch_apriltag_gazebo_detect,
    ])

if __name__ == '__main__':
    generate_launch_description()
