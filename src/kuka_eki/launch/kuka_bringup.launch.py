import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 패키지 경로 설정
    kuka_description_dir = get_package_share_directory('kuka_description')
    realsense_launch_dir = get_package_share_directory('realsense2_camera')
    apriltag_ros_launch_dir = get_package_share_directory('apriltag_ros')

    # 로봇 모델 경로 설정
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

    # 로봇 상태 퍼블리셔 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(kuka_description_dir, 'rviz', 'display.rviz')],
    )

    # Realsense 카메라 런치 파일
    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={'serial_no': '_135122251049'}.items()
    )

    # AprilTag 런치 파일
    apriltag_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apriltag_ros_launch_dir, 'launch', 'apriltag_real_detect.launch.py')
        )
    )

    # KUKA EKI 조인트 서버 노드
    kuka_joint_server_node = Node(
        package='kuka_eki',
        executable='kuka_eki_joint_server',
        name='kuka_eki_joint_server',
        output='screen'
    )

    # 서비스 호출 명령어
    service_call_command = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/kuka_joint',
            'kuka_control_box_srvs/srv/KukaJoint',
            "{a1: 0.0, a2: -90.0, a3: 90.0, a4: 0.0, a5: 90.0, a6: 0.0}"
        ],
        output='screen'
    )

    # KUKA IK 노드
    kuka_ik_node = Node(
        package='kuka_control_box',
        executable='kuka_ik_node',
        name='kuka_ik_node',
        output='screen'
    )

    # 서비스 호출 대기 후 KUKA IK 노드 실행을 위한 타이머 액션
    service_call_timer = TimerAction(
        period=3.0,  # 3초 대기
        actions=[kuka_ik_node]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        rviz_node,
        realsense_camera_launch,
        apriltag_ros_launch,
        kuka_joint_server_node,
        service_call_command,  # 서비스 호출 명령어 실행
        service_call_timer     # 서비스 호출 후 KUKA IK 노드 실행
    ])
