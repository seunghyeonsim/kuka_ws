import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("kuka_description"),
                    "urdf",
                    "kuka.urdf.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 토픽 발행 명령 설정 
    publish_trajectory_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/arm_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            '{joint_names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "end_effector_joint"], points: [{positions: [0, -1.57, 1.57, 0, 1.57, 0, 0], time_from_start: {sec: 1, nanosec: 0}}]}'
        ],
        output='screen'
    )
    # arm_controller_spawner 노드가 시작된 후 토픽 발행
    event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=arm_controller_spawner,
            on_start=[publish_trajectory_command]
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            event_handler,  # 이벤트 핸들러 등록
        ]
    )
