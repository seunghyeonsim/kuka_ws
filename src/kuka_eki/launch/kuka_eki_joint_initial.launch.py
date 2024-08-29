from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # kuka_eki_joint_server 노드 실행
    kuka_joint_server_node = Node(
        package='kuka_eki',  # 패키지 이름
        executable='kuka_eki_joint_server',  # 실행할 노드
        name='kuka_eki_joint_server',  # 노드의 이름 (옵션)
        output='screen'  # 출력 옵션 (화면에 출력)
    )

    # kuka_joint_server 실행 후 서비스 호출 명령어
    service_call_command = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/kuka_joint',
            'kuka_control_box_srvs/srv/KukaJoint',
            "{a1: 0.0, a2: -90.0, a3: 90.0, a4: 0.0, a5: 90.0, a6: 0.0}"
        ],
        output='screen'
    )

    # kuka_joint_server 노드 시작 시 서비스 호출을 등록하는 이벤트 핸들러
    event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=kuka_joint_server_node,
            on_start=[service_call_command]
        )
    )

    return LaunchDescription([
        kuka_joint_server_node,
        event_handler
    ])
