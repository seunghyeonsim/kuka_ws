import rclpy
from rclpy.node import Node
from kuka_control_box_srvs.srv import KukaJoint  # KukaJoint 서비스 메시지 타입 임포트
from sensor_msgs.msg import JointState  # JointState 메시지 타입 임포트
from std_msgs.msg import Header
import threading  # 사용자 입력을 위한 스레드 임포트
import time  # 시간 지연을 위해 사용

class KukaClientJointUpdate(Node):
    def __init__(self):
        super().__init__('kuka_client_joint_update')  # 노드 이름 설정
        self.client = self.create_client(KukaJoint, 'kuka_joint')  # KukaJoint 서비스 클라이언트 생성
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)  # JointState 토픽 퍼블리셔 생성

        # 기본 조인트 상태 설정
        self.current_joint_state = JointState()
        self.current_joint_state.header = Header()
        self.current_joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "end_effector_joint"]
        self.current_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # end_effector_joint를 항상 0으로 설정

        # 타이머를 사용하여 기본 상태 퍼블리시 계속 수행
        self.timer = self.create_timer(1.0, self.publish_initial_state)

        # 사용자 입력을 3초 후에 받을 스레드 실행
        threading.Thread(target=self.delayed_user_input_thread, daemon=True).start()

        # 서비스 준비 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kuka_joint 서비스 준비 중...')

        self.request = KukaJoint.Request()  # 서비스 요청 메시지 생성

    def publish_initial_state(self):
        # 기본 조인트 상태 퍼블리시
        self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.current_joint_state)
        self.get_logger().info('기본 조인트 상태 퍼블리시 중...')

    def delayed_user_input_thread(self):
        # 3초 대기 후 사용자 입력 스레드 시작
        time.sleep(3)
        self.get_logger().info('사용자 입력을 대기 중...')
        self.user_input_thread()

    def user_input_thread(self):
        # 사용자 입력 대기 및 요청 처리
        while rclpy.ok():
            try:
                # 사용자 입력 받기
                desired_joints = input('원하는 조인트 각도를 입력하세요 (예: "0.5 0.5 0.5 0.5 0.5 0.5"): ').split()
                if len(desired_joints) != 6:
                    self.get_logger().info('잘못된 입력입니다. 6개의 값을 입력하세요.')
                    continue

                # 입력받은 각도 값을 서비스 요청 메시지에 할당
                self.request.a1, self.request.a2, self.request.a3, self.request.a4, self.request.a5, self.request.a6 = map(float, desired_joints)
                self.send_request()  # 서비스 요청 전송

            except Exception as e:
                self.get_logger().error(f'입력 처리 중 오류 발생: {e}')

    def send_request(self):
        # 비동기적으로 서비스 요청 전송
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.handle_response)  # 요청 완료 후 콜백 등록
        self.get_logger().info('kuka_joint 서비스 요청 전송됨...')

    def handle_response(self, future):
        try:
            # 서비스 응답 결과 받기
            response = future.result()
            if response.success:
                self.get_logger().info(f'응답 수신 성공: A1={response.a1}, A2={response.a2}, A3={response.a3}, '
                                       f'A4={response.a4}, A5={response.a5}, A6={response.a6}, '
                                       f'X={response.x}, Y={response.y}, Z={response.z}, '
                                       f'A={response.a}, B={response.b}, C={response.c}, '
                                       f'S={response.s}, T={response.t}')

                # 응답을 토대로 JointState 메시지 업데이트
                self.current_joint_state.position = [response.a1, response.a2, response.a3, response.a4, response.a5, response.a6, 0.0]  # end_effector_joint를 항상 0으로 설정
                
                # JointState 메시지 퍼블리시
                self.publish_initial_state()
                self.get_logger().info('현재 조인트 상태 퍼블리시 완료.')

            else:
                self.get_logger().info('응답 수신 실패: 로봇이 목표 위치에 도달하지 못했습니다.')
        except Exception as e:
            self.get_logger().error(f'응답 처리 중 오류 발생: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KukaClientJointUpdate()
    rclpy.spin(node)  # 노드 실행
    rclpy.shutdown()


if __name__ == '__main__':
    main()
