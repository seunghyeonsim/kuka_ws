import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from kuka_control_box_srvs.srv import KukaJoint
from kuka_eki.eki import EkiMotionClient, EkiStateClient
from kuka_eki.krl import Axis
import numpy as np

# 전역 변수 선언
joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "end_effector_joint"]
joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0]  # 초기 값 설정

class KukaEkiJointServer(Node):

    def __init__(self):
        super().__init__('kuka_eki_joint_server')
        self.joint_service = self.create_service(KukaJoint, 'kuka_joint', self.handle_joint_service)
        self.eki_motion_client = EkiMotionClient("172.31.1.147")
        self.eki_state_client = EkiStateClient("172.31.1.147")
        
        # Publisher 생성
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # 주기적으로 joint 상태를 퍼블리시하기 위한 타이머 생성
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 0.1초마다 퍼블리시

        # 서비스 서버가 준비되었다는 메시지 출력
        self.get_logger().info("Service server is ready and waiting for requests.")
        
        try:
            self.eki_motion_client.connect()
            self.eki_state_client.connect()
        except Exception as e:
            self.get_logger().error(f"로봇 연결 실패: {e}")
            raise RuntimeError("로봇 연결에 실패했습니다.")
        


    def handle_joint_service(self, request, response):
        global joint_positions  # 전역 변수 사용 선언

        try:
            # 요청된 목표 위치를 설정
            target_axis = Axis(request.a1, request.a2, request.a3, request.a4, request.a5, request.a6)
            self.eki_motion_client.ptp(target_axis, 0.5)

            while True:
                state = self.eki_state_client.state()
                
                # 현재 로봇 상태를 확인
                current_axis = np.array([state.axis.a1, state.axis.a2, state.axis.a3, state.axis.a4, state.axis.a5, state.axis.a6], dtype=float)
                target_axis_array = np.array([request.a1, request.a2, request.a3, request.a4, request.a5, request.a6])

                # 문자열인 경우 float로 변환하여 출력
                pos_x = float(state.pos.x)
                pos_y = float(state.pos.y)
                pos_z = float(state.pos.z)
                pos_a = float(state.pos.a)
                pos_b = float(state.pos.b)
                pos_c = float(state.pos.c)
                pos_s = float(state.pos.s)
                pos_t = float(state.pos.t)

                # 로봇 상태를 로그로 출력
                self.get_logger().info(f"현재 로봇 상태: Axis - {current_axis.tolist()}, Position - ({pos_x:.4f}, {pos_y:.4f}, {pos_z:.4f}, "
                                       f"{pos_a:.4f}, {pos_b:.4f}, {pos_c:.4f}, {pos_s:.4f}, {pos_t:.4f})")

                # 각 joint 위치 업데이트 (모든 값을 float로 변환)
                joint_positions[:6] = [float(val) for val in current_axis.tolist()]
                joint_positions[6] = 0.0  # end_effector_joint는 항상 0으로 설정

                # 상태가 유효할 때만 퍼블리시
                if np.linalg.norm(current_axis) > 0.001:
                    self.publish_joint_states()

                # 목표 위치와 현재 위치의 차이가 0.1 이하이면 루프 탈출
                if np.linalg.norm(current_axis - target_axis_array) < 0.1:
                    break

            # 업데이트된 joint 위치 출력
            for joint_name, joint_position in zip(joint_names, joint_positions):
                self.get_logger().info(f"{joint_name}: {joint_position:.4f}")

            # 응답 설정
            response.success = True
            response.a1 = float(state.axis.a1)
            response.a2 = float(state.axis.a2)
            response.a3 = float(state.axis.a3)
            response.a4 = float(state.axis.a4)
            response.a5 = float(state.axis.a5)
            response.a6 = float(state.axis.a6)
            response.x = pos_x
            response.y = pos_y
            response.z = pos_z
            response.a = pos_a
            response.b = pos_b
            response.c = pos_c
            response.s = pos_s
            response.t = pos_t

        except Exception as e:
            self.get_logger().error(f"서비스 처리 중 오류 발생: {e}")
            response.success = False  # 오류 발생 시 응답이 실패를 나타내도록 설정

        return response


    def publish_joint_states(self):
        # joint_positions의 각 값을 최신 상태로 설정 (서비스에서 이미 설정됨)
        joint_positions[6] = 0.0  # end_effector_joint는 항상 0으로 설정

        # 각도를 라디안으로 변환하여 퍼블리시
        radians_positions = [np.deg2rad(pos) for pos in joint_positions]  # 각도를 라디안으로 변환

        # JointState 메시지를 생성하고 퍼블리시
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 설정
        msg.name = joint_names
        msg.position = radians_positions

        self.publisher.publish(msg)  # 메시지 퍼블리시

def main(args=None):
    rclpy.init(args=args)
    try:
        node = KukaEkiJointServer()
        rclpy.spin(node)
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
