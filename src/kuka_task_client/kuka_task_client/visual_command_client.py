import rclpy
from rclpy.node import Node
from kuka_control_box_srvs.srv import KukaTransformInput
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import numpy as np
from scipy.spatial.transform import Rotation as R

class VisualCommandClient(Node):

    def __init__(self):
        super().__init__('visual_command_client')
        self.cli = self.create_client(KukaTransformInput, 'kuka_transform_input')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = KukaTransformInput.Request()

        # TF listener 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self, source_frame, target_frame, timeout=10.0):
        # 주어진 시간 동안 TF 프레임이 준비될 때까지 대기
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            try:
                transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
                self.get_logger().info(f"Transform from {source_frame} to {target_frame}: {transform}")
                return transform
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().info(f'Waiting for transform between {source_frame} and {target_frame}...')
                rclpy.spin_once(self, timeout_sec=0.5)  # 0.5초 대기 후 다시 시도

        self.get_logger().error(f"Timeout exceeded while waiting for transform from {source_frame} to {target_frame}.")
        return None

    def send_request(self):
        # base_link에서 tag1까지의 변환 얻기
        T_base_tag1 = self.get_transform('base_link', 'tag1')
        # link_6에서 drill_link까지의 변환 얻기
        T_end_effector_drill = self.get_transform('link_6', 'drill_link')

        if T_base_tag1 is None or T_end_effector_drill is None:
            self.get_logger().error('Failed to get necessary transforms, exiting.')
            return None

        # TransformStamped에서 위치와 회전을 추출하여 4x4 변환 행렬을 만듭니다.
        # quaternion 변환 부분 수정
        def transform_to_matrix(transform):
            translation = transform.transform.translation
            q = [transform.transform.rotation.x,
                 transform.transform.rotation.y,
                 transform.transform.rotation.z,
                 transform.transform.rotation.w]
            
            # Rotation matrix를 Quaternion에서 변환
            q_normalized = q / np.linalg.norm(q)  # 쿼터니언 정규화
            rot_matrix = R.from_quat([q_normalized[0], q_normalized[1], q_normalized[2], q_normalized[3]]).as_matrix()

            # Transform 행렬 생성
            trans_matrix = np.eye(4)
            trans_matrix[:3, 3] = [translation.x, translation.y, translation.z]
            trans_matrix[:3, :3] = rot_matrix

            return trans_matrix

        # 변환 행렬로 변환
        T_base_tag1_mat = transform_to_matrix(T_base_tag1)
        print("T_base_tag1_mat:\n", T_base_tag1_mat)  # 변환 행렬 출력

        T_end_effector_drill_mat = transform_to_matrix(T_end_effector_drill)
        print("T_end_effector_drill_mat:\n", T_end_effector_drill_mat)  # 변환 행렬 출력

        # 목표 위치: tag1에서 x축 방향으로 -300mm 이동
        T_target_drill_mat = T_base_tag1_mat.copy()
        T_target_drill_mat[0, 3] -= 0.3  # x축 방향으로 -500mm 이동
        print("T_target_drill_mat (tag1 offset):\n", T_target_drill_mat)  # 변환 행렬 출력

        # end-effector_link의 새로운 위치 계산
        T_base_end_effector_new_mat = T_target_drill_mat @ np.linalg.inv(T_end_effector_drill_mat)
        print("T_base_end_effector_new_mat:\n", T_base_end_effector_new_mat)  # 변환 행렬 출력

        # TransformStamped 메시지 생성
        transform_msg = TransformStamped()
        
        # 현재 시간을 헤더에 설정
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = "base_link"

        # 변환 행렬을 translation과 rotation으로 분해하여 메시지에 할당
        transform_msg.transform.translation.x = T_base_end_effector_new_mat[0, 3]
        transform_msg.transform.translation.y = T_base_end_effector_new_mat[1, 3]
        transform_msg.transform.translation.z = T_base_end_effector_new_mat[2, 3]

        rotation_matrix = T_base_end_effector_new_mat[:3, :3]
        quat = R.from_matrix(rotation_matrix).as_quat()
        transform_msg.transform.rotation.x = quat[0]
        transform_msg.transform.rotation.y = quat[1]
        transform_msg.transform.rotation.z = quat[2]
        transform_msg.transform.rotation.w = quat[3]

        # 요청 메시지의 target_transform에 TransformStamped 할당
        self.req.target_transform = transform_msg

        # 비동기 서비스 호출
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        result = self.future.result()

        return result

def main(args=None):
    rclpy.init(args=args)

    visual_command_client = VisualCommandClient()
    response = visual_command_client.send_request()

    if response and response.success:
        visual_command_client.get_logger().info('Task executed successfully')
    else:
        visual_command_client.get_logger().warn('Task execution failed')

    visual_command_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
