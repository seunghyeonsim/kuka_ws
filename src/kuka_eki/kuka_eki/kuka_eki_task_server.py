import rclpy
from rclpy.node import Node
from kuka_control_box_srvs.srv import KukaTask
from kuka_eki.eki import EkiMotionClient, EkiStateClient
from kuka_eki.krl import Pos
import numpy as np

class KukaEkiTaskServer(Node):

    def __init__(self):
        super().__init__('kuka_eki_task_server')
        self.task_service = self.create_service(KukaTask, 'kuka_task', self.handle_task_service)
        self.eki_motion_client = EkiMotionClient("172.31.1.147")
        self.eki_state_client = EkiStateClient("172.31.1.147")
        self.eki_motion_client.connect()
        self.eki_state_client.connect()

    def handle_task_service(self, request, response):
        target_pos = Pos(request.x, request.y, request.z, request.a, request.b, request.c)
        self.eki_motion_client.ptp(target_pos, 0.5)

        while True:
            state = self.eki_state_client.state()
            # print(f"Current State: Pos({state.pos.x}, {state.pos.y}, {state.pos.z}, {state.pos.a}, {state.pos.b}, {state.pos.c})")  # 디버그 출력 추가
            current_pos = np.array([state.pos.x, state.pos.y, state.pos.z, state.pos.a, state.pos.b, state.pos.c], dtype=float)
            target_pos_array = np.array([request.x, request.y, request.z, request.a, request.b, request.c])
            if np.linalg.norm(current_pos - target_pos_array) < 0.1:
                break

        response.success = True
        response.a1 = float(state.axis.a1)
        response.a2 = float(state.axis.a2)
        response.a3 = float(state.axis.a3)
        response.a4 = float(state.axis.a4)
        response.a5 = float(state.axis.a5)
        response.a6 = float(state.axis.a6)
        response.x = float(state.pos.x)
        response.y = float(state.pos.y)
        response.z = float(state.pos.z)
        response.a = float(state.pos.a)
        response.b = float(state.pos.b)
        response.c = float(state.pos.c)
        response.s = float(state.pos.s)
        response.t = float(state.pos.t)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KukaEkiTaskServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
