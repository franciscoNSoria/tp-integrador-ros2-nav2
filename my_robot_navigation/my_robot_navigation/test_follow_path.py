import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath


class TestFollowPath(Node):
    """
    Encadena compute_path_to_pose (3b) -> follow_path (3c) para verificar el
    controller_server con un path real, sin depender del bt_navigator.
    Uso: ros2 run my_robot_navigation test_follow_path <x> <y> [controller_id]
    """

    def __init__(self, x, y, controller_id):
        super().__init__('test_follow_path')
        self.controller_id = controller_id
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.controller_client = ActionClient(self, FollowPath, 'follow_path')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.get_logger().info(f'Pidiendo path hacia ({x}, {y})...')
        self.planner_client.wait_for_server()
        send_future = self.planner_client.send_goal_async(
            ComputePathToPose.Goal(goal=goal, planner_id='GridBased'))
        send_future.add_done_callback(self.plan_goal_response)

    def plan_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El planner rechazo el goal')
            return
        goal_handle.get_result_async().add_done_callback(self.plan_result)

    def plan_result(self, future):
        result = future.result()
        path = result.result.path
        if result.status != 4 or len(path.poses) == 0:
            self.get_logger().error(
                f'compute_path_to_pose fallo: status={result.status} '
                f'(4=SUCCEEDED, 5=CANCELED, 6=ABORTED), poses={len(path.poses)}. '
                f'No se manda nada a follow_path. Probablemente el goal esta '
                f'fuera del area ya mapeada, o en una celda ocupada/desconocida.')
            rclpy.shutdown()
            return
        self.get_logger().info(
            f'Path recibido con {len(path.poses)} poses. Mandando a follow_path '
            f'con controller_id={self.controller_id}...')
        self.controller_client.wait_for_server()
        send_future = self.controller_client.send_goal_async(
            FollowPath.Goal(path=path, controller_id=self.controller_id),
            feedback_callback=self.follow_feedback)
        send_future.add_done_callback(self.follow_goal_response)

    def follow_feedback(self, feedback_msg):
        self.get_logger().info(
            f'distance_to_goal={feedback_msg.feedback.distance_to_goal:.2f} m '
            f'speed={feedback_msg.feedback.speed:.2f} m/s')

    def follow_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El controller_server rechazo el goal')
            return
        goal_handle.get_result_async().add_done_callback(self.follow_result)

    def follow_result(self, future):
        self.get_logger().info(f'follow_path termino, status={future.result().status}')
        rclpy.shutdown()


def main():
    if len(sys.argv) < 3:
        print('Uso: ros2 run my_robot_navigation test_follow_path <x> <y> [controller_id]')
        return
    x, y = float(sys.argv[1]), float(sys.argv[2])
    controller_id = sys.argv[3] if len(sys.argv) > 3 else 'RPP'

    rclpy.init()
    node = TestFollowPath(x, y, controller_id)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
