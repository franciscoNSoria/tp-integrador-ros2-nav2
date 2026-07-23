import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from geometry_msgs.msg import PoseStamped

#ComputePathToPose es el tipo de mensaje de acción propio de Nav2
from nav2_msgs.action import ComputePathToPose, FollowPath


class OrquestadorNode(Node):

    def __init__(self):
        super().__init__('orquestador_node')

        # ros2 ubica al robot en un arbol de frames
        # TransformListener se subscribe a /tf y lo manda al buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Crea el action client idem a test flow pero aca el nodo lo guarda como atribut lo que permite levantar varios goal repetidamente
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        #bloquea el arranque del nodo hasta que el planner_server este disponible
        self.planner_client.wait_for_server()

        self.controller_client = ActionClient(self, FollowPath, 'follow_path')
        self.controller_client.wait_for_server()

        # parametro para poder elegir DWB o RPP en runtime sin recompilar
        # (ros2 param set /orquestador_node controller_id DWB, o via launch)
        self.declare_parameter('controller_id', 'RPP')
        self.controller_id = self.get_parameter('controller_id').value

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.get_logger().info('Orquestador esperando la meta de /goal_pose...')

    def get_robot_pose(self):
        """Devuelve la pose actual del robot (map -> base_link) o None si no hay TF todavia."""
        try:
            # Esto devuelve un TransformStamped con la x,y del robot
            return self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException as ex:
            # Caso de falla: todavia no hay TF (SLAM/AMCL no publico map todavia, etc)
            self.get_logger().warn(f'No se pudo leer TF map->base_link: {ex}')
            return None

    #solicita un path al planner_server para la meta recibida
    def request_path(self, goal_pose: PoseStamped):
        self.get_logger().info('Pidiendo path al planner...')
        
        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.planner_id = 'GridBased'
        send_future = self.planner_client.send_goal_async(goal)
        
        #Cuando el planner_server  responde ejecuta plan_goal_response
        send_future.add_done_callback(self.plan_goal_response)

    def plan_goal_response(self, future):
        goal_handle = future.result()
        #Revisa si el servidor rechazo la solicitud 
        if not goal_handle.accepted:
            self.get_logger().error('El planner rechazo la meta')
            return
        #Si aprobo la solicitud,pide el resultado final del calculo y la asigna funcion de callback(result)
        goal_handle.get_result_async().add_done_callback(self.plan_result)

    def plan_result(self, future):
        result = future.result()
        path = result.result.path
        #Comprueba segun el estandar de ros(4 STATUS_SUCCEEDED) y que el path no este vacio
        if result.status != 4 or len(path.poses) == 0: 
            self.get_logger().error(
                f'compute_path_to_pose fallo: status={result.status}, '
                f'error_code={result.result.error_code}. No hay path para seguir.')
            return
        self.get_logger().info(f'Path recibido con {len(path.poses)} poses.')
        self.follow_path(path)

    #envia el path calculado al controller_server para que el robot lo siga
    def follow_path(self, path):
        self.get_logger().info(f'Mandando a follow_path (controller_id={self.controller_id})...')
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = 'general_goal_checker'
        send_future = self.controller_client.send_goal_async(
            goal, feedback_callback=self.follow_feedback)
        send_future.add_done_callback(self.follow_goal_response)

    def follow_feedback(self, feedback_msg):
        self.get_logger().info(
            f'distance_to_goal={feedback_msg.feedback.distance_to_goal:.2f} m')

    def follow_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El controller rechazo el path')
            return
        goal_handle.get_result_async().add_done_callback(self.follow_result)

    def follow_result(self, future):
        result = future.result()
        if result.status == 4:
            self.get_logger().info('Meta alcanzada, follow_path exitoso.')
        else:
            self.get_logger().error(f'follow_path fallo: status={result.status}')

    #Disparador , Quien recibe la Meta
    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg 
        self.get_logger().info(
            f'Meta recibida: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f} (frame={msg.header.frame_id})')

        # Se obtiene aca la pose para ver donde esta el robot actualmente
        robot_tf = self.get_robot_pose()
        if robot_tf is not None:
            t = robot_tf.transform.translation
            self.get_logger().info(f'Pose actual del robot: x={t.x:.2f}, y={t.y:.2f}')

        #llama request_path(msg) enviandole al meta
        self.request_path(msg)


def main():
    rclpy.init()
    node = OrquestadorNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
