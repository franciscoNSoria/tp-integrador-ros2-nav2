import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from geometry_msgs.msg import PoseStamped

#ComputePathToPose es el tipo de mensaje de acción propio de Nav2
from nav2_msgs.action import ComputePathToPose
#FollowPath es el tipo mensaje de acción para conectarse con el controller_server (generar los /cmd_vel para mover robt)
from nav2_msgs.action import  FollowPath

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

        # parametro para poder elegir entre DWB o RPP en runtime sin recompilar
        self.declare_parameter('controller_id', 'RPP')
        self.controller_id = self.get_parameter('controller_id').value

        self.state = 'IDLE'
        self.current_goal = None
        self.active_follow_goal_handle = None

        self.declare_parameter('replan_frequency', 1.0)
        replan_freq = self.get_parameter('replan_frequency').value
        self.replan_timer = self.create_timer(1.0 / replan_freq, self.replan_callback)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.get_logger().info('Orquestador esperando la meta de /goal_pose...')

    def get_robot_pose(self):
        ##Devuelve la pose actual del robot o None si no hay TF todavia
        try:

            # Esto devuelve un TransformStamped con la x,y del robot
            return self.tf_buffer.lookup_transform('map', 'base_link', Time())
        
        except tf2_ros.TransformException as ex:
            # Caso de falla: todavia no hay TF 
            self.get_logger().warn(f'No se pudo leer TF map->base_link: {ex}')
            return None

    #solicita un path al planner_server para la meta recibida
    def request_path(self, goal_pose: PoseStamped, is_replan=False):

        if not is_replan:
            self.get_logger().info('Pidiendo path al planner...')

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.planner_id = 'GridBased'
        send_future = self.planner_client.send_goal_async(goal)

        #Cuando el planner_server  responde ejecuta plan_goal_response
        send_future.add_done_callback(
            lambda future: self.plan_goal_response(future, is_replan))

    def plan_goal_response(self, future, is_replan=False):
        goal_handle = future.result()
        #Revisa si el servidor rechazo la solicitud
        if not goal_handle.accepted:
            self.get_logger().error('El planner rechazo la meta')
            if not is_replan:
                self.state = 'IDLE'
            return
        #Si aprobo la solicitud,pide el resultado final del calculo y la asigna funcion de callback(result)
        goal_handle.get_result_async().add_done_callback(
            lambda future: self.plan_result(future, is_replan))

    def plan_result(self, future, is_replan=False):
        result = future.result()
        path = result.result.path

        #Comprueba segun el estandar de ros(4 STATUS_SUCCEEDED) y que el path no este vacio
        if result.status != 4 or len(path.poses) == 0:
            if is_replan:
                self.get_logger().warn('Replanificacion fallo, se mantiene el path actual.')
            else:
                self.get_logger().error(
                    f'compute_path_to_pose fallo: status={result.status}, '
                    f'error_code={result.result.error_code}. No hay path para seguir.')
                self.state = 'IDLE'
            return

        if is_replan:
            self.get_logger().info(f'Path actualizado con {len(path.poses)} poses (replanificacion).')
        else:
            self.get_logger().info(f'Path recibido con {len(path.poses)} poses.')

        #entrega la lista de poses que se envía al controlador
        self.follow_path(path)


    #envia el path calculado al controller_server para que el robot lo siga
    def follow_path(self, path):
        self.state = 'FOLLOWING'

        self.get_logger().info(f'Mandando a follow_path (controller_id={self.controller_id})...')
        goal = FollowPath.Goal()
        goal.path = path
        
        #goal_checker_id Le indica a Nav2 que criterio usar para considerar que el robot llego

        goal.controller_id = self.controller_id
        goal.goal_checker_id = 'general_goal_checker'
        send_future = self.controller_client.send_goal_async(
            goal, feedback_callback=self.follow_feedback)
        send_future.add_done_callback(self.follow_goal_response)

    def follow_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El controller rechazo el path')
            return
        self.active_follow_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda future: self.follow_result(future, goal_handle))

    #Mientras el robot conduce, el controlador publica periódicamente retroalimentación.
    def follow_feedback(self, feedback_msg):
        self.get_logger().info(
            #captura y muestra en consola la distancia que le falta al robot para llegar a la meta
            f'distance_to_goal={feedback_msg.feedback.distance_to_goal:.2f} m')

    def follow_result(self, future, goal_handle):
        if goal_handle is not self.active_follow_goal_handle:
            return

        result = future.result()
        if result.status == 4:
            self.get_logger().info('Meta alcanzada, follow_path exitoso.')
            self.state = 'IDLE'
        else:
            self.get_logger().error(f'follow_path fallo: status={result.status}')
            self.state = 'IDLE'

    def replan_callback(self):
        if self.state != 'FOLLOWING':
            return
        self.get_logger().info('Replanificando...')
        self.request_path(self.current_goal, is_replan=True)

    #Disparador , Quien recibe la Meta
    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg
        self.state = 'PLANNING'
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
