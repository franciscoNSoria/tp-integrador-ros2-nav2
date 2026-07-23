#!/usr/bin/env python3

from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# TF2
from tf2_ros import Buffer, TransformListener, TransformException

# Acciones de Nav2
from nav2_msgs.action import ComputePathToPose, FollowPath


class NavigationState(Enum):
    IDLE = 0
    COMPUTE_PATH = 1
    WAITING_FOR_PATH = 2
    FOLLOW_PATH = 3
    WAITING_FOR_EXECUTION = 4
    CANCELLING = 5


class Orchestrator(Node):

    def __init__(self):
        super().__init__('orchestrator')

        # Parámetros configurables
        self.declare_parameter('replanning_frequency', 1.0)  # Hz
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.replanning_freq = self.get_parameter('replanning_frequency').value
        self.planner_id = self.get_parameter('planner_id').value
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Control de Estado interno
        self.current_state = NavigationState.IDLE
        self.target_pose = None

        # Handles para cancelar las metas activas de las acciones
        self.planner_goal_handle = None
        self.controller_goal_handle = None

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Clients
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.controller_client = ActionClient(self, FollowPath, 'follow_path')

        # Suscripción a RViz /goal_pose
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Timer principal para control y replanificación
        timer_period = 1.0 / self.replanning_freq
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("Orquestador de Navegación en Python inicializado.")

    # --- CALLBACKS & CONTROL DE ESTADO ---

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Nueva meta recibida desde RViz (X: {msg.pose.position.x:.2f}, Y: {msg.pose.position.y:.2f})"
        )
        self.target_pose = msg

        # Si ya hay una navegación en curso, cancelamos lo activo primero
        if self.current_state != NavigationState.IDLE:
            self.get_logger().warn("Cancelando navegación anterior para procesar la nueva meta...")
            self.cancel_active_actions()
            self.current_state = NavigationState.CANCELLING
        else:
            self.current_state = NavigationState.COMPUTE_PATH

    def control_loop(self):
        # Si se están cancelando metas anteriores
        if self.current_state == NavigationState.CANCELLING:
            if self.planner_goal_handle is None and self.controller_goal_handle is None:
                self.get_logger().info("Acciones canceladas. Iniciando nueva meta.")
                self.current_state = NavigationState.COMPUTE_PATH
            return

        # Replanificación periódica si ya estamos ejecutando
        if self.current_state == NavigationState.WAITING_FOR_EXECUTION:
            self.get_logger().debug("Ejecutando replanificación periódica...")
            self.request_path()
            return

        # Inicio de cálculo de ruta
        if self.current_state == NavigationState.COMPUTE_PATH:
            self.request_path()

    # --- TF2: OBTERNER POSE ACTUAL ---

    def get_current_pose(self) -> PoseStamped:
        try:
            # lookup_transform(target_frame, source_frame, time)
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            current_pose = PoseStamped()
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.header.frame_id = self.global_frame
            current_pose.pose.position.x = tf.transform.translation.x
            current_pose.pose.position.y = tf.transform.translation.y
            current_pose.pose.position.z = tf.transform.translation.z
            current_pose.pose.orientation = tf.transform.rotation
            return current_pose

        except TransformException as ex:
            self.get_logger().error(f"Error consultando TF ({self.global_frame} -> {self.base_frame}): {ex}")
            return None

    # --- PLANNER (ComputePathToPose) ---

    def request_path(self):
        if not self.planner_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Fallo: El servidor ComputePathToPose no está disponible.")
            self.current_state = NavigationState.IDLE
            return

        start_pose = self.get_current_pose()
        if start_pose is None:
            self.get_logger().error("Fallo en etapa: TF / Obtención de Pose Actual.")
            self.current_state = NavigationState.IDLE
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = self.target_pose
        goal_msg.planner_id = self.planner_id

        if self.current_state == NavigationState.COMPUTE_PATH:
            self.current_state = NavigationState.WAITING_FOR_PATH

        send_goal_future = self.planner_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.planner_goal_response_callback)

    def planner_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("El servidor de Planner rechazó el Goal.")
            self.current_state = NavigationState.IDLE
            return

        self.planner_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.planner_result_callback)

    def planner_result_callback(self, future):
        self.planner_goal_handle = None
        result = future.result()

        # result.status representa el estado de finalización de la acción
        if result.status != 4:  # STATUS_SUCCEEDED = 4 en rclpy
            self.get_logger().error("Navegación Fallida en etapa: PLANNER (ComputePathToPose).")
            self.current_state = NavigationState.IDLE
            return

        path = result.result.path
        self.get_logger().info(f"Path calculado con éxito. Puntos: {len(path.poses)}")
        self.send_path_to_controller(path)

    # --- CONTROLLER (FollowPath) ---

    def send_path_to_controller(self, path: Path):
        if not self.controller_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Fallo: El servidor FollowPath no está disponible.")
            self.current_state = NavigationState.IDLE
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id

        self.current_state = NavigationState.WAITING_FOR_EXECUTION

        send_goal_future = self.controller_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.controller_goal_response_callback)

    def controller_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("El servidor del Controller rechazó el Goal.")
            self.current_state = NavigationState.IDLE
            return

        self.controller_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.controller_result_callback)

    def controller_result_callback(self, future):
        self.controller_goal_handle = None
        result = future.result()

        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info("¡ÉXITO! Meta alcanzada correctamente.")
            self.current_state = NavigationState.IDLE
        elif status == 5:  # CANCELED
            self.get_logger().info("Controlador cancelado (Replanificación o nueva meta).")
        elif status == 6:  # ABORTED
            self.get_logger().error("Navegación Fallida en etapa: CONTROLLER (Obstáculo o fuera de trayecto).")
            self.current_state = NavigationState.IDLE
        else:
            self.get_logger().error("Navegación Fallida en etapa: CONTROLLER (Error desconocido).")
            self.current_state = NavigationState.IDLE

    # --- CANCELACIÓN DE ACCIONES ---

    def cancel_active_actions(self):
        if self.planner_goal_handle is not None:
            self.planner_goal_handle.cancel_goal_async()
            self.planner_goal_handle = None

        if self.controller_goal_handle is not None:
            self.controller_goal_handle.cancel_goal_async()
            self.controller_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    orchestrator = Orchestrator()
    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()