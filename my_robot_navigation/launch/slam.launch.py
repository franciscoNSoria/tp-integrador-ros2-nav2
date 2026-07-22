import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # --- Rutas: dónde están los archivos que necesitamos ---
    nav_dir = get_package_share_directory('my_robot_navigation')
    bringup_dir = get_package_share_directory('my_robot_bringup')

    slam_params = os.path.join(nav_dir, 'config', 'mapper_params_online_async.yaml')

    # --- Bloque 1: incluir la simulación existente ---
    simulacion = IncludeLaunchDescription(
        os.path.join(bringup_dir, 'launch', 'bringup.launch.xml')
    )

    # --- Bloque 2: el nodo de SLAM ---
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
    )

    # --- Bloque 3: el gestor de ciclo de vida ---
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
        output='screen',
    )

    return LaunchDescription([simulacion, slam, lifecycle_manager])