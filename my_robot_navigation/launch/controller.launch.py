import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav_dir = get_package_share_directory('my_robot_navigation')
    # el controller_server arma su propio local_costmap interno
    
    costmap_params = os.path.join(nav_dir, 'config', 'costmap.yaml')
    controller_params = os.path.join(nav_dir, 'config', 'controller.yaml')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[costmap_params, controller_params],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server'],
            # sin bond_timeout: 0.0 a proposito: controller_server si implementa bond
        }],
        output='screen',
    )

    return LaunchDescription([controller_server, lifecycle_manager])
