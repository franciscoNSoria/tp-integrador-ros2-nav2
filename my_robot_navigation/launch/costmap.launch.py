import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav_dir = get_package_share_directory('my_robot_navigation')
    costmap_params = os.path.join(nav_dir, 'config', 'costmap.yaml')

    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace='global_costmap',
        parameters=[costmap_params],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['global_costmap/global_costmap'],
        }],
        output='screen',
    )

    return LaunchDescription([global_costmap, lifecycle_manager])