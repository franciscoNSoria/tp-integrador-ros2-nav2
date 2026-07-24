from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    orquestador = Node(
        package='my_robot_navigation',
        executable='orquestador_node',
        name='orquestador_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([orquestador])
