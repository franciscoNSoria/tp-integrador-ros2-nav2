import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # --- Rutas ---
    nav_dir = get_package_share_directory('my_robot_navigation')
    bringup_dir = get_package_share_directory('my_robot_bringup')

    params_file = os.path.join(nav_dir, 'config', 'localization.yaml')
    maps_dir = os.path.join(nav_dir, 'maps')

    # --- Reescribir el yaml: expandir el nombre del mapa a ruta absoluta ---
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'yaml_filename': os.path.join(maps_dir, 'my_living.yaml')},
        convert_types=True,
    )

    # --- Argumentos ---
    declare_world = DeclareLaunchArgument('world', default_value='my_living.sdf')
    declare_x = DeclareLaunchArgument('x', default_value='1.3044')
    declare_y = DeclareLaunchArgument('y', default_value='-7.2093')

    # --- Bloque 1: la simulación ---
    simulacion = IncludeLaunchDescription(
        os.path.join(bringup_dir, 'launch', 'bringup.launch.xml'),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
        }.items(),
    )

    # --- Bloque 2: map_server (ahora lee del yaml) ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[configured_params],
        output='screen',
    )

    # --- Bloque 3: amcl ---
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[configured_params],
        output='screen',
    )

    # --- Bloque 4: lifecycle manager (administra los dos nodos) ---
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_world, declare_x, declare_y,
        simulacion, map_server, amcl, lifecycle_manager,
    ])