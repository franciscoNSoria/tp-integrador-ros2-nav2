import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    nav_dir = get_package_share_directory('my_robot_navigation')

    planner = IncludeLaunchDescription(
        os.path.join(nav_dir, 'launch', 'planner.launch.py'))

    controller = IncludeLaunchDescription(
        os.path.join(nav_dir, 'launch', 'controller.launch.py'))

    return LaunchDescription([planner, controller])
