import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                            DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('harness_description')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'empty_room.yaml')
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'spawn.launch.py'))
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        sim,
        TimerAction(period=25.0, actions=[nav2]),
    ])