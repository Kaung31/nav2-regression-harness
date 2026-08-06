import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('harness_description')
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'empty_room.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')

    robot_desc = xacro.process_file(xacro_file).toxml()

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v2',
             '--headless-rendering', world_file],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'harness_bot',
                   '-x', '0', '-y', '0', '-z', '0.15'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg,
                     'use_sim_time': True}],
        output='screen'
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765, 'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz,
        rsp,
        bridge,
        foxglove,
        TimerAction(period=8.0, actions=[spawn]),
    ])