import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

NAV_NODES = ['controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'velocity_smoother', 'bt_navigator',
             'waypoint_follower']


def generate_launch_description():
    pkg = get_package_share_directory('harness_description')

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')
    use_foxglove = LaunchConfiguration('use_foxglove')

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(
            pkg, 'worlds', 'empty_room.sdf')),
        DeclareLaunchArgument('map', default_value=os.path.join(
            pkg, 'maps', 'empty_room.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
    ]

    robot_desc = xacro.process_file(
        os.path.join(pkg, 'urdf', 'robot.urdf.xacro')).toxml()
    sim_time = {'use_sim_time': True}

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v2', '--headless-rendering', world],
        output='screen')

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}, sim_time],
               output='screen')

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': os.path.join(
                      pkg, 'config', 'bridge.yaml')}, sim_time],
                  output='screen')

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', '/robot_description',
                            '-name', 'harness_bot',
                            '-x', '0', '-y', '0', '-z', '0.15'],
                 output='screen')

    foxglove = Node(package='foxglove_bridge', executable='foxglove_bridge',
                    parameters=[{'port': 8765}, sim_time],
                    condition=IfCondition(use_foxglove), output='screen')

    map_server = Node(package='nav2_map_server', executable='map_server',
                      name='map_server',
                      parameters=[params, {'yaml_filename': map_yaml}, sim_time],
                      output='screen')

    amcl = Node(package='nav2_amcl', executable='amcl', name='amcl',
                parameters=[params, sim_time], output='screen')

    lm_local = Node(package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    parameters=[sim_time, {'autostart': True},
                                {'node_names': ['map_server', 'amcl']}],
                    output='screen')

    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=[params, sim_time],
             remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', parameters=[params, sim_time],
             output='screen'),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', parameters=[params, sim_time],
             output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', parameters=[params, sim_time],
             output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[params, sim_time],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', parameters=[params, sim_time],
             output='screen'),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', parameters=[params, sim_time],
             output='screen'),
    ]

    lm_nav = Node(package='nav2_lifecycle_manager',
                  executable='lifecycle_manager',
                  name='lifecycle_manager_navigation',
                  parameters=[sim_time, {'autostart': True},
                              {'node_names': NAV_NODES}],
                  output='screen')

    return LaunchDescription(
        args + [gz, rsp, bridge, foxglove,
                TimerAction(period=8.0, actions=[spawn]),
                TimerAction(period=20.0, actions=[map_server, amcl, lm_local]),
                TimerAction(period=25.0, actions=nav_nodes + [lm_nav])])