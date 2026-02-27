import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_operations_bringup')

    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Rewrite params to set the BT XML path and use_sim_time
    param_substitutions = {
        'use_sim_time': use_sim_time,
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        # Nav2 Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
        ),

        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),

        # Operations Server (lifecycle node)
        Node(
            package='nav2_operations_server',
            executable='operations_server_node',
            name='operations_server',
            output='screen',
            parameters=[configured_params],
        ),
        
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[configured_params],
            # remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),

        # Lifecycle Manager for Nav2 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'bt_navigator',
                    'operations_server',
                    'behavior_server',
                ],
            }],
        ),

        # Fake Robot
        Node(
            package='nav2_operations_test_nodes',
            executable='fake_robot_node',
            name='fake_robot',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_x': 0.0,
                'initial_y': 0.0,
                'initial_yaw': 0.0,
                'publish_rate': 50.0,
            }],
        ),

        # Blade Simulator
        Node(
            package='nav2_operations_test_nodes',
            executable='blade_simulator_node',
            name='blade_simulator',
            output='screen',
        ),

        # Camera Simulator
        Node(
            package='nav2_operations_test_nodes',
            executable='camera_simulator_node',
            name='camera_simulator',
            output='screen',
            parameters=[{
                'rotation_speed': 45.0,
                'update_rate': 20.0,
            }],
        ),
    ])
