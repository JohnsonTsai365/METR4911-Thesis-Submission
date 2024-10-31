import launch

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml
import launch_ros
import os

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'map_server',
        'smoother_server',
        'amcl',
        'collision_monitor',
        'velocity_smoother',
    ]
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': 'true'}
    
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    bringup_dir = get_package_share_directory('navigation')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    
    nav_include = GroupAction(
        actions=[
            SetParameter('use_sim_time', 'true'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )
    
    rviz_node = launch_ros.actions.Node(
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        declare_namespace_cmd,
        declare_params_file_cmd,
        rviz_node,
        nav_include,
    ])
