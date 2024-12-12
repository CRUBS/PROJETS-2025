import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav2_file=os.path.join(get_package_share_directory('robot_navigation'),'config', 'nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('robot_navigation'),'maps','map.yaml')

    lifecycle_nodes = ['map_server', 'amcl']

    nav2_map = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_file],
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True}, 
                    {'node_names': lifecycle_nodes}],
    )

    return LaunchDescription([
        nav2_map,
        nav2_amcl,
        nav2_lifecycle_manager
    ])

