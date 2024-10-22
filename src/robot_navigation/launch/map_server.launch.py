import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_file = os.path.join(get_package_share_directory('robot_navigation'),'maps','map.yaml')

    map_server=Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'use_sim_tome':True},
                    {'yaml_filename':map_file}]
    )
    lifecycle_manager=Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time':True},
                    {'autostart':True},
                    {'node_names':['map_server']}]
    )

    return LaunchDescription([
        map_server,
        lifecycle_manager
    ])

