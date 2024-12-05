import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav2_file=os.path.join(get_package_share_directory('robot_navigation'),'config', 'nav2_params.yaml')
    default_bt_xml_path= os.path.join(get_package_share_directory('robot_navigation'),'config','navigate_w_replanning_and_recovery.xml')
    map_file = os.path.join(get_package_share_directory('robot_navigation'),'maps','map.yaml')

    nav2_map = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl_server',
        output='screen',
        parameters=[nav2_file],
    )
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_file]
    )
    nav2_planner=Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_file]
    )
    nav2_recorveries= Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recorveries_server',
        output='screen'
    )
    nav2_bt=Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_file, {'default_bt_xml_filename': default_bt_xml_path}]
    )
    nav2_lifecycle_manager=Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart':True},
                    {'node_names':['controller_server',
                                   'planner_server',
                                   'recorveries_server',
                                   'bt_navigator',
                                   'amcl_server',
                                   'map_server']}]
    )
    return LaunchDescription([
        nav2_map,
        nav2_amcl,
        nav2_controller,
        nav2_planner,
        nav2_recorveries,
        nav2_bt,
        nav2_lifecycle_manager,
    ])