import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
    # Nom du package
    package_name='robot_navigation'

    pkg_path = os.path.join(get_package_share_directory('robot_navigation'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'true',
        description = 'Use Gazebo clock Y/N'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value = os.path.join(pkg_path,'config','slam.yaml'),
        description = 'Absolute path to the nav2 params file'
    )

    # Lancement de la simulation
    simulation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('robot_gazebo'),'launch','gazebo.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 
    slam  = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
                    launch_arguments={'param-file': params_file, 'use_sim_time': use_sim_time}.items()
             )
    
    start_sync_slam_toolbox_node = Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time}
                ],)

    # Lancement teleop manette ps3
    teleop = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')]),
                    launch_arguments={'joy_dev': '/dev/input/event26', 'use_sim_time': 'true'}.items()
             )

    return LaunchDescription([
        #simulation,
        slam,
        start_sync_slam_toolbox_node,
        #teleop
    ])