import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # Nom du package
    package_name = 'robot_description'

    pkg_path = os.path.join(get_package_share_directory('robot_gazebo'))
    world_path = os.path.join(pkg_path,'worlds','mondeCDF24.sdf')

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value=world_path,
        description='Path to world sdf file'
    )

    # Lancement de la description du robot
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'description.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()  # enlever le ros2_control
    )

    # Lancement de gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments=[('world', world_path)],
    )

    # Fait apparaitre le robot dans gazebo
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_dif',
            '-x', '1.35851',
            '-y', '0.769643',
            '-z', '1',
            '-Y', '1.5708'],
        output='screen')

    return LaunchDescription([
        world_arg,
        description,
        gazebo,
        spawn_entity
    ])
