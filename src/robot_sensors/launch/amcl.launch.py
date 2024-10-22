import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Nom du package
    package_name = 'robot_sensors'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    robot_localization_file_path = os.path.join(pkg_path, 'config/amcl.yaml')

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[robot_localization_file_path]
        )

    # Lancer l'imu
    imu = Node(
        package='imu_package', executable='imu_main',
        )


    wheel = Node(
        package = 'imu_package', executable = 'wheel_listener'
    )

    return LaunchDescription([
        start_robot_localization_cmd,
        #imu,
        wheel,
    ])
