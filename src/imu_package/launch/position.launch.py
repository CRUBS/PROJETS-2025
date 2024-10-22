import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Nom du package
    package_name = 'imu_package'

    pkg_path = os.path.join(get_package_share_directory('imu_package'))
    robot_localization_file_path = os.path.join(pkg_path, 'config/ekf.yaml')

    # Start robot localization using an Extended Kalman filter
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[robot_localization_file_path]
    )

    wheel = Node(
        package = 'imu_package', 
        executable = 'wheel_listener'
    )

    tf_base_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': True,  # Utilise le magnétomètre pour une orientation plus précise
            'world_frame': 'enu',  # Format des données de sortie (East-North-Up)
            'publish_tf': True,  # Désactiver la publication TF si non nécessaire
            'frequency': 25.0,  # Fréquence de traitement (ajuster selon les capacités de l'IMU)
            'gain': 0.3  # Ajuste le gain pour la précision du filtre
        }]
    )

    return LaunchDescription([
        ekf,
        madgwick,
        wheel,
        tf_base_to_odom
    ])
