import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # Nom du package
    package_name = 'robot_bringup'

    pkg_path = os.path.join(get_package_share_directory('robot_bringup'))

    # Lancement de la description du robot
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_description'), 'launch', 'description.launch.py'
        )]), launch_arguments={}.items()
    )

    # Lancement de l interface i2c
    interfaceI2C = Node(
        package='robot_communication',
        executable='interfaceI2C',
        output='screen',
        parameters=[]
    )

    # Lancement de l interface gpio
    interfaceGPIO = Node(
        package='robot_communication',
        executable='interfaceGPIO',
        output='screen',
        parameters=[]
    )

   # Lancement de l interface pami
    interfacePAMI = Node(
	package='robot_communication',
	executable='interfacePAMI',
	parameters=[]
    )

    # Lancement de la sequence
    sequenceReader = Node(
        package='robot_accessories',
        executable='sequence_reader',
        output='screen',
        parameters=[]
    )

    # Lancement des accessoires
    accessories = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_accessories'), 'launch', 'allAccessories.launch.py'
        )]), launch_arguments={}.items()
    )

    return LaunchDescription([
        interfaceI2C,
        interfaceGPIO,
	    interfacePAMI,
        description,
        accessories,
        sequenceReader
    ])
