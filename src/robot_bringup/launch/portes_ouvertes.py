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
    
    # Lancement de l interface i2c
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
	
    # Launch!
    return LaunchDescription([
	interfaceI2C,
	interfaceGPIO,
	accessories,
	sequenceReader	
    ])
