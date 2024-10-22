import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
	
    nodeClock = Node(
        package='robot_accessories',
        executable='clock',
        output='screen',
        parameters=[]
    )
    
    nodePrepareLcd = Node(
        package='robot_accessories',
        executable='lcd_screen_prepare',
        output='screen',
        parameters=[]
    )
    
    nodePrepareLeds = Node(
        package='robot_accessories',
        executable='lidar_led_prepare',
        output='screen',
        parameters=[]
    )
    
    nodePrepareServos = Node(
        package='robot_accessories',
        executable='servos_prepare',
        output='screen',
        parameters=[]
    )
    
    nodePrepareMotors = Node(
        package='robot_accessories',
        executable='motors_prepare',
        output='screen',
        parameters=[]
    )
    
    lidar = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join(
		get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld06.launch.py')]),
        	launch_arguments={}.items()
    )
	
    # Launch!
    return LaunchDescription([
        nodeClock,
        nodePrepareLeds,
        lidar,
        #nodePrepareServos,
        nodePrepareMotors,
        nodePrepareLcd
    ])
