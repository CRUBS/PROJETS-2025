import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Load parameters
    params_file = LaunchConfiguration('params_file')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('robot_navigation'), 'config', 'smoother.yaml'),
        description='Path to the YAML configuration file'
    )

    # Launch nav2_velocity_smoother node
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[],
        arguments=['--ros-args', '--param', 'desired_frequency:=10']
    )

    return LaunchDescription([declare_params_file, velocity_smoother_node])
