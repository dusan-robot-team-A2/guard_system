import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    guard_node = Node(
        package='guard_system',
        executable='guard_node',
        parameters=[],
        arguments=[],
        output='screen'
    )

    central_controller = Node(
        package='central_controller',
        executable='central_node',
        parameters=[],
        arguments=[],
        output='screen'
    )

    # create and return launch description object
    return LaunchDescription(
        guard_node,
        central_controller,
        
        [
            ExecuteProcess(
                cmd=["ros2", "run", "rviz2", "rviz2"], output="screen"
            ),
        ]
    )