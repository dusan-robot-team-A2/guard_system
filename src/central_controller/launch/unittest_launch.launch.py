import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    central_node = Node(
        package='central_controller',
        executable='central_node',
        parameters=[],
        arguments=[],
        output='screen'
    )

    test_node = Node(
        package='central_controller',
        executable='test_node',
        parameters=[],
        arguments=[],
        output='screen'
    )

    # create and return launch description object
    return LaunchDescription(
        [
            central_node,
            test_node,
        ]
    )