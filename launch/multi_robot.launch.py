from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()

    # TurtleBot3 namespace
    turtlebot3 = Node(
        package='turtlebot3_bringup',
        executable='robot_state_publisher',
        namespace='turtlebot3',
        output='screen'
    )

    # Husky namespace
    husky = Node(
        package='husky_gazebo',
        executable='husky_robot_state_publisher',
        namespace='husky',
        output='screen'
    )

    ld.add_action(turtlebot3)
    ld.add_action(husky)

    return ld
