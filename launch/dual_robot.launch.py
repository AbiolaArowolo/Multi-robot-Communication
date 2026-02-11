from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # 1. Launch Husky (Clearpath) - uses system packages
    husky_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/clearpath_gz/launch/simulation.launch.py'
        ])
    )
    
    # 2. Delay TurtleBot3 spawn to avoid collision
    spawn_turtlebot = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'turtlebot3',
                     '-database', 'TurtleBot3 Burger',
                     '-x', '2.0',
                     '-y', '2.0',
                     '-z', '0.1',
                     '-robot_namespace', 'turtlebot3'],
                output='screen'
            )
        ]
    )
    
    # 3. Communication test nodes
    husky_talker = Node(
        package='task3_launcher',
        executable='husky_talker',
        name='husky_talker',
        namespace='husky',
        output='screen'
    )
    
    turtle_talker = Node(
        package='task3_launcher',
        executable='turtle_talker', 
        name='turtle_talker',
        namespace='turtlebot3',
        output='screen'
    )
    
    return LaunchDescription([
        husky_sim,
        spawn_turtlebot,
        husky_talker,
        turtle_talker
    ])
