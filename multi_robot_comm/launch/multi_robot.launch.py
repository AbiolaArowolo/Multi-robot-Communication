from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Paths
pkg_multi_robot = get_package_share_directory('multi_robot_comm')
turtlebot3_urdf = os.path.join(pkg_multi_robot, 'urdf', 'husky.urdf.xacro')
world_path = os.path.join(pkg_multi_robot, 'worlds', 'warehouse.world')

def generate_launch_description():
    return LaunchDescription([
        # Gazebo simulator
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            arguments=['--verbose', world_path],
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'husky1',
                '-file', turtlebot3_urdf,
                '-x', '0', '-y', '0', '-z', '0.01'
            ],
            output='screen'
        ),

        # Add more robot nodes here if you have Python ROS2 nodes
        # Node(
        #     package='multi_robot_comm',
        #     executable='robot_spawner',
        #     name='robot_spawner',
        #     output='screen'
        # ),
    ])

