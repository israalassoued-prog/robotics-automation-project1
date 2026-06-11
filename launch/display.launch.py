import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'robot_arm', 'urdf', 'arm.urdf'
    )
    
    rviz_config = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'robot_arm', 'rviz', 'arm.rviz'
    )
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])