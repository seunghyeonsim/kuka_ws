import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-file', os.path.join(
                os.getenv('HOME'), 'kuka_ws/src/kuka_apriltag/urdf/taskspace.urdf'),
                '-entity', 'taskspace',
                '-x', '2.8',
                '-y', '0.0',
                '-z', '0.0']
        )
    ])
