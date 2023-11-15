import os

from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ac_dir = FindPackageShare(package='auto_car').find('auto_car')
    robot_description_file = os.path.join(ac_dir, 'urdf/auto_car.urdf')
    rviz_conf_file = os.path.join(ac_dir, 'rviz/driver.rviz')

    return LaunchDescription([
        Node(
            package='auto_car',
            node_executable='drive_controller',
        ),
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen',
            arguments=[robot_description_file],
        ),
        Node(
            package='rviz2',
            node_executable='rviz2',
            arguments=['-d', rviz_conf_file],
        )
    ])
