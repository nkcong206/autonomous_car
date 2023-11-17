# File: my_launch_file.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

package_share_directory = get_package_share_directory('auto_car')
start = os.path.join(package_share_directory, 'scripts', 'start.sh')
def generate_launch_description():
    chmod_start_file = ExecuteProcess(
        cmd=['chmod +x', start], 
        shell=True,  
        output='screen'  
    )
    update_python_can = ExecuteProcess(
        cmd=['bash', start], 
        shell=True,  
        output='screen'  
    )
    socketio = Node(
        package='auto_car',
        node_executable='socketio',
        name='socketio',
        output='screen'
    )
    gps = Node(
        package='auto_car',
        node_executable='gps',
        name='socketio',
        output='screen'
    )
    planning = Node(
        package='auto_car',
        node_executable='planning',
        name='planning',
        output='screen'
    )
    controller = Node(
        package='auto_car',
        node_executable='controller',
        name='controller',
        output='screen'
    )
    return LaunchDescription([
        chmod_start_file,
        update_python_can, 
        socketio,
        gps,
        planning,
        controller
    ])

