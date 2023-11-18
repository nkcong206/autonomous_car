# File: my_launch_file.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

package_share_directory = get_package_share_directory('auto_car')
runstream = os.path.join(package_share_directory, 'scripts', 'runstream.sh')
print(runstream)
def generate_launch_description():
    chmod_ttyUSB = ExecuteProcess(
        cmd=['sudo chmod 666 /dev/ttyUSB1'], 
        shell=True
    )
    chmod_runstream = ExecuteProcess(
        cmd=['chmod +x',runstream], 
        shell=True
    )
    socketio = Node(
        package='auto_car',
        node_executable='socketio',
        output='screen'
    )
    gps = Node(
        package='auto_car',
        node_executable='gps',
        output='screen'
    )
    planning = Node(
        package='auto_car',
        node_executable='planning',
        output='screen'
    )
    controller = Node(
        package='auto_car',
        node_executable='controller',
        output='screen'
    )
    return LaunchDescription([
        chmod_ttyUSB,
        chmod_runstream,
        socketio,
        gps,
        planning,
        controller
    ])

