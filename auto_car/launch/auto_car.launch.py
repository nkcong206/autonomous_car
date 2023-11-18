from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

package_share_directory = get_package_share_directory('auto_car')
start = os.path.join(package_share_directory, 'scripts', 'start.sh')
def generate_launch_description():
    chmod_gps = ExecuteProcess(
        cmd=['sudo chmod 666 /dev/ttyUSB1'], 
        shell=True
    )
    chmod_start_file = ExecuteProcess(
        cmd=['chmod +x', start], 
        shell=True
    )
    update_python_can = ExecuteProcess(
        cmd=['bash', start], 
        shell=True,  
        output='screen'  
    )
    wait_for_update_python_can = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=update_python_can,
            on_exit=LaunchDescription([
                socketio,
                gps,
                planning,
                controller
            ])
        )
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
        chmod_gps,
        chmod_start_file,
        update_python_can, 
        wait_for_update_python_can,
        socketio,
        gps,
        planning,
        controller
    ])

