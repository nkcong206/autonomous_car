# File: my_launch_file.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    execute_script_action = ExecuteProcess(
        cmd=['bash', './start.sh'], 
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
        execute_script_action, 
        socketio,
        gps,
        planning,
        controller
    ])

