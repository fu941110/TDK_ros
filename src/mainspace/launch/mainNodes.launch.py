from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 永遠開啟的節點
        Node(package='SerialTest', executable='mission_serial_node', name='mission_serial_node', output='screen'),
        Node(package='SerialTest', executable='motor_serial_node', name='motor_serial_node', output='screen'),
        Node(package='mainspace', executable='navigator_main', name='navigator_main', output='screen'),
        Node(package='mainspace', executable='StageManager', name='StageManager', output='screen'),
        Node(package='SerialTest', executable='control_node', name='control_node', output='screen'),
    ])
