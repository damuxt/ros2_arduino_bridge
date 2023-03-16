from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    
    return LaunchDescription([
        Node(package="ros2_arduino_bridge", 
            executable="arduino_node", 
            name="ros2_arduino_node",
            parameters=[os.path.join(get_package_share_directory("ros2_arduino_bridge"),"params","arduino.yaml")],
        )
    ])