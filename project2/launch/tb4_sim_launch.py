from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project2_control',
            executable='project2_controller',
            name='project2_controller',
            output='screen',
            parameters=[{
                'linear_vel': 0.1,          
                'angular_vel': 0.2,         
                'distance_threshold': 0.3048 
            }]
        )
    ])