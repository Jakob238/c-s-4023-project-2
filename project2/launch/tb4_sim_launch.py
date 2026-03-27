from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch rviz2 in navigation mode
    view_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py'
            )
        )
    )

    # Build the SLAM mapping for TurtleBot4
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'slam.launch.py'
            )
        )
    )

    # Launches the controller node
    controller_node = Node(
        package='project2_control',
        executable='project2_controller',
        name='project2_controller',
        output='screen',
    )

    # Returns the launch description with the nodes and launch files
    return LaunchDescription([
        controller_node,
        view_navigation_launch,
        slam_launch
        
    ])