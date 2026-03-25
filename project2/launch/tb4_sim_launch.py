from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    view_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py'
            )
        )
    )

    controller_node = Node(
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

    return LaunchDescription([
        controller_node,
        view_navigation_launch
        
    ])