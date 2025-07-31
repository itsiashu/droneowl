from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Mission loader
        Node(
            package='droneowl',
            executable='mission_loader',
            output='screen',
            parameters=[{'mission_file': 'config/mission.yaml'}]
        ),

        # Nav2 bringup
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=['config/nav2_params.yaml']
        ),

        # PX4 offboard control node
        Node(
            package='droneowl',
            executable='offboard_control',
            output='screen',
            parameters=[{'waypoint_file': 'config/mission.yaml'}]
        )
    ])
