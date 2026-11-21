from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    controller = Node(
        package='p1_reactive',
        executable='controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'slam.launch.py'
            ])
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py'
            ])
        )
    )

    return LaunchDescription([
        controller,
        slam,
        rviz
    ])
