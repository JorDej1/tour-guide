from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import SetLaunchConfiguration
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world = PathJoinSubstitution([
        FindPackageShare('p1_reactive'),
        'resource',
        'gazebo_world'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_gz_bringup'),
                'launch',
                'sim.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'model': 'standard',
        }.items()
    )

    set_bridge_world = SetLaunchConfiguration('world', 'empty')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_gz_bringup'),
                'launch',
                'turtlebot4_spawn.launch.py'
            ])
        ),
        launch_arguments={
            'yaw': '3.14159'
        }.items()
    )
    
    controller = Node(
        package='p1_reactive',
        executable='controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        gazebo,
        set_bridge_world,
        spawn_robot,
        controller,
        slam_toolbox
    ])
