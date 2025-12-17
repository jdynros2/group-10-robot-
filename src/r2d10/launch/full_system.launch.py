import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    r2d10_dir = get_package_share_directory('r2d10')
    nav_dir = get_package_share_directory('r2d10_navigation')
    
    # Launch world + robot + sensors
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(r2d10_dir, 'launch', 'world_launch.py')
        )
    )
    
    # Launch navigation after 5 seconds
    nav_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_dir, 'launch', 'navigation.launch.py')
                )
            )
        ]
    )
    
    # Launch robot view RViz after 8 seconds
    robot_rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_robot',
                arguments=['-d', os.path.join(r2d10_dir, 'rviz', 'default.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        world_launch,
        nav_launch,
        robot_rviz
    ])