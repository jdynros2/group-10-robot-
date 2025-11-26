from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('r2d10')

    world_path = os.path.join(pkg_share, 'worlds', 'church_simple.sdf')
    models_path = os.path.join(get_package_share_directory('r2d10'), 'worlds', 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    robot_path = os.path.join(pkg_share, 'urdf', 'assembly_3.urdf')
    gazebo_model_path = os.path.join(pkg_share, 'meshes')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')

    AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gazebo_model_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '--file', robot_path,
            '-name', 'r2d10',
            '-x', '7.5', '-y', '1', '-z', '1.0'
        ]
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=4.0, actions=[spawn_robot])
    ])
