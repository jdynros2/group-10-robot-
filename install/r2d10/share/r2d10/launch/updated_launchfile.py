
# Updated_launchfile.py
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # model_arg = DeclareLaunchArgument('model', description='Path to URDF model')
    # name_arg = DeclareLaunchArgument('name', default_value='r2d10')
    # x_arg = DeclareLaunchArgument('x', default_value='0.0')
    # y_arg = DeclareLaunchArgument('y', default_value='0.0')
    # z_arg = DeclareLaunchArgument('z', default_value='0.0')

    # model = LaunchConfiguration('model')
    # robot_name = LaunchConfiguration('name')
    # x_pos = LaunchConfiguration('x')
    # y_pos = LaunchConfiguration('y')
    # z_pos = LaunchConfiguration('z')

    robot_urdf_model = 'assembly_3.urdf'

    pkg_path = get_package_share_directory('r2d10')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'default.rviz')

    robot_path = os.path.join(pkg_path, 'urdf', robot_urdf_model)

    # --- World file ---
    world_path = os.path.join(pkg_path, 'worlds', 'church.sdf')

    # --- Gazebo launcher ---import os
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')


    try:
        with open(robot_path, 'r') as infp:
            robot_description_content = infp.read()

    except FileNotFoundError:
        print(f"\n\nERROR: URDF file not found at: {robot_path}\n \n")

        return LaunchDescription([])



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments={'gz_args': '-r empty.sdf' }.items()
    )
        
    # ----------------------------------------------------
    # robot_state_publisher (URDF loaded from file string)
    # ----------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            # '--file', robot_description_content,
            '-topic', 'robot_description',
            '-name', 'r2d10',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            # x_arg,
            # y_arg,
            # z_arg
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        # model_arg, name_arg, x_arg, y_arg, z_arg,
        
        gazebo,
        # robot_state_publisher,
        # joint_state_publisher,
        # spawn_entity,
        # rviz
    ])
