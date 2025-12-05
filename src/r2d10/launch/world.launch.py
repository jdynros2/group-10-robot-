# # world.launch.py
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     pkg_name = 'r2d10'
#     pkg_share = get_package_share_directory(pkg_name)

#     # --- World ---
#     world_path = os.path.join(pkg_share, 'worlds', 'church.sdf')

#     # --- Robot URDF ---
#     robot_path = os.path.join(pkg_share, 'urdf', 'assembly_3.urdf')

#     # --- Gazebo launch ---
#     ros_gz_sim_share_dir = get_package_share_directory('ros_gz_sim')
#     gz_sim_launch = os.path.join(ros_gz_sim_share_dir, 'launch', 'gz_sim.launch.py')

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(gz_sim_launch),
#         launch_arguments={'gz_args': world_path}.items()
#     )

#     # --- Spawn robot & launch RViz ---
#     updated_launch = os.path.join(pkg_share, 'launch', 'Updated_launchfile.py')

#     spawn_robot = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(updated_launch),
#         launch_arguments={
#             'model': robot_path,
#             'name': 'r2d10',
#             'x': '0.0',
#             'y': '0.0',
#             'z': '0.0'
#         }.items()
#     )

#     return LaunchDescription([
#         gazebo,
#         spawn_robot
#     ])

# world.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_name = 'r2d10'
    pkg_share = get_package_share_directory(pkg_name)

    # --- World file ---
    world_path = os.path.join(pkg_share, 'worlds', 'church.sdf')

    # --- Robot URDF/XACRO ---
    robot_path = os.path.join(pkg_share, 'urdf', 'assembly_3.urdf')

    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    # # --- Gazebo launcher ---
    # ros_gz_sim_share = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    # gz_sim_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    # gazebo_model_path = os.path.join(pkg_share, 'meshes')

# --- Gazebo launcher ---
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    gazebo_model_path = os.path.join(pkg_share, 'meshes')


    # Adding Gazebo model path environment variable
    AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gazebo_model_path)

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gz_sim_launch),
    #     launch_arguments={
    #         'gz_args': f'-r empty.sdf'}.items()
    # )




    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items()
    )


    # --- Include robot spawn launch ---
    # updated_launch = os.path.join(pkg_share, 'launch', 'updated_launchfile.py')


    try:
        with open(robot_path, 'r') as infp:
            robot_description_content = infp.read()

    except FileNotFoundError:
        print(f"\n\nERROR: URDF file not found at: {robot_path}\n \n")

        return LaunchDescription([])



    # spawn_robot = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(updated_launch),
    #     launch_arguments={
    #         'model': robot_path,
    #         'name': 'r2d10',
    #         'x': '0.0',
    #         'y': '0.0',
    #         'z': '0.0'
    #     }.items()
    # )


    # delay_on_spawn = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=[
    #         # '--file', robot_description_content,
    #         '-topic', {'robot_description': robot_description_content},
    #         '-name', 'r2d10',
    #         '-x', '0.0',
    #         '-y', '0.0',
    #         '-z', '0.0',
    #         # x_arg,
    #         # y_arg,
    #         # z_arg
    #     ]
    # )

    delay_on_spawn = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=[
        '--file', robot_path,
        '-name', 'r2d10',
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.0'
    ]
)


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


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )


    return LaunchDescription([
    AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gazebo_model_path),

    gazebo,

    robot_state_publisher,
    joint_state_publisher,

    TimerAction(
        period=4.0,
        actions=[delay_on_spawn]
    ),

    rviz
])

