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
    
    # Read URDF for robot_state_publisher
    with open(robot_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    
    # ========== GAZEBO ==========
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )
    
    # ========== SPAWN ROBOT ==========
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
    
    # ========== ROBOT STATE PUBLISHER ==========
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )
    
    # ========== JOINT STATE PUBLISHER ==========
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ========== ROS-GAZEBO BRIDGE ==========
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    slam = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    ]),
    launch_arguments={
        'use_sim_time': 'true',
        'base_frame': 'base_link',  # ← Add this
        'odom_frame': 'odom',
        'map_frame': 'map'
    }.items()
    )
    
    # ========== STATIC TF: map -> odom ==========
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ========== ODOMETRY TO TF BROADCASTER ==========
    odom_to_tf = Node(
        package='r2d10',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        TimerAction(period=4.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[robot_state_publisher]),
        TimerAction(period=5.0, actions=[joint_state_publisher]),
        TimerAction(period=6.0, actions=[bridge]),
        TimerAction(period=6.0, actions=[static_tf_map_odom]),
        TimerAction(period=7.0, actions=[odom_to_tf]),
    ])
