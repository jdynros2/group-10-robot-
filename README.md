AUTONOMOUS NAVIGATION ROBOT 'R2D10'

MOST RECENT
How to run:

[ROBOT LOADED INTO RVIS]
cd ~/ros2_ws
source install/setup.bash
ros2 launch r2d10 launchfile2.py


[ROBOT LOADED INTO WORLD]
cd ~/ros2_ws
source install/setup.bash
ros2 launch r2d10 worldlaunch2.py
[This world file needs some more terminals to function]

needs seperate terminals for:

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 run ros_gz_bridge parameter_bridge /odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/ros2_ws/install/r2d10/share/r2d10/urdf/assembly_3.urdf)"
ros2 run joint_state_publisher joint_state_publisher
/Downloads$ python3 odom_to_tf.py

launch slam and teleop 
---

[New launch file reduces all seperate terminals into one simple launch file- slam_complete_launch - runs all extra terminals in one.] - needs rvis fixed and debugged
