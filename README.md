AUTONOMOUS NAVIGATION ROBOT 'R2D10'

How to run:

[COMPLETE ROBOT RUN GUIDE]
--
cd ~/ros2_ws

colcon build 

source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/r2d10/worlds/models:$GZ_SIM_RESOURCE_PATH

ros2 launch r2d10 full_system.launch.py

*This opens the Gazebo wnidow with the robot inside of it after delays it opens an Rviz2 window with Nav2 and a windows with the robot in regular Rviz2 window. After some time for the systems to all launch an process the Nav2 window automatically sets the initial pose of the robot, however the goal needs to be set manualy. The regular Rviz window usually also automatically opens the camera, however if that does not happen, it can be launched by goig into the left side panel, clicking "Add" then going into the "by topic" section and selecting the "image" under the topic, it is the topic "face_image".*





*gazebo opens + robot spawns*

ros2 launch r2d10 statepublish.py

ros2 launch r2d10_navigation navigation.launch.py use_sim_time:=true

*rviz opens, set robot origin estimation, then a goal toward human - navigate around objects.*

-TO SEE LIVE CAMERA WITH FACIAL RECOGNITION OPEN RVIS AND ADD CAMERA TOPICS FACE_IMAGE-

rviz2

*robot will see human with green box around face and save screen shots to ros2_ws/face_screenshots*

[if auto nav doesnt work]

ros2 run teleop_twist_keyboard teleop_twist_keyboard



SAVING to the repo
---
cd ~/ros2_ws 

git add .

git commit -m "Write whatever changes you made here"

git push

