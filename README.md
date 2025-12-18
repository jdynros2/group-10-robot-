AUTONOMOUS NAVIGATION ROBOT 'R2D10'

How to run:

[COMPLETE ROBOT RUN GUIDE]
--
cd ~/ros2_ws

colcon build 

source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/r2d10/worlds/models:$GZ_SIM_RESOURCE_PATH

ros2 launch r2d10 full_system.launch.py







*gazebo opens + robot spawns*

ros2 launch r2d10 statepublish.py

ros2 launch r2d10_navigation navigation.launch.py use_sim_time:=true

*rviz opens, set robot origin estimation, then a goal toward human - navigate around objects.*

-TO SEE LIVE CAMERA WITH FACIAL RECOGNITION OPEN RVIS AND ADD CAMERA TOPICS FACE_IMAGE-

rviz2

*robot will see human with green box around face and save screen shots to ros2_ws/face_screenshots*

[if auto nav doesnt work]

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Problems and work to be done 
---
*LIGHT SOURCE TO BE SAVED NEAR HUMAN FACE SO CAMERA WORKS*, report writing.

Auto add a goal to auto_navigate




SAVING to the repo
---
cd ~/ros2_ws 

git add .

git commit -m "Write whatever changes you made here"

git push

