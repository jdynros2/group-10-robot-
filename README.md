AUTONOMOUS NAVIGATION ROBOT 'R2D10'

MOST RECENT
How to run:

[ROBOT LOADED INTO Gazebo]
--
cd ~/ros2_ws

source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/r2d10/worlds/models:$GZ_SIM_RESOURCE_PATH

ros2 launch r2d10 world_launch.py

rviz2 {needs to open image instantly}

[face detector and screenshotting]
--
cd src/r2d10

ros2 run r2d10 facedetection1.py

[RUN NAV2]
--

ros2 launch r2d10 statepublish.py

ros2 launch r2d10_navigation navigation.launch.py use_sim_time:=true


[if auto nav doesnt work]

ros2 run teleop_twist_keyboard teleop_twist_keyboard




Problems and work to be done 
---
navigate towards face script, report writing.


SAVING to the repo
---
cd ~/ros2_ws 

git add .

git commit -m "Write whatever changes you made here"

git push

