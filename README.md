AUTONOMOUS NAVIGATION ROBOT 'R2D10'

MOST RECENT
How to run:

[ROBOT LOADED INTO RVIS]
--
cd ~/ros2_ws

source install/setup.bash

ros2 launch r2d10 launchfile2.py


[ROBOT LOADED INTO WORLD]
--
Note:
{Human model head needs added without crashing the simulation} 

cd ~/ros2_ws

source install/setup.bash

ros2 launch r2d10 worldlaunch2.py

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True base_frame:=base_link transform_timeout:=2.0

ros2 run teleop_twist_keyboard teleop_twist_keyboard




Problems and work to be done 
---
Add human model head without crashing machine, addition of IMU to create working Slam map, NAV, face detection integration, report writing.


SAVING to the repo
---
cd ~/ros2_ws 

git add .

git commit -m "Write whatever changes you made here"

git push

