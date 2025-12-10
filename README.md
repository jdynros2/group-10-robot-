AUTONOMOUS NAVIGATION ROBOT 'R2D10'

MOST RECENT
How to run:

[ROBOT LOADED INTO RVIS]
--
cd ~/ros2_ws

source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/r2d10/worlds/models:$GZ_SIM_RESOURCE_PATH

ros2 launch r2d10 launchfile2.py


[ROBOT LOADED INTO WORLD]
--
Note:
{Human model head needs added without crashing the simulation} 

cd ~/ros2_ws

source install/setup.bash

ros2 launch r2d10 worldlaunch2.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard




Problems and work to be done 
---
NAV, face detection integration, report writing.


SAVING to the repo
---
cd ~/ros2_ws 

git add .

git commit -m "Write whatever changes you made here"

git push

