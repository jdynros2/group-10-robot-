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
{robot will only load into empty world with no obstacles - issue is with complexity of the furniture and lack of gpu on mini pc, use models from gazebo library to solve} 

cd ~/ros2_ws

source install/setup.bash

ros2 launch r2d10 worldlaunch2.py

{static_transform_publisher may not need to be used anymore, check with gpt}

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard

rvis



Problems and work to be done 
---
Furniture too complex, assignement brief tells us - 
1. at least one chair,
2. at least one coffee table,
3. one bookshelf,
4. at least one sphere,
5. at least one cube,
   
["You can use objects from the Gazebo database." although these models are still heavy on gpu, recommended to use simplified shape models as seen below"]

<img width="254" height="324" alt="Screenshot 2025-11-27 at 9 06 02 AM" src="https://github.com/user-attachments/assets/989ba02f-06df-4380-96e1-27028800350d" />

SAVING to the repo
---
cd ~/ros2_ws 

git add .

git commit -m "Write whatever changes you made here"

git pull --rebase

git push

