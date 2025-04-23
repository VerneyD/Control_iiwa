Github to control IIWA arm with a xbox controller

cd ~/ros2_ws

source install/setup.bash

source /usr/share/gazebo/setup.sh

ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_planning:=true use_servoing:=true server:=true                 

