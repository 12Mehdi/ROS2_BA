#!/bin/bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch fairino5_v6_moveit2_config fr5_bringup.launch.py use_sim:=true
