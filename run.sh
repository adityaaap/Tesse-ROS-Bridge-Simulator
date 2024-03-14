#!/bin/bash

# cd ~/aero_740_builds/lab3
# ./lab3.x86_64 &
# sleep 3000


cd ~/aero_740/lab3/src
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch &

cd ~/aero_740/lab3/src/controller_pkg
roslaunch controller_pkg traj_tracking.launch &
rviz -d rviz/lab3.rviz 

