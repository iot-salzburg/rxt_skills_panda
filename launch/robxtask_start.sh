#!/bin/bash

source ~/ros_workspace/devel/setup.bash


# start the ROS action server in python using rospy
gnome-terminal -- rosrun rxt_skills_panda panda_action_server.py
sleep 2

# start roscore (serves as middle layer between all devices in ROBxTASK)
# gnome-terminal -- roscore
# sleep 5

# start all nodes and communication layer from CPP application
gnome-terminal -- roslaunch rxt_skills_panda Panda.launch




