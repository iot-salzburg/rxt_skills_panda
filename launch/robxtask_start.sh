#!/bin/bash

source ~/ros_workspace/devel/setup.bash

# start all nodes and communication layer from CPP application
roslaunch rxt_skills_panda Panda.launch

# start the ROS action server in python using rospy
rosrun rxt_skills_panda panda_action_server.py
