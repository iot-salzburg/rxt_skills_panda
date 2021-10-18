# rxt_skills_panda
Implementation of ROBxTASK skills for Franka Panda robot as ROS Action Server

Consists of a CPP application and python-based ROS action server

Start everything with command: "rosrun rxt_skills_panda robxtask_start.sh""

# CPP application

This package realizes a software prototype to let a franka panda cobot (collaborative robot) recognize its surroundings with two 3D cameras and avoid obstacles.
The cobot is controlled via ROS and Ubuntu.The cobot's surroundings are sensed with two Intel D435 3D-Cameras, which are mounted above a human-robot-collaborative (HRC) workspace. Their point cloud streams are first semi-automatically aligned with the iterative closest point algorithm (ICP), such that the final 3D point cloud stream of the HRC workspace shows the whole workspace almost without any masked areas. Afterwards, the point clouds are converted into Octomaps and visualized inside Rviz together with the 3D model of the cobot.

## Prerequisites
- [Ubuntu Xenial](http://releases.ubuntu.com/16.04/)
- [ROS Kinetic](http://wiki.ros.org/kinetic)

## Installation
- Install Ubuntu and ROS.
- Create a [catkin](http://wiki.ros.org/catkin) workspace (see [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).
- Clone this repository into the workspace's `src` directory.
- Follow the instructions in [here](https://github.com/nerovalerius/collision_avoidance) 


## Usage

The robot is then either controllable via ros topics OR via OPC-UA:
- Via ROS Topics:
    ```sh
    rostopic pub /ros_opcua_order std_msgs/String "DD"
    ```
    Possible Commands are:
    - "DD" for the Collision Avoidance Scenario
    - "SO X" to let the robot take something out of the storage, X stands for a Number between 0-9
    - "PO" to let the robot take something from the 3D Printer
    - ...

- Via OPC-UA:
    -Connect to the OPC-UA Server and use the same Strings ("DD", "SO X", "PO") inside the MoveRobotRos Meethod


## All commands

PO :
1) Move To Near Printer
2) Move To Printer and grasp object
3) Move To Near Conveyor Belt
4) Move To Conveyor Belt
5) Move To Initial Position

PS:
1) Move To Near Printer
2) Move To Printer and grasp object
3) Move To Near Storage
4) Move To Storage Place
5) Move To Near Storage
6) Move To Initial Position

SO:
1) Move To Near Storage
2) Move To Near Storage and Grasp Object before
3) Move To Storage Place
4) Move To Near Storage and Grasp Object before
5) Move To Near Conveyor Belt
6) Move To Conveyor Belt
7) Move To Initial Position

SC:
1) Move from storage to cart

DD:
1) Move To Box Position on the left side of the desk
2) Move to box and grip it
3) Move To Box Position on the left side of the desk
4) Move To Box Position on the right side of the desk
5) Move to box and grip it
6) Move To Box Position on the right side of the desk


# Python ROS Action Server

see documentation of rxt_skills_qbo project

start with either launch script or with:
- rosrun rxt_skills_panda panda_action_server.py
- rosrun rxt_skills_panda panda_action_client.py



