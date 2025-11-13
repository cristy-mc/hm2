# HW2 - Kinematic and vision-based controller for ros2-iiwa

## Overview: 

This project implements Homework 2 of the Robotics Lab 2025 course.
The goal of this homework is to develop kinematic and a vision-based controller for a robotic manipulator arm in the simulation environment using KDL and aruco_ros.
The project is based on the ros2_kdl_package package and the ros2_iiwa package provided by the course.

Download the repository's content in the docker's image folder

We created a .launch.py file for every point of the homework.

## 1 Kinematic control:

##  a: 
To launch the iiwa robot:
```bash
ros2 launch iiwa_bringup iiwa.launch.py
```
To launch the controller node:
```bash
ros2 launch ros2_kdl_package kdl_node_1a.launch.py
```

## b:
To launch the iiwa robot:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```
To launch the velocity controller:
-Velocity_ctrl
```bash
ros2 launch ros2_kdl_package kdl_node_1b.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl
```
-Velocity_ctrl_null
```bash
ros2 launch ros2_kdl_package kdl_node_1b.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
```

NOTE:
- ros2_iiwa: original folder, to run the 1a-1b points
- ros2_kdl_package: contains the implementation of the 1a-1b points of the homework
- ros2_kdl_package1c: contains the implementation of the 1c point of the homework
- ros2_iiwa2: contains the implementation of the 2a point of the homework of the homework
- iiwa_description: contains the implementation of the 2b point (the robot that spawns in the world with the aruco tag)

(Molte cose potrebbero non funzionare per problemi con il computer, consultare la cartella del mio collega)
