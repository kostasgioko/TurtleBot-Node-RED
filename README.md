# TurtleBot Node-RED

## A graphical application development methodology for remote robots in the context of cyber-physical systems.

This project was part of my Diploma Thesis for the Electrical and Computer Engineering programme of Aristotle University of Thessaloniki.

## Abstract

Just as the Internet has transformed the way people interact with information, cyber-physical systems are transforming the way people interact with computational systems. Cyber-physical systems integrate sensing, computation, control and networking into physical objects, connecting them to the Internet and to each other.

A typical example of such systems are robotic systems, as they combine interaction with the environment and computational abilities. Even though robotics is closely tied to the manufacturing industry, in recent years it has branched out to other fields, such as medicine and autonomous exploration, and even in aspects of our daily life, such as for domestic use.

A growth of similar scale can be seen in the Internet of Things (IoT) domain, where everyday objects are equipped with sensors to collect data from the environment and are able to connect to the Internet to share this data. We envision that, due to the mobility offered by robotic systems, their integration with IoT would enable better interaction with the environment, and simultaneously allow robots to make decisions based on data from other devices.

To make this possible, there are certain limitations that must be overcome. On one hand, it is especially important to have the ability to control and monitor the robot remotely. Unfortunately, the Robot Operating System (ROS), the most widespread middleware for robotics development, restricts the management of the robot to the local network. On the other hand, it is desirable for users to have the ability to create their applications without having extensive robotics and programming knowledge.

This thesis focuses on developing a system to address the aforementioned limitations. To establish the communication between the robot and the remote computer, the RabbitMQ message broker is used. At the same time, application development and the integration of the robot with the IoT world are accomplished through Node-RED, a tool for building applications for IoT systems through a graphical interface, thus simplifying the programming procedure. Furthermore, various use cases are presented, which showcase the capabilities of the system for developing robotic applications as part of the IoT.


## Software Used

Robot Side:
- Ubuntu 16.04.7 LTS
- ROS Kinetic  
  Packages:  
    - robot_pose_publisher
- Gazebo Simulator 7.16.1  
  Model Datasets:  
    - http://data.nvision2.eecs.yorku.ca/3DGEMS/
    - https://github.com/aws-robotics/aws-robomaker-small-house-world
- ros2broker: https://github.com/klpanagi/ros2broker


Broker Side:
- RabbitMQ

Remote Side:
- Node-RED v1.0.6  
  Packages:
    - node-red-contrib-config
    - node-red-contrib-moment
    - node-red-dashboard
    - node-red-node-base64
    - node-red-node-email
    - node-red-node-ui-microphone
    - node-red-node-ui-table
- Node.js v10.21.0

## Installation

Firstly, install all required software.

Robot Side:  
1. Place all three ROS Packages folders into /Your install path/catkin_ws/src/  
2. Place building_editor_models content into your own respective folder. If it doesn't exist, place the whole building_editor_models folder into the home directory.  
3. Place the turtlebot folder into /home/username/ros2broker/.  
4. Place the turtlebot_maps and turtlebot_worlds into the home directory.  
5. Change the username in the following places:  
    - ROS Packages/turtlebot_nodered/scripts/navigation_map.py  
    - ROS Packages/multi_robot/scripts/navigation_map_multi.py  
    - ROS Packages/turtlebot_nodered/scripts/remote_handler.py  

Remote Side:  
1. Place the logo picture in /home/username/Pictures.  
2. Import the flows to Node-RED.  

## Usage

Robot Side:  
Run remote_handler.py of turtlebot_nodered package.

Remote side:  
Launch Node-RED.  
Open Node-RED Dashboard.

## Features

The system offers the following features:

- Remote control of the robot
- Simultaneous Localization and Mapping (SLAM)
- Localization
- Navigation
- Teleoperation
- Multiple robot support
- Visualization and easy handling through the Node-RED Dashboard

## Use Cases

To showcase the capabilities of the system, three use case applications have been created.  

1. Coffee Shop  
The robot has the role of a waiter.

2. House Security  
The robot patrols the user's house and informs him of intruders.

3. Art Gallery  
The two robots act as guides of the gallery.
