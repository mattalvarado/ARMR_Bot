ARMR_Bot
========

This repository contains the ROS code for the 2012 Olin College ARL SCOPE Team.

This code was written to serve as an example of how to interface with the ARMR Bot.

It contains nodes for reading IMU data, LIDAR data, Odometery data, and Status data from the cRIO
and publishing that data via ROS.

It also contains a node which subscribes to the cmd_vel topic and sends Twist messages to the robot
for driving.