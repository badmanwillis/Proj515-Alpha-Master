# Proj515-Alpha-Master
The master repository, to be deployed to the Alpha robot for testing. This repo contains the various packages developed by the team.

## Requirements
* Ubuntu 16.04
* Ros Kinetic

## Introduction
The robot will make use of the [ROS Navigation package](http://wiki.ros.org/navigation), which contains packages for costmaps, path planning and much more. The tutorial for [Setup and Configuration of the Navigation Stack on a Robot](http://wiki.ros.org/navigation/Tutorials/RobotSetup) provides instructions for setting up the navigation stack on a custom robot, which requires platform specific nodes.


![overview_tf](http://wiki.ros.org/navigation/Tutorials/RobotSetup?action=AttachFile&do=get&target=overview_tf.png)


The platform specific nodes, shown in blue, have been writted by various members of the team, and are included in this repository.

### Sensor transforms
[Proj515-Alpha-RobotSetup](https://github.com/badmanwillis/Proj515-Alpha-RobotSetup). Includes a URDF model of the robot. 

### Sensor sources
Using the robopeak A1 lidar, which requires the [rplidar_ros](https://github.com/Slamtec/rplidar_ros) package.

### Odometry source
Using custom built 

### Base controller
