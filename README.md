# Proj515-Alpha-Master
The master repository, to be deployed to the Alpha robot for testing. This repo contains the various packages developed by the team. It is a software only repository for the Alpha, meant for quick deployment and version control.

## Requirements
* [Ubuntu 16.04](https://wiki.ubuntu.com/XenialXerus/ReleaseNotes)
* [Ros Kinetic](http://wiki.ros.org/kinetic)
* [The ROS navigation package](http://wiki.ros.org/navigation)

## Introduction
The robot makes use of the [ROS Navigation package](http://wiki.ros.org/navigation), which contains packages for costmaps, path planning and much more. The tutorial for [Setup and Configuration of the Navigation Stack on a Robot](http://wiki.ros.org/navigation/Tutorials/RobotSetup) provides instructions for setting up the navigation stack on a custom robot, which requires platform specific nodes.


![overview_tf](http://wiki.ros.org/navigation/Tutorials/RobotSetup?action=AttachFile&do=get&target=overview_tf.png)


The platform specific nodes, shown in blue, have been writted by various members of the team, and are included in this repository.

## Platform specific packages/nodes

### Sensor transforms | alpha_urdf
[Proj515-Alpha-RobotSetup](https://github.com/badmanwillis/Proj515-Alpha-RobotSetup). Includes a URDF model of the robot, and Rviz support.

### Sensor sources | rplidar_ros-master
Using the robopeak A1 lidar, which requires the [rplidar_ros](https://github.com/Slamtec/rplidar_ros) package.

### Odometry source & Base controller |
[PROJ515_Motor_Control_and_Odometry](https://github.com/ElliWhite/PROJ515_Motor_Control_and_Odometry) The code for both the odometry and the base controller have been written in C++ to run on the [Mbed ST-Nucleo-F429ZI microcontroller](https://os.mbed.com/platforms/ST-Nucleo-F429ZI/). The board interfaces with the hardware for the encoders and motor controllers, and communicates with the computer using the [ros_serial node](http://wiki.ros.org/rosserial). This keeps the code running in realtime, isolated from the computer.

#### About the Odom
Using custom built magnetic encoders based on the [as5600 position sensor](https://ams.com/as5600).
* link elliotts original package?

#### About the Base controller
xbox controller, geomtry twist etc
* link masons original package?


## Usage
All of the folder are ROS packages, containing the relevant code. Place them into your_workspace/src. You will need to change the filepaths in alpha_urdf for display.launch and alpha.xml.





