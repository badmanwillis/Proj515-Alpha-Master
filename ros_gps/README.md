# ROS GPS for robot_pose_ekf
The provided code and tutorial explain how to interface a GPS module with robot_pose_ekf ROS package.
Please note the code already contains all the necessary parts and only pin asigments and names might need to be changed.

![gps_module](https://storage.googleapis.com/stateless-www-faranux-com/2017/08/GYNEO6MV2-GPS-Module-NEO-6M-GY-NEO6MV2.jpg)


## Requirements
* [Ubuntu 16.04](https://wiki.ubuntu.com/XenialXerus/ReleaseNotes)
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* [ROS robot_pose_ekf](http://wiki.ros.org/robot_pose_ekf)
* [ROS Serial](http://wiki.ros.org/usb_cam) Mbed drivers
* [STM32F746ZG Nucleo](https://www.st.com/en/evaluation-tools/nucleo-f746zg.html) (Other boards will work with correct pin assigments)

## Setup and Deployment
This will guide you through setting up the GPS module to work with ROS. There are three parts to this.
  1) Setting up the GPS module on Mbed
  2) Setting up a ROS Publisher node on Mbed
  3) Setting up ROS Serial subscriber on the PC

### Setup

#### Mbed Libraries 

    TinyGPSPlus (Import from Mbed online compiler wizard)
    ros_lib_kinetic (Import from Mbed online compiler wizard)

#### Mbed Include files
    
    #include "mbed.h"
    #include "TinyGPSPlus.h"
    #include "ros.h" 
    #include "nav_msgs/Odometry.h" 
    #include "std_msgs/String.h"
    
### 1) Setting up the GPS module on Mbed

Set up an instance of TynyGPSPlus called gpsModule. This will provide you with GPS class APIs for gpsModule to use.
Set up an instance of Serial called gpsSer. This will provide you with Serial class APIs for gpsModule to use.

    TinyGPSPlus gpsModule;                      // Instance of GPS class
    Serial gpsSer(GPS_TX, GPS_RX);              // Instance of Serial class to gps

### 2) Setting up a ROS Publisher node on Mbed

Set up namespaces:

    using namespace std;                 
    using namespace ros;                       
    using namespace nav_msgs;                   
    using namespace std_msgs;  

Set up class instances for ROS:
    
    NodeHandle nh;                              // Instance for ROS Node Handler
    Odometry gps_odom_msg;                      // Instance for ROS Odometry message
    String   gps_stng_msg;                      // Instance for ROS String message
    Publisher gps_odom_pub("gps_odom", &gps_odom_msg); // Instance for ROS publisher (Odometry Message)
    Publisher gps_stng_pub("gps_stng", &gps_stng_msg); // Instance for ROS publisher (String Message)
    ros::Time ros_time;                         // Instance for ROS time





(Needs "ros::" to eliminate ambiguity between namespaces)

### 3) Setting up ROS Serial subscriber on the PC


### Deployment

    
## Results
