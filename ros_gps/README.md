# ROS GPS for robot_pose_ekf
The provided code and tutorial explain how to interface a GPS module with robot_pose_ekf ROS package.
Please note the code already contains all the necessary parts and only pin asigments and names might need to be changed.
Before starting please read through this [tutorial](http://wiki.ros.org/robot_pose_ekf/Tutorials/AddingGpsSensor)

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
    
#### Mbed increasing the buffer size
This step is necessary for longer messages like the Pose.
Go to:

    ros_lib_kinetic/ros/nodehandler.h
    
Modify the line 97 as shown to double the output buffer size:
    
    int OUTPUT_SIZE=(512*2)>
    
### 1) Setting up the GPS module on Mbed

Set up an instance of TynyGPSPlus called gpsModule. This will provide you with GPS class APIs for gpsModule to use.
Set up an instance of Serial called gpsSer. This will provide you with Serial class APIs for gpsModule to use.

    TinyGPSPlus gpsModule;                      // Instance of GPS class
    Serial gpsSer(GPS_TX, GPS_RX);              // Instance of Serial class to gps

### 2) Setting up a ROS Publisher node on Mbed

For tutorial please see http://wiki.ros.org/robot_pose_ekf/Tutorials/AddingGpsSensor

Set up namespaces:

    using namespace std;                        // Makes all "std" symbols visible
    using namespace ros;                        // Makes all "ros" symbols visible
    using namespace nav_msgs;                   // Makes all "nav_msgs" symbols visible
    using namespace std_msgs;                   // Makes all "std_msgs" symbols visible

Set up class instances for ROS:
    
    NodeHandle nh;                              // Instance for ROS Node Handler
    Odometry gps_odom_msg;                      // Instance for ROS Odometry message
    String   gps_stng_msg;                      // Instance for ROS String message
    Publisher gps_odom_pub("gps_odom", &gps_odom_msg); // Instance for ROS publisher (Odometry Message)
    Publisher gps_stng_pub("gps_stng", &gps_stng_msg); // Instance for ROS publisher (String Message)
    ros::Time ros_time;                         // Instance for ROS time (Needs "ros::" to eliminate ambiguity between namespaces)

In main() initialise the following:

    nh.initNode();                              // Initialise ROS Node Handler
    nh.advertise(gps_odom_pub);                 // Adverstise Odometry topic
    nh.advertise(gps_stng_pub);                 // Adverstise String topic
    
    while(!nh.connected()){                     // While node handler in not connected
        nh.spinOnce();                          // Connect to ROS Master
        }
    
Set up the message according to the [tutorial](http://wiki.ros.org/robot_pose_ekf/Tutorials/AddingGpsSensor)

Publish the message by: 
    
     nh.spinOnce();                             // Connect to ROS Master
     gps_odom_pub.publish(&gps_odom_msg);       // Publish the Odometry message
     gps_stng_pub.publish(&gps_stng_msg);       // Publish the String message (Is bound to user button to provide signs of life on button press)

### 3) Setting up ROS Serial subscriber on the PC

Change to workspace:

    cd catkin_ws

Install ROS serial for Mbed:

    sudo apt-get install ros-kinetic-rosserial
    
Make the workspace:

    catkin make
    
Source the workspace:

    source devel/setup.bash

### Deployment
  
Program the Mbed board and plug into your pc.
  
Run roscore:
      
      roscore 
      
Run rosserial (Where ttyACM0 is the name of your device):

    rosrun rosserial_python serial_node.py /dev/ttyACM0
    
If you get an error: "Failed to open port /dev/ttyACM0", you need togive permission to access the port by running:
    
    sudo chmod a+rw /dev/ttyACM0
    
List and subscribe to relevant topics from your robot_pose_ekf:

    rostopic list
    rostopic echo /gps_odom
    rostopic echo /gps_stng
    
## Results
![Results](https://raw.githubusercontent.com/badmanwillis/Proj515-Alpha-Master/master/ros_gps/Screenshot%20from%202019-02-25%2015-05-16.png)

Terminal 1 is running ROS core (Top LEFT)
Terminal 2 is running ROS serial (Top RIGHT)
Terminal 3 is echoing ROS topic /gps_stng (Bot LEFT)
Terminal 4 is echoing ROS topic /gps_odom (Bot RIGHT)
