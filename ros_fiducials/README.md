# ROS Fiducials Package
This package provides a system that allows a robot to determine its position and orientation by looking at a number of fiducial markers (similar to QR codes) that are fixed in the environment of the robot. Initially, the position of one marker is specified, or automatically determined. After that, a map (in the form of a file of 6DOF poses) is created by observing pairs of fiducial markers and determining the translation and rotation between them.

![aruco_mark](https://www.researchgate.net/profile/Sondre_Tordal/publication/316592734/figure/fig7/AS:668911410352129@1536492316073/Corner-points-of-each-Aruco-marker-as-they-are-represented-in-the-OpenCV-Aruco-library.png)


## Requirements
* [Ubuntu 16.04](https://wiki.ubuntu.com/XenialXerus/ReleaseNotes)
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* [ROS USB Cam](http://wiki.ros.org/usb_cam) for USB camera drivers
* [ROS Fiducials](http://wiki.ros.org/fiducials) for Aruco markers detection


## Setup and Deployment
This will guide you through setting up the Aruco marker detection using an external USB camera.
There are two options:
  1) Copy the package into your workspace
  2) Create the package (For more customisability)

### Installation
    sudo apt-get install ros-kinetic-usb-cam
    sudo apt-get install ros-kinetic-fiducials
    
#### Option 1 - Copying the package
Copy the "usb_cam_custom package" from this repo into [workspace]/src

Go to "Deployment" step.
    
#### Option 2 - Creating a package for external USB camera
Switch to workspace
    
    cd [workspace]

Create a new package "usb_cam_custom" with rospy and usb_cam dependancies.

    catkin_create_pkg usb_cam_custom rospy usb_cam

Go to the new package folder. 

    cd [workspace]/usb_cam_custom
    
Create new folder called "launch" in [workspace]/usb_cam_custom.

Create new file called "usb_cam_custom.launch" in [workspace]/usb_cam_custom/launch.

Navigate to "[computer]/opt/ros/kinetic/share/usb_cam/launch".

Copy the contents of "usb_cam-test.launch" back into "usb_cam_custom.launch".

In the "usb_cam_custom.launch" make the following changes:

    <param name="video_device" value="/dev/video0" />
    <remap from="image" to="/fiducial_images"/>
    
(OPTIONAL) To specify the framerate of the camera add the folloing line and change the value to desired framerate:

    <param name="framerate" value="60" />

### Deployment
Switch to workspace.
    
    cd [workspace]
    
Make and source the workspace.
    
    catkin_make
    source devel/setup.bash
    
Launch the ROS core in a new terminal window.

  roscore

Launch the "usb_cam_custom" camera package in a new terminal window.

    roslaunch usb_cam_custom usb_cam_custom.launch
    
Launch the Aruco detection in a new terminal window and specyfy the camera node as "usb_cam" and image topic as "/image_raw".

    roslaunch aruco_detect aruco_detect.launch camera:=usb_cam image:=/image_raw
    
## Results
To view the ROS computation graph for the Aruco detection run the following command in a new terminal window:

    rqt_graph
    
  ![aruco_rqt_graph](https://raw.githubusercontent.com/ldanilovic/PROJ515/master/pictures/Screenshot%20from%202019-02-14%2011-16-03.png?token=ApjXRixPsOIpXpHP5RQgiEj9hfh2U1YOks5cbojxwA%3D%3D)

The graph shows that the "usb_cam" image is rerouted through "fiducial_images" which detects Aruco markers, overlays the transform onto the image and displays it in "image_view".

The results look like this:

  ![aruco_rqt_graph](https://raw.githubusercontent.com/ldanilovic/PROJ515/master/pictures/Screenshot%20from%202019-02-14%2011-22-43.png?token=ApjXRms065QaAE6JDaX-J7F5-9CryGmVks5cboo1wA%3D%3D)
  
## Further work
To obtain the transform frames, you need to subsrcibe to:
  
    /fiducial_transforms
    /fiducial_vertices

For a full list of active topics run the following comand:

    rostopic list
