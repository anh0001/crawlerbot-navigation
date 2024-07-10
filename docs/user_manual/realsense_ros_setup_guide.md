# Step-by-Step Guide

## 1. Install ROS Noetic
If ROS Noetic is not already installed, follow these steps:

Set up the sources list:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Add the ROS key:
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Update the package list and install ROS Noetic:
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Initialize rosdep:
```bash
sudo rosdep init
rosdep update
```

Set up the ROS environment:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Install the RealSense ROS Wrapper
The RealSense ROS wrapper allows you to use RealSense cameras with ROS.

Install the ROS wrapper for RealSense:
```bash
sudo apt install ros-noetic-realsense2-camera ros-noetic-ddynamic-reconfigure ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

Clone the RealSense ROS wrapper repository, but use ros1-legacy as we are going to use ros1:
```bash
cd src/
git subtree add --prefix realsense-ros https://github.com/IntelRealSense/realsense-ros.git ros1-legacy --squash
cd ..
catkin_make
```

Source the workspace:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Launch the RealSense Node
Finally, you can launch the RealSense node to start using the D455 camera with ROS.

Launch the RealSense node:
```bash
roslaunch realsense2_camera rs_camera.launch
```

This setup will allow you to use the Intel RealSense D455 camera with ROS Noetic on Ubuntu 20.04. The RealSense ROS wrapper provides various topics and services to interact with the camera, such as depth images, color images, and IMU data.

# Important Note

Ensure that the RealSense D455 camera's USB cable is connected to a high-speed USB port on the Intel NUC box.
Do not install realsense libraries using sudo apt if you compile and install from the github as it causes strange errors.