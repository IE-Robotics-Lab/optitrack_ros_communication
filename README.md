# OptiTrack to ROS Communication
This repository allows for data communication between OptiTrack System (Motive software), ROS, and UDP.

## Set-up:
1. Install VRPN Package into the ROS environment. Run the following command in your terminal:
```bash
sudo apt install ros-$(rosversion -d)-vrpn-client-ros
```
2. Import the `optitrack_ros_communication` package into your catkin workspace and build it:
```bash
cd CATKIN_WORKSPACE_NAME/src
git clone https://github.com/IE-Robotics-Lab/optitrack_ros_communication.git
cd CATKIN_WORKSPACE_NAME
catkin_build
```
Note: Replace CATKIN_WORKSPACE_NAME with your catkin workspace directory name

## Run
Once the set-up is completed, simply launch the file `optitrack_nodes.launch` using the `roslaunch` command with the desired parameters (or modify the default values of the launch file). Some examples are provided below:
```bash
roslaunch optitrack_ros_communication optitrack_nodes.launch

roslaunch optitrack_ros_communication optitrack_nodes.launch number_of_nodes:=10

roslaunch optitrack_ros_communication optitrack_nodes.launch server:=10.205.3.3 number_of_nodes:=2 ip:=10.205.3.224
```
Note: The number of nodes refers to the amount of rigid body markers you want to transmit.


If UDP transmition is not required, you can simply launch the file `optitrack_vrpn_client.launch`:
```bash
roslaunch optitrack_ros_communication optitrack_vrpn_client.launch
```
