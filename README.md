# OptiTrack to ROS Communication
This repository allows for data communication between OptiTrack System (Motive software), ROS, and UDP. There are currently two options available with Motive software: NatNet has a rate of ~120hz and VRPN has a rate of ~50hz.

## Option 1. Set-up NatNet:
1. Install the requirements by running the following command in your terminal:
```bash
sudo apt install -y ros-$ROS_DISTRO-tf2* wget
```
2. Clone both the `natnet_ros_cpp` and `optitrack_ros_communication` package into your catkin workspace and build it (Note: Replace `CATKIN_WORKSPACE_NAME` with your catkin workspace directory name):
```bash
cd CATKIN_WORKSPACE_NAME/src
git clone https://github.com/L2S-lab/natnet_ros_cpp
git clone https://github.com/IE-Robotics-Lab/optitrack_ros_communication.git
cd CATKIN_WORKSPACE_NAME
catkin_build
```

## Option 2. Set-up VRPN:
1. Install VRPN Package into the ROS environment. Run the following command in your terminal:
```bash
sudo apt install ros-$(rosversion -d)-vrpn-client-ros
```
2. Clone the `optitrack_ros_communication` package into your catkin workspace and build it (Note: Replace `CATKIN_WORKSPACE_NAME` with your catkin workspace directory name):
```bash
cd CATKIN_WORKSPACE_NAME/src
git clone https://github.com/IE-Robotics-Lab/optitrack_ros_communication.git
cd CATKIN_WORKSPACE_NAME
catkin_build
```

## Run
Once the set-up is completed, simply launch the file `optitrack_nodes.launch` using the `roslaunch` command with the desired parameters (or modify the default values of the launch file). The default protocol is NatNet, but can be changed to VRPN by setting the `namespace` parameter to `optitrack`. Some examples are provided below:
```bash
roslaunch optitrack_ros_communication optitrack_nodes.launch

roslaunch optitrack_ros_communication optitrack_nodes.launch number_of_nodes:=10 namespace:=optitrack

roslaunch optitrack_ros_communication optitrack_nodes.launch server:=10.205.3.3 number_of_nodes:=2 ip:=10.205.3.224
```
Note: The number of nodes refers to the amount of rigid body markers you want to transmit.

If UDP transmition is not required, you can simply launch the file `natnet_client.launch` or `vrpn_client.launch`:
```bash
roslaunch optitrack_ros_communication natnet_client.launch
 
roslaunch optitrack_ros_communication vrpn_client.launch
```
