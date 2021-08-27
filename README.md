# raspimouse_slam_navigation
Make sure you are working on ROS Melodic.

## SLAM
Create map using RPLIDAR and F710.
```sh
ROBOT$ roslaunch raspimouse_ros_examples mouse_with_lidar.launch rplidar:=true port:=/dev/ttyUSB0
ROBOT$ roslaunch raspimouse_ros_examples teleop.launch mouse:=false joy:=true joyconfig:=f710
PC$ roslaunch raspimouse_slam raspimouse_slam.launch rplidar:=true
```
Move around... A map should appear on RViz.

Save the map
```sh
PC$ cd ~/ros_ws/raspimouse_slam_navigation_ros/raspimouse_slam/maps
PC$ rosrun map_server map_saver -f <MAP_NAME>
```

Then, there should be two files. `pgm` and `yaml`
```sh
maps$ ls
<MAP_NAME>.pgm <MAP_NAME>.yaml
```

Close all ROS nodes.

## Navigation
This package uses amcl and move_base.
The example below uses RPLIDAR.
```sh
ROBOT$ roslaunch raspimouse_navigation robot_navigation.launch rplidar:=true
PC$ roslaunch raspimouse_navigation pc_navigation.launch
```