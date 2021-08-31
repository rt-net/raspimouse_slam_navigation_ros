# raspimouse_slam_navigation
## Requirements
以下の表に使用しているOSやROSのバージョンを示す  
開発PCも必要だよって書いておく？？
|名称|バージョン|
|----|----|
|Ubuntu|18.04|
|ROS|Melodic|
|Raspberry Pi|3B|
|RaspberryPi OS|nandakke|

また、本パッケージでは以下の機材を使用している  
|種類|名称|
|----|----|
|ゲームパッド|Logicool F710|
|レーザ測域センサ|RPLIDAR|

## Installation
以下のコマンドを実行してインストールを行う。
```sh
cd ~/ros_ws/src
# Clone the ROS packages
git clone https://github.com/ryuichiueda/raspimouse_ros_2
git clone -b melodic-devel https://github.com/rt-net/raspimouse_ros_examples
git clone -b feature/support-melodic-devel https://github.com/rt-net/raspimouse_slam_navigation_ros
# Install dependencies
rosdep install -r -y --from-paths . --ignore-src

# make and install
catkin build
catkin source
```

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

# License
このリポジトリは？？ライセンスの元、公開されています。ライセンスについては[LICENSE](hoge)をご参照ください。