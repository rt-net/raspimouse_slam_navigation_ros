# raspimouse_slam_navigation
Raspberry Pi MouseでSLAMやナビゲーションを行うためのROSメタパッケージです。

---
# Table of Concents
 - [Requirements](#Requirements)
 - [Installation](#Installation)
 - [QuickStart](#QuickStart)
 - [raspimouse_slam](#SLAM)
 - [raspimouse_navigation](#Navigation)
 - [License](#License)
---

<a name="Requirements"></a>
## Requirements
Raspberry Pi Mouse V3と開発PCを用意しましょう。以下のリストは、必要なソフトや対応しているセンサなどの一覧を示します。  

 - [Raspberry Pi Mouse V3](https://rt-net.jp/products/raspberrypimousev3/)
    - Raspberry Pi
        - Raspberry Pi 3B
    - Linux OS
        - Ubuntu 18.04
    - Device Driver
        - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
    - ROS
        - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - Raspberry Pi Mouse ROS Package
        - [ryuichiueda/raspimouse_ros_2](https://github.com/ryuichiueda/raspimouse_ros_2)
    - オプションパーツ
        - [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)

 - Remote PC
    - Linux OS
        - Ubuntu 18.04
    - ROS
        - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - Raspberry Pi Mouse ROS Package
        - [ryuichiueda/raspimouse_ros_2](https://github.com/ryuichiueda/raspimouse_ros_2)        

また、本パッケージは以下の機材に対応しています。  
 - ゲームパッド
    - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
    - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)
 - レーザ測域センサ
    - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)
    - [LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)

<a name="Installation"></a>
## Installation
以下のコマンドを実行してインストールを行います。
```sh
cd ~/ros_ws/src
# Clone the ROS packages
git clone https://github.com/ryuichiueda/raspimouse_ros_2
git clone -b melodic-devel https://github.com/rt-net/raspimouse_ros_examples
git clone -b melodic-devel https://github.com/rt-net/raspimouse_slam_navigation_ros
# Install dependencies
rosdep install -r -y --from-paths . --ignore-src

# make and install
cd ~/ros_ws
catkin_make
source ~/ros_ws/deve/setup.bash
```

<a name="QuickStart"></a>
## QuickStart
無事インストールが完了したら、以下の一連のコマンドを実行しましょう。SLAMで地図生成を行い、作った地図を利用してナビゲーションを行うことができます。それぞれの詳しい動かし方などについては[SLAM](#slam)、[ナビゲーション](#navigation)を参照してください。  
ここでは例として、ゲームパッドのLogicool F710とレーザ測域センサのLDS-01を使用しています。
```sh
# SLAMで地図生成
## ロボット側で以下の2つのコマンドを実行
roslaunch raspimouse_ros_examples mouse_with_lidar.launch lds:=true port:=/dev/ttyUSB0
roslaunch raspimouse_ros_examples teleop.launch mouse:=false joy:=true joyconfig:=f710
## PC側で次のコマンドを実行実行
roslaunch raspimouse_slam raspimouse_slam.launch lds:=true
## 地図ができたら引き続きPC側で実行
cd ~/ros_ws/raspimouse_slam_navigation_ros/raspimouse_slam/maps
rosrun map_server map_saver -f <MAP_NAME>

# ナビゲーション
## ロボット側で次のコマンドを実行
roslaunch raspimouse_navigation robot_navigation.launch lds:=true
## PC側で次のコマンドを実行
roslaunch raspimouse_navigation pc_navigation.launch　map_file:=$(find raspimouse_slam)/maps/<MAP_NAME>.yaml
## RVizが立ち上がるのでそこで操作してみましょう
```

---
<a name="raspimouse_slam"></a>
## raspimouse_slam
LIDARを使ってSLAM（自己位置推定と地図生成）を行うパッケージです。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_gmapping.png width=500 />

ここでは、レーザ測域センサとして[LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)、コントローラとして[Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)を使用しています。

### Usage
Raspberry Pi Mouse上で、次のコマンドを実行します。LIDARなどを起動します。
```sh
roslaunch raspimouse_ros_examples mouse_with_lidar.launch lds:=true port:=/dev/ttyUSB0
```

Raspberry Pi Mouse上で、次のコマンドを実行します。ゲームパッドで制御することができます。
```sh
roslaunch raspimouse_ros_examples teleop.launch mouse:=false joy:=true joyconfig:=f710
```

次のコマンドを実行して、SLAMを開始します。開発用PC側で起動することを推奨します。この時、開発用PCとRaspberry Pi Mouseが同じROS Master下にいる必要があります。
```sh
roslaunch raspimouse_slam raspimouse_slam.launch lds:=true
```

RVizが立ち上がり、Raspberry Pi Mouseを動かすと地図が構築されていく様子が見れます。

地図の保存には次のROSノードを実行します。開発用PC側で起動することを推奨します。
```sh
cd ~/ros_ws/raspimouse_slam_navigation_ros/raspimouse_slam/maps
rosrun map_server map_saver -f <MAP_NAME>
```

`pgm`と`yaml`の2つのファイルが生成されています。
```sh
~/ros_ws/raspimouse_slam_navigation_ros/raspimouse_slam/maps$ ls
<MAP_NAME>.pgm <MAP_NAME>.yaml
```

地図の確認ができたら、起動しているROSノードを全て終了します。

### Video
以下の動画は、実際にRaspberry Pi MouseがSLAMをしている様子を映しています。  
[![slam_urg](http://img.youtube.com/vi/gWozU47UqVE/sddefault.jpg)](https://youtu.be/gWozU47UqVE)

<a name="raspimouse_navigation"></a>
## raspimouse_navigation
このパッケージはamclとmove_baseを利用しています。予め作られた地図と周辺環境の情報から自己位置推定を行い、地図上の任意の座標まで自律移動を行うことができます。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigating_goalpoint.png width=500 />

ここでは、レーザ測域センサとして[LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)を使用しています。

### Usage
Raspberry Pi Mouse上で、次のコマンドを実行します。Raspberry Pi MouseのモータとLIDARを起動するためのノードを起動しています。
```sh
roslaunch raspimouse_navigation robot_navigation.launch lds:=true
```

開発用のパソコン上で、次のコマンドを実行します。自己位置推定と経路生成用のノードを起動し、RVizを立ち上げます。（下記画像を参照）  
`map_file`パラメータがあるので、随時環境に合わせて変更をしてください。
```sh
roslaunch raspimouse_navigation pc_navigation.launch　map_file:=$(find raspimouse_slam)/maps/<MAP_NAME>.yaml
```
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_afterlaunched.png width=500 />  

無事RVizが起動したら、まずは初期位置・姿勢を合わせます。RVizの画面上部の緑色の矢印*2D Pose Estimate*をクリックしましょう。地図上で、ロボット実機が最もらしい位置までマウスを持ってきてクリックし**そのままホールド**します。大きな矢印が出ている状態で、マウスを動かすと向きを指示することが可能なので、最もらしい向きに合わせてから、マウスを離しましょう。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_setting_initialpose.gif width=500 />

初期位置・姿勢の指示が完了したら、次は目標位置・姿勢を指示します。RVizの画面上部の紫色の矢印*2D Nav Goal*をクリックしましょう。地図上で、初期位置・姿勢を合わせた時と同様に、地図上をクリックして位置を、ホールドしたままマウスを動かして向きを指示しましょう。すると、ロボットが自律移動を開始します。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_setting_goalpose.gif width=500 />


<a name="License"></a>
# License
このリポジトリはApache License Version 2.0 ライセンスの元、公開されています。ライセンスについては[LICENSE](./LICENSE)をご参照ください。