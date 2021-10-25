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
Raspberry Pi Mouse V3とRemote PCを用意しましょう。  
Raspberry Pi Mouse V3にはデバイスドライバをインストールしている必要があります。こちらの[RT Software Turtorials](https://rt-net.github.io/tutorials/raspimouse/driver/install.html)をご参照ください。  
また、ロボットとRemote PCは、同じネットワーク上で同じROS Masterを指定する必要があります。詳しい設定方法についてはこちらの[RT Software Tutorials](https://rt-net.github.io/tutorials/raspimouse/ros/samples.html#raspberry-pipcros)をご参照ください。  
以下のリストは、必要なソフトや対応しているセンサなどの一覧を示します。  

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
        - [rt-net/raspimouse_slam_navigation_ros](https://github.com/rt-net/raspimouse_slam_navigation_ros)
        - [rt-net/raspimouse_ros_examples](https://github.com/rt-net/raspimouse_ros_examples)
        - [ryuichiueda/raspimouse_ros_2](https://github.com/ryuichiueda/raspimouse_ros_2)
    - オプションパーツ
        - [マルチLiDARマウント](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)

 - Remote PC
    - Linux OS
        - Ubuntu 18.04
    - ROS
        - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - Raspberry Pi Mouse ROS Package
        - [rt-net/raspimouse_slam_navigation_ros](https://github.com/rt-net/raspimouse_slam_navigation_ros)    

また、本パッケージは以下の機材に対応しています。  
 - ゲームパッド
    - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
    - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)
 - レーザ測域センサ
    - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)
    - [LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)

<a name="Installation"></a>
## Installation
### Raspberry Pi Mouse V3
以下のコマンドをロボット側で実行してインストールを行います。
```sh
cd ~/catkin_ws/src
# Clone the ROS packages
git clone https://github.com/ryuichiueda/raspimouse_ros_2
git clone -b melodic-devel https://github.com/rt-net/raspimouse_ros_examples
git clone -b melodic-devel https://github.com/rt-net/raspimouse_slam_navigation_ros
# Install dependencies
rosdep install -r -y --from-paths . --ignore-src

# make and install
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Remote PC
以下のコマンドをRemote PC側で実行してインストールを行います。
```sh
cd ~/catkin_ws/src
# Clone the ROS packages
git clone -b melodic-devel https://github.com/rt-net/raspimouse_slam_navigation_ros
# Install dependencies
rosdep install -r -y --from-paths . --ignore-src

# make and install
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

<a name="QuickStart"></a>
## QuickStart
無事インストールが完了したら、以下の一連のコマンドを実行しましょう。SLAMで地図生成を行い、作った地図を利用してナビゲーションを行うことができます。それぞれの詳しい動かし方などについては[SLAM](#slam)、[ナビゲーション](#navigation)を参照してください。  
ここでは例として、ゲームパッドのLogicool F710とレーザ測域センサのLDS-01を使用しています。
```sh
# SLAMで地図生成
## ロボット側で以下の2つのコマンドを実行
## ゲームパッドの操作方法については、 https://github.com/rt-net/raspimouse_ros_examples#joystick_control を参照してください
roslaunch raspimouse_slam robot_bringup.launch lds:=true port:=/dev/ttyUSB0
roslaunch raspimouse_slam teleop.launch joy:=true joyconfig:=f710
## PC側で次のコマンドを実行実行
roslaunch raspimouse_slam raspimouse_slam.launch lds:=true
## 地図ができたら引き続きPC側で実行
cd $(rospack find raspimouse_slam)/maps
rosrun map_server map_saver -f $MAP_NAME

# ナビゲーション
## ロボット側で次のコマンドを実行
roslaunch raspimouse_navigation robot_navigation.launch lds:=true port:=/dev/ttyUSB0
## PC側で次のコマンドを実行
roslaunch raspimouse_navigation pc_navigation.launch map_file:=$(rospack find raspimouse_slam)/maps/$MAP_NAME.yaml
## RVizが立ち上がるのでそこで操作してみましょう
```

---
<a name="raspimouse_slam"></a>
## raspimouse_slam
LiDARを使ってSLAM（自己位置推定と地図生成）を行うパッケージです。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_gmapping.png width=500 />

ここでは、レーザ測域センサとして[LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)、ゲームパッドとして[Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)を使用しています。

### Usage
Raspberry Pi Mouse上で、次のコマンドを実行します。LiDARなどを起動します。
```sh
roslaunch raspimouse_slam robot_bringup.launch lds:=true port:=/dev/ttyUSB0
```

Raspberry Pi Mouse上で、次のコマンドを実行します。ゲームパッドで制御することができます。  
ゲームパッドの操作方法については、[raspimouse_ros_examplesの"joystick_control"](https://github.com/rt-net/raspimouse_ros_examples#joystick_control)を参照してください。
```sh
roslaunch raspimouse_slam teleop.launch mouse:=false joy:=true joyconfig:=f710
```

次のコマンドを実行して、SLAMを開始します。
RVizが立ち上がり、Raspberry Pi Mouseを動かすと地図が構築されていく様子が見られます。

Remote PC側で起動することを推奨します。この時、Remote PCとRaspberry Pi Mouseが同じネットワーク上で同じROS Masterを指定している必要があります。

```sh
roslaunch raspimouse_slam raspimouse_slam.launch lds:=true
```

地図の保存には次のROSノードを実行します。Remote PC側で起動することを推奨します。
```sh
cd ~/catkin_ws/raspimouse_slam_navigation_ros/raspimouse_slam/maps
rosrun map_server map_saver -f $MAP_NAME
```

`.pgm`と`.yaml`の2つのファイルが生成されています。
```sh
~/catkin_ws/raspimouse_slam_navigation_ros/raspimouse_slam/maps$ ls
$MAP_NAME.pgm $MAP_NAME.yaml
```

地図の確認ができたら、起動しているROSノードを全て終了します。

### Video
以下の動画は、実際にRaspberry Pi MouseがSLAMをしている様子を映しています。  
[![slam_urg](http://img.youtube.com/vi/gWozU47UqVE/sddefault.jpg)](https://youtu.be/gWozU47UqVE)

<a name="raspimouse_navigation"></a>
## raspimouse_navigation
SLAMで地図を生成した後、その地図を使って自己位置推定を行い、地図上の任意の座標まで自律移動を行います。  
このパッケージは[amcl](http://wiki.ros.org/amcl)と[move_base](http://wiki.ros.org/move_base)を使用しています。予め作られた地図と周辺環境の情報から自己位置推定を行い、地図上の任意の座標まで自律移動を行うことができます。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigating_goalpoint.png width=500 />

ここでは、レーザ測域センサとして[LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)を使用しています。  
また、Raspberry Pi MouseとRemote PCが同じネットワーク上で同じROS Masterを指定している必要があります。

### Usage
まずはRaspberry Pi Mouse上で、次のコマンドを実行します。Raspberry Pi MouseのモータとLiDARを起動するためのノードを起動しています。
```sh
roslaunch raspimouse_navigation robot_navigation.launch lds:=true port:=/dev/ttyUSB0
```

Remote PC上で、次のコマンドを実行します。自己位置推定と経路生成用のノードを起動し、RVizを立ち上げます。（下記画像を参照）  
引数のmap_fileパラメータには、SLAMで生成した地図（.yamlファイル）を指定してください。
```sh
roslaunch raspimouse_navigation pc_navigation.launch map_file:=$(rospack find raspimouse_slam)/maps/$MAP_NAME.yaml
```
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_afterlaunched.png width=500 />  

無事RVizが起動したら、まずは初期位置・姿勢を合わせます。RVizの画面上部の緑色の矢印*2D Pose Estimate*をクリックしましょう。地図上で、ロボット実機が最もらしい位置までマウスを持ってきてクリックし**そのままホールド**します。大きな矢印が出ている状態で、マウスを動かすと向きを指示することが可能なので、最もらしい向きに合わせてから、ボタンを離しましょう。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_setting_initialpose.gif width=500 />

初期位置・姿勢の指示が完了したら、次は目標位置・姿勢を指示します。RVizの画面上部の紫色の矢印*2D Nav Goal*をクリックしましょう。地図上で、初期位置・姿勢を合わせた時と同様に、地図上をクリックして目標位置を、ホールドしたままマウスを動かして目標姿勢を指示しましょう。すると、ロボットが自律移動を開始します。  
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_setting_goalpose.gif width=500 />

### Stopping the Robot
与えた目標位置・姿勢への移動を停止したい場合は、新しいターミナルで次のコマンドを実行しましょう。RViz上には目標位置・姿勢が残りますが、ロボットは停止します。新たに、2D Nav Goalを設置すると、そちらに目標位置・姿勢が置き換わります。
```sh
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}
```

また、ロボットが予期しない挙動をした場合は、安全に気をつけながらRaspberry Pi Mouse V3のモータ用電源をOFFにしましょう。
モータ用電源はRaspberry Pi Mouse V3に搭載されたスイッチでON / OFFできます。
あるいは、次のコマンドを実行すると、ソフトウェアスイッチでモータ電源をOFFにできます。
```sh
rosservice call /motor_off
```

### Video
以下の動画は、実際にRaspberry Pi Mouseがナビゲーション（初期位置の設定や目標位置・姿勢の指示）をしている様子を映しています。  
[![nav_lds](http://img.youtube.com/vi/xSMtBZYmhFw/sddefault.jpg)](https://youtu.be/xSMtBZYmhFw)

<a name="License"></a>
# License

(C) 2021 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。 ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。 バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
