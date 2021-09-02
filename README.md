# raspimouse_slam_navigation
## Requirements
以下の表に使用しているOSやROSのバージョンを示します。  
開発PCも必要だよって書いておく？？
|名称|バージョン|
|----|----|
|Ubuntu|18.04|
|ROS|Melodic|
|Raspberry Pi|3B|

また、本パッケージでは以下の機材を使用しています。（ここは該当パッケージに書くのでも良さそう）  
|種類|名称|
|----|----|
|ゲームパッド|Logicool F710|
|レーザ測域センサ|RPLIDAR|

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

## raspimouse_slam
画像があると良さそう  
LIDARを使ってSLAM（自己位置推定と地図生成）を行うパッケージです。

### Used devices
書くこと
 * 開発PC使ってる
 * 対応しているLIDAR
 * 対応しているコントローラ

### Usage
Raspberry Pi Mouse上で、次のコマンドを実行します。LIDARなどを起動します。
```sh
roslaunch raspimouse_ros_examples mouse_with_lidar.launch rplidar:=true port:=/dev/ttyUSB0
```

Raspberry Pi Mouse上で、次のコマンドを実行します。ゲームパッドで制御することができます。
```sh
roslaunch raspimouse_ros_examples teleop.launch mouse:=false joy:=true joyconfig:=f710
```

次のコマンドを実行して、SLAMを開始します。開発用PC側で起動することを推奨します。この時、開発用PCとRaspberry Pi Mouseが同じROS Master下にいる必要があります。
```sh
roslaunch raspimouse_slam raspimouse_slam.launch rplidar:=true
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

## raspimouse_navigation
画像があるといいよね
このパッケージはamclとmove_baseを利用しています。予め作られた地図と周辺環境の情報から自己位置推定を行い、地図上の任意の座標まで自律移動を行うことができます。

### Used devices
LIDARと開発PC  
ここではRPLIDARを使っているです。LDSにも対応したのは確認できた。URGはまだ。

### Usage
画像入れつつ起動する手順とか示しましょう〜  

Raspberry Pi Mouse上で、次のコマンドを実行します。Raspberry Pi MouseのモータとLIDARを起動するためのノードを起動しています。
```sh
roslaunch raspimouse_navigation robot_navigation.launch rplidar:=true
```

開発用のパソコン上で、次のコマンドを実行します。自己位置推定と経路生成用のノードを起動し、RVizを立ち上げます。
```sh
roslaunch raspimouse_navigation pc_navigation.launch
```

まだ書いてねー、RVizのスクショとかも貼ってねー

# License
このリポジトリは？？ライセンスの元、公開されています。ライセンスについては[LICENSE](hoge)をご参照ください。