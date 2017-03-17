# GMapping 调试篇
## 目的
在 ROS 中使用激光雷达调试 GMapping，深入了解其工作流程及数据处理流程。

## GMapping 环境搭建
### 安装 ROS
- 参考 [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- 选择 ros-kinetic-desktop-full
```shell
$ sudo apt-get install ros-kinetic-desktop-full
```
- 检查 PYTHONPATH 是否正常
```shell
$ echo $PYTHONPATH
/opt/ros/kinetic/lib/python2.7/dist-packages:/usr/lib/python2.7:/usr/lib/python2.7/plat-x86_64-linux-gnu:/usr/lib/python2.7/lib-tk:/usr/lib/python2.7/lib-old:/usr/lib/python2.7/lib-dynload:/usr/local/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages/PILcompat:/usr/lib/python2.7/dist-packages/gtk-2.0
```
- 如果 PYTHONPATH 为空或者仅含 /opt/ros/kinetic/lib/python2.7/dist-packages 则需要使用下面的命令找到 PYTHONPATH 中需要包含的路径
```shell
$ python -c 'import sys; print(sys.path)'
['', '/opt/ros/kinetic/lib/python2.7/dist-packages', '/usr/lib/python2.7', '/usr/lib/python2.7/plat-x86_64-linux-gnu', '/usr/lib/python2.7/lib-tk', '/usr/lib/python2.7/lib-old', '/usr/lib/python2.7/lib-dynload', '/usr/local/lib/python2.7/dist-packages', '/usr/lib/python2.7/dist-packages', '/usr/lib/python2.7/dist-packages/PILcompat', '/usr/lib/python2.7/dist-packages/gtk-2.0']
```
- 在 ～/.bashrc 中加入 PYTHONPATH
```shell
export PYTHONPATH="${PYTHONPATH}:/usr/lib/python2.7:/usr/lib/python2.7/plat-x86_64-linux-gnu:/usr/lib/python2.7/lib-tk:/usr/lib/python2.7/lib-old:/usr/lib/python2.7/lib-dynload:/usr/local/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages/PILcompat:/usr/lib/python2.7/dist-packages/gtk-2.0"
```
- 建议在 ～/.bashrc 中同时加入 ROS setup
```shell
source /opt/ros/kinetic/setup.bash
source $HOME/catkin_ws/devel/setup.bash
```

### 设置使用镭神 LS01C
#### 安装
- 下载驱动到 $HOME/catkin_ws/src 目录下。驱动位置在 https://192.168.1.2/svn/zxdj_project/0088 项目工作/智能控制组/0092 导航
避障专项/0080 代码开发/0000 slam/talker
- 安装驱动依赖包
```shell
$ rosdep install talker
```
- 编译镭神驱动
```shell
$ cd ~/catkin_ws
$ catkin_make
```
- 在雷达连上USB的情况下查看USB端口号。这里结果显示为 usb 3.0 的7号端口（在 Lenovo 台式机上为正面的 USB 2号口）。
```shell
$ udevadm info --attribute-walk --path=/sys/bus/usb-serial/devices/ttyUSB0 | grep "KERNELS==" | head -1
KERNELS=="3-7:1.0"
```
- 绑定端口到 /dev/ls01c。修改 talker/config/57-ls01c-desktop.rules 文件中 "KERNELS==" 内容与上面端口号内容一致。然后执行 talker/scripts/create_desktop_rules
```shell
$ chmod +x ~/catkin_ws/src/talker/scripts/create_desktop_rues
$ ~/catkin_ws/src/talker/scripts/create_desktop_rues
```
- 然后USB口重新插入雷达。使用下面命令查看雷达是否出现ls01c端口。
```shell
$ ls /dev | grep ls01c
ls01c
```
- 不绑定USB端口的话也是可以的。但是需要使用下面的命令更改 USB 端口权限。并在 talker.launch 里设置 serial_port 为 /dev/ttyUSB0。
```shell
$ sudo chmod 777 /dev/ttyUSB0
```
#### 运行
- 初始化 ROS
```shell
$ roscore
```
- 在新的 Terminal 中
```shell
$ rosrun talker talker.launch
```
- 在rviz中显示数据
```shell
$ roslaunch talker rviz.launch
```
- 可使用 rostopic 查看、接受、发布雷达的 topics
```shell
$ rostopic list
/rosout
/rosout_agg
/scan
/startOrStop
```
- 其中，/scan 为镭神雷达的发布激光数据；
- /startOrStop 用于接收停止雷达数据、电机转动，或者启动转动的命令。消息类型为std_msgs::Int32，消息值为1时停止数据，为2时停止电机，为4时启动扫描。举例如下：
```shell
rostopic pub /startOrStop std_msgs/Int32 "data: 2"
rostopic pub /startOrStop std_msgs/Int32 "data: 4"
```

### 安装 Gazobo 模拟器（Optional）
- Install with ROS
```shell
$ ros-kinetic-gazebo-ros
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
- Or install the latest version (Untested)
```shell
$ curl -ssL http://get.gazebosim.org | sh
```
- Install all the models manually
```shell
#!/bin/sh
# Download all model archive files
wget -l 2 -nc -r "http://models.gazebosim.org/" --accept gz
# This is the folder into which wget downloads the model archives
cd "models.gazebosim.org"
# Extract all model archives
for i in *
do
  tar -zvxf "$i/model.tar.gz"
done
# Copy extracted files to the local model folder
cp -vfR * "$HOME/.gazebo/models/"
```

### TurtleBot with Gazebo
- Installation
```shell
$ sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi  ros-kinetic-ar-track-alvar-msgs ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-rviz-launchers
```
- The following command starts the turtlebot in the simulated world.
```shell
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```
- 控制 turtlebot 移动
```shell
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```
- Mapping within Gazebo
```shell
$ roslaunch turtlebot_gazebo gmapping_demo.launch
```
- Use RViz to visualize the map building process
```shell
$ roslaunch turtlebot_rviz_launchers view_navigation.launch
```
- Save the map to disk
```shell
$ rosrun map_server map_saver -f <your map name>
```

### GMapping 相关设置

## 参考文献
1. [TurtleBot - Make a map and navigate with it](http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it)
