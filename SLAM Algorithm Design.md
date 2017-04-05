# SLAM 算法设计
## 目的
在[Jetson TK1 + ROS + GMapping + Hokuyo UST-10LX + Odometry]的基础上，使室内 SLAM 达到以下指标：

- 1. 构建出完整的办公区域室内地图。地图精度达到 3 cm。
     + 北阳雷达精度为 [ 9 mm - 88 mm]（$2\sin(0.25\pi/180)$ × [0.06 - 10] m $\pm$ 40 mm）。
- 2. 要达到 1 中的精度，最大建图半径为 3m 。瞬时建图区域以机器人为中心，为前方 270$^{\circ}$ 的扇形区域。约计 28 $m^2$ 。
- 3. 所建地图轮廓清楚，总是可以识别场景主要特征，可以闭环。
- 4. 建图具有一定的鲁棒性。建图过程不受里程计的绕动影响。
     + 绕动包括轮子打滑，机器人被举起等情况。
- 5. TBD

## 问题分析
目前 SLAM 所用的 GMapping 效果图如下：

![](images/slam_test_000.png)

- 地图有许多零碎的未知区域，没有建图。
    + 可能建图时没有扫描到这些区域；
    + GMapping 没有及时对这些区域作出正确的判断。
- 地图边缘不连贯，轮廓误差明显。
    + GMapping 参数设置未能符合北阳雷达性能；
    + 环境对于传感器的干扰。
- 地图上一些地标性物体不能准确辨认，如桌椅门窗等。
    + GMapping 没有对 2D 地图进一步提取特征的能力；
    + 对于较远处物体，雷达精度未能达到识别要求。
- 建图过程会发生地图整体偏移。
    + GMapping 建议分布对里程计度数依赖较大。

## SLAM 改进建议
### 参数调整
- 针对机器人配置及运行环境，人工调整 GMapping 参数。
- 引入动态参数调整机制。
    + 重构 GMapping 代码，使其可以动态配置主要参数；
    + 编写建图评估模块，实时评估建图效果；
    + 引入HMM或遗传算法或者神经网络模块，实时动态调整参数。

### 引入 2D 地图特征识别
- 在扫描匹配中加入地图特征识别，构建地标来提高扫描匹配的精度；
- 2D 图片识别可以用 OpenCV 中支持的方法；
- 可以考虑用卷积神经网络从粒子地图中提取地图特征。

## 实验
### 实验环境
目前调试用的实验环境为三种：

- 评估机办公室环境；
    + 评估机搭建及使用请详见 GMapping 配置调试文档。
    + 办公室环境 GMapping 建图效果请见[问题分析](#问题分析)。
- 使用 Gazebo 模拟环境；
    + Gazebo 模拟环境搭建请详见 GMapping 配置调试文档。
    + 下载 https://192.168.1.2/svn/zxdj_project/0088 项目工作/智能控制组/0092 导航
    避障专项/0080 代码开发/0000 slam/test_slam_sim 到本地 ROS 工作目录中。默认为 ~/catkin_ws/src 文件夹下。
    ```shell
    cd ~/catkin_ws
    catkin_make
    ```
    + 运行 Gazebo 模拟环境
    ```shell
    roslaunch test_slam_sim oriental_world.launch
    ```
    + 运行 GMapping 并移动机器人建图
    ```shell
    roslaunch test_slam_sim gmapping.launch
    roslaunch turtlebot_teleop keyboard_teleop.launch
    ```
    + 该模拟环境默认集成了里程计，Hokuyo 雷达，Kinect 和 IMU。

    + 建图效果如图所示：

    ![](images/slam_sim_test_001.png)

- 使用 rosbag 重播来重现 topic。
    + rosbag 使用请详见 GMapping 配置调试文档。
    + 目前已录制好一份办公室Hokuyo激光雷达+mrobot里程计的rosbag。放在 test_slam_sim/bagfiles 目录下。重播该份数据制作的 3cm GMapping 效果如图所示：

    ![](images/slam_test_002.png)

### 人工参数调整
目前 mrobot-indigo/mrobot_nav/launch/gmapping.launch 参数调整如下：
```xml
<launch>

  <arg name="scan_topic" default="scan" />

  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen" clear_params="true">
    <param name="odom_frame" value="odom"/>
    <param name="map_update_interval" value="30.0"/>

    <!-- number of particles -->
    <param name="particles" value="30"/>

    <!-- measurement integration -->
    <param name="angularUpdate" value="0.5"/>
    <param name="linearUpdate" value="1"/>
    <param name="temporalUpdate" value="-1.0"/>

    <!-- map resolution -->
    <param name="delta" value="0.05"/>

    <!-- scan matcher -->
    <!-- maximum valid range of the Laser -->
    <param name="maxRange" value="10.0"/>
    <!-- use up to range of the Laser -->
    <param name="maxUrange" value="10.0"/>
    <!-- scan matcher cell sigma, for the greedy search -->
    <param name="sigma" value="0.075"/>
    <param name="iterations" value="5"/>

    <!-- ## default settings for a 0.1 m map cell -->
    <!-- linear search step (choose delta) -->
    <param name="lstep" value="0.1"/>
    <!-- angular search step, this is fine, depending on the odometry error and the update interval -->
    <param name="astep" value="0.05"/>
    <!-- sigma likelihood of 1 beam -->
    <param name="lsigma" value="0.1"/>
    <!-- beams to skip in the likelihood computation -->
    <param name="lskip" value="0"/>

    <!-- the higher the value the slower the filter, the better it can deal with noise, but the less precise and slower -->
    <param name="kernelSize" value="1"/>
    <!-- gain for smoothing the likelihood -->
    <param name="ogain" value="3.0"/>
    <!-- when neff  is below this value a resampling occurs -->
    <param name="resampleThreshold" value="0.5"/>

    <!-- likelihood sampling -->
    <!-- linear range -->
    <param name="llsamplerange" value="0.01"/>
    <!-- linear step -->
    <param name="llsamplestep" value="0.01"/>
    <!-- angular range -->
    <param name="lasamplerange" value="0.005"/>
    <!-- angular step -->
    <param name="lasamplestep" value="0.005"/>

    <!-- motion model parameters -->
    <!-- translation as a function of translation -->
    <param name="srr" value="0.01"/>
    <!-- translation as a function of rotation -->
    <param name="srt" value="0.01"/>
    <!-- rotation as a function of translation -->
    <param name="str" value="0.01"/>
    <!-- rotation as a function of rotation -->
    <param name="stt" value="0.01"/>

    <!--
      inital map params
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      make the starting size small for the benefit of the Android client's memory...
    -->
    <param name="xmin" value="-1.0"/>
    <param name="ymin" value="-1.0"/>
    <param name="xmax" value="1.0"/>
    <param name="ymax" value="1.0"/>

    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
```
评估机办公室环境建图效果如下：

![](images/slam_test_001.png)

评估机大致轨迹（手绘）如下：

![](images/slam_test_001_path.png)

经过参数调整后，整个 SLAM 效果有显著进步。

- 本次操作较为谨慎，已基本消灭了之前地图上出现的零碎未知区域。目前存在的位置区域都可以断定是障碍物遮挡所致。
- 整体地图轮廓连贯。
- 地图上的主要障碍物及地标都能有所发现。

### GMapping 重构
#### 重构目标
+ 可在 GMapping 运行时动态改变尽可能多的 GMapping 参数；
+ 可快速调整 GMapping 算法流程，插入其他算法模块。

### 基于 OpenCV 的 2D 地图特征提取
#### 通过地图在分辨率从低到高状态下的 Maximum Likelihood，层层递进地猜测机器人的 pose
- 使用 50cm X 50cm / 1 pixel 的地图分辨率做 Maximum Likelihood；
- 使用 20cm X 20cm / 1 pixel 的地图分辨率做 Maximum Likelihood；
- 使用 5cm X 5cm / 1 pixel 的地图分辨率做 Maximum Likelihood；

#### 通过地图在不同卷积下的 Maximum Likelihood，猜测机器人的 pose
- 可用的 CV filter：Mean filter，Correlation filter, Gaussian filter，Gabor filter
- Thinning: Non-maxima suppression
- Maximum Likelihood / ICP

#### Using the methods from [OpenCV.feature2D](http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_table_of_contents_feature2d/py_table_of_contents_feature2d.html)
- Extract features: SURF or SIFT or Harris Corner Detector or ...
- Match the features: FLANN ...
- Find the geometrical transformation: RANSAC or LMeds...

### 安装 TensorFlow (1.5 Day)

### 实现实时动态参数调整 (5 Day)

### 实现 CNN 地图特征提取匹配 (10 Day)
