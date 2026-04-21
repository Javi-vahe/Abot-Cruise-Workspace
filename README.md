# Abot Cruise Workspace

>基于 ROS1 的视觉巡航赛机器人（以打破赛事记录的绝对领先水平斩获**中国机器人及人工智能大赛全国一等奖**）的完整工作空间。ps：我当年是第一年参加这个比赛，面对的是各类拥有祖传国一代码的外校队伍以及同校三年老队伍的竞争，从零开始完成约 20 天的高强度开发和调试，将校内竞争的队伍全部打败（因为这个比赛同一个学校只能有一支队伍获得国一），真的很不容易。该工作空间集成任务执行、底盘驱动、IMU、2D 激光 SLAM / 定位导航、视觉识别、AR Tag 跟踪、任务调度与语音/发射控制等功能模块。
>如果这个项目对你有帮助，欢迎点一个 Star，感谢！

<p align="center">
  <img alt="ROS1" src="https://img.shields.io/badge/ROS-1-blue" />
  <img alt="Catkin" src="https://img.shields.io/badge/Build-catkin-orange" />
  <img alt="Language" src="https://img.shields.io/badge/C%2B%2B-Python-green" />
  <img alt="Status" src="https://img.shields.io/badge/Use-Competition%20Robot-red" />
</p>

## 项目简介
本项目是一个面向**中国机器人及人工智能大赛**的工作空间。围绕比赛平台 **Abot** 的轮式移动机器人展开，构建了一套完整的 ROS1 竞赛代码。核心能力包括：

- 底盘串口驱动与里程计融合
- IMU 接入与姿态滤波
- RPLidar 激光雷达接入与点云/扫描过滤
- gmapping 建图
- 基于已有地图的导航与局部规划
- 基于激光地图匹配的自定义定位节点
- 视觉检测（各类物体、光流/目标追踪）
- AR Tag 跟踪与竞赛任务触发
- 多目标点巡航与分区逻辑判断
- 任务节点封装与比赛流程编排
- 语音识别 / TTS / 发射控制等扩展模块

我开源的是一个**可运行的比赛级工作空间**，包含硬件驱动、感知、导航、控制、任务调度和若干实验/演示模块。

---

## 工作空间结构

```text
Abot_Cruise_ws/
├── .catkin_workspace
├── 1-gmapping.sh                # 一键启动建图流程
├── 2-navigation.sh              # 一键启动导航/巡航流程
├── chmod.sh
└── src/
    ├── abot_base/
    │   ├── abot_bringup/        # 底盘驱动、EKF里程计桥接、基础启动
    │   ├── abot_imu/            # IMU 驱动与消息定义
    │   ├── abot_model/          # 机器人模型、URDF、Gazebo 启动
    │   └── lidar_filters/       # 激光雷达过滤
    ├── abot_find/               # find_object_2d 目标识别
    ├── ar_tracker/              # AR 跟踪相关启动
    ├── cam_track/               # 摄像头跟踪控制
    ├── color_pkg/               # 颜色线跟随 / 火焰检测
    ├── face_pkg/                # 人脸检测/识别、行人检测
    ├── imu_filter/              # Madgwick / Mahony 等滤波实现
    ├── jie_ware/                # 自定义激光定位模块 lidar_loc
    ├── pid_follow_planner/      # 自定义 PID 路径跟踪局部规划器
    ├── robot_slam/              # 建图、导航、多点巡航、服务节点（比赛任务逻辑也在这里）
    ├── robot_voice/             # 语音识别 / TTS / 语音助手
    ├── shoot_cmd/               # 串口发射控制
    ├── tracker_pkg/             # KCF/LK/特征跟踪
    ├── track_tag/               # AR Tag 跟踪与射击触发
```

---

## 系统环境建议

### 推荐环境

本工程**更推荐 ROS Melodic + Python 3 环境**。核心说明：

- Ubuntu 18.04
- ROS Melodic
- Python 2.7 / Python 3 混合
- catkin_make

### 可迁移环境

实际上理论上可以迁移到：

- Ubuntu 20.04
- ROS Noetic

但需要额外处理：

- Python2 → Python3 兼容修改
- 旧版 `gnome-terminal -e` 启动脚本适配
- 某些第三方库接口变化
- 部分专用串口/摄像头/语音 SDK 的重新配置

因此想要使用此代码的友友们，建议跟我们一样直接烧录官方给的镜像即可

---

## 主要依赖

### ROS 侧依赖

- `roscpp`
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf`
- `tf2_ros`
- `tf_conversions`
- `actionlib`
- `actionlib_msgs`
- `move_base_msgs`
- `message_generation`
- `message_runtime`
- `cv_bridge`
- `image_transport`
- `message_filters`
- `robot_pose_ekf`
- `map_server`
- `move_base`
- `gmapping`
- `joy`
- `usb_cam`
- `ar_track_alvar`
- `rplidar_ros`
- `imu_filter_madgwick`
- `gazebo_ros`
- `joint_state_publisher`
- `robot_state_publisher`
- `rviz`
- `teleop_twist_keyboard`

### 系统 / 第三方库依赖

- OpenCV
- Eigen3
- Qt5
- 串口库 / 自定义硬件通信接口
- 语音 SDK

### 参考安装方式

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-joy \
  ros-$ROS_DISTRO-usb-cam \
  ros-$ROS_DISTRO-rplidar-ros \
  ros-$ROS_DISTRO-ar-track-alvar \
  ros-$ROS_DISTRO-robot-pose-ekf \
  ros-$ROS_DISTRO-gmapping \
  ros-$ROS_DISTRO-map-server \
  ros-$ROS_DISTRO-move-base \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-imu-filter-madgwick \
  ros-$ROS_DISTRO-gazebo-ros \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport
```

---

## 编译方式

```bash
cd ~/Abot_Cruise_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make
source devel/setup.bash
```

如果脚本没有执行权限：

```bash
find src -type f -name "*.py" -exec chmod +x {} \;
chmod +x 1-gmapping.sh 2-navigation.sh
```

或者直接用我们的一键赋予权限小脚本（执行这个脚本可能也需要权限）：


```bash
chmod +x chmod.sh
bash chmod.sh
```

---

## 核心运行链路

### 1. 建图流程

我的脚本 `1-gmapping.sh` 给出了标准建图顺序：

1. 启动 `roscore`
2. 启动底盘与 IMU：`abot_bringup/robot_with_imu.launch`
3. 启动 `robot_slam/gmapping.launch`
4. 启动 RViz：`robot_slam/view_mapping.launch`
5. 启动键盘遥控：`teleop_twist_keyboard`

等价命令：

```bash
roscore
roslaunch abot_bringup robot_with_imu.launch
roslaunch robot_slam gmapping.launch
roslaunch robot_slam view_mapping.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

地图保存：

```bash
roslaunch robot_slam save_map.launch
```

默认地图文件位于：

```text
src/robot_slam/maps/my_lab.yaml
src/robot_slam/maps/my_lab.pgm
```

---

### 2. 导航 / 巡航赛流程

我的顶层脚本 `2-navigation.sh` 给出了比赛执行链路：

1. 启动相机标定/USB 摄像头
2. 启动 `find_object_2d`
3. 启动 AR 跟踪相机节点
4. 启动底盘 + IMU
5. 启动导航定位：`robot_slam/navigation.launch`
6. 启动 RViz 可视化：`robot_slam/view_nav.launch`
7. 启动多目标点任务：`robot_slam/multi_goal.launch`

等价命令：

```bash
roscore
roslaunch track_tag usb_cam_with_calibration.launch
roslaunch find_object_2d find_object_2d.launch
roslaunch track_tag ar_track_camera.launch
roslaunch abot_bringup robot_with_imu.launch
roslaunch robot_slam navigation.launch
roslaunch robot_slam view_nav.launch
roslaunch robot_slam multi_goal.launch
```

---

## 整体架构解析

### A. 底盘层：`abot_base`

#### `abot_bringup`
负责整车基础启动与串口底盘驱动。

关键文件：

- `src/base_driver.cpp`：底盘驱动核心
- `src/serial_transport.cpp`：串口通信实现
- `launch/bringup.launch`：仅轮速/里程计
- `launch/bringup_with_imu.launch`：底盘 + IMU + EKF
- `params/base_params.yaml`
- `params/base_params_with_imu.yaml`

已识别参数：

```yaml
port: /dev/abot
buadrate: 921600
base_frame: base_link
odom_frame: odom
odom_topic: wheel_odom
publish_tf: true/false
cmd_vel_topic: cmd_vel
```

其中：

- `abot_driver` 负责底盘串口收发
- `robot_pose_ekf` 融合轮速与 IMU
- `odom_ekf.py` 将 `odom_combined` 桥接为 `odom`

#### `abot_imu`
负责 IMU 采集与原始消息发布。

- 自定义消息：`msg/RawImu.msg`
- 启动文件：`launch/imu_ahrs.launch`
- 在 `bringup_with_imu.launch` 中与 `imu_filter_madgwick` 协同使用

#### `abot_model`
提供机器人模型与 Gazebo 仿真启动：

- `urdf/abot_model.urdf`
- `launch/display.launch`
- `launch/gazebo.launch`
- `launch/gazebo_world.launch`

#### `lidar_filters`
为激光数据做过滤，`rplidar.launch` 中自动 include：

- `launch/box_filter_example.launch`

---

### B. SLAM / 定位 / 导航：`robot_slam + jie_ware + pid_follow_planner`

#### `robot_slam`
这个就是整个比赛流程的核心包。

主要功能：

- gmapping 建图
- 地图加载与导航
- 多目标点巡航
- 姿态微调服务
- 区块判断服务
- 导航速度转发
- RViz 视图配置

关键 launch：

- `launch/gmapping.launch`
- `launch/navigation.launch`
- `launch/multi_goal.launch`
- `launch/view_mapping.launch`
- `launch/view_nav.launch`
- `launch/save_map.launch`

关键源码：

- `src/adjust_server.cpp`：到点后精调服务 `/adjust`
- `src/block_judge.cpp`：四象限分区逻辑服务 `/block_judge`
- `src/cmd_vel_relay.cpp`：速度转发与调制
- `src/navigate.cpp`：导航控制相关
- `scripts/navigation_multi_goals.py`：比赛多点任务主脚本
- `scripts/formula.py`：识别/公式相关辅助逻辑

#### `navigation.launch`
我的导航主入口主要做了三件事：

1. 使用 `map_server` 加载静态地图 `my_lab.yaml`
2. 启动 `move_base` 参数体系
3. 启动两个自定义能力节点（一个是ICP定位节点、一个是PID局部规划器）：
   - `jie_ware/lidar_loc`
   - `pid_follow_planner/pid_follow_planner`

这也是我能斩获全国冠军的核心：我完全没有用传统的 AMCL，而是结合了**自定义激光定位 + 自定义 PID 路径跟踪器**。

#### `jie_ware`
这里感谢好朋友**B站ROS知名教学UP主《机器人工匠阿杰》倾心开源**的2D激光雷达许多常用功能的功能包，在里面也实现了名为 `lidar_loc` 的激光定位节点，核心思想是ICP配准，其功能为：

- 订阅地图、激光、初始位姿等信息
- 将地图与扫描数据进行匹配
- 发布 / 修正位姿与 TF
- 具备清理 costmap 的配套逻辑

#### `pid_follow_planner`
这是我自定义的 PID 路径跟踪局部控制器。

关键参数包括：

- `plan_frequency`
- `max_x_speed`
- `max_y_speed`
- `goal_dist_tolerance`
- `prune_ahead_distance`
- `p_value`
- `i_value`
- `d_value`

我在这里的 `navigation.launch` 中将 `/cmd_vel_origin` 重映射到 `/cmd_vel_original`，再由其他节点接管最终速度输出。

---

### C. 比赛任务编排：`robot_slam` 与 `user_demo`

#### `multi_goal.launch`
这是比赛模式最重要的启动文件之一。它同时拉起：

- `navigation_multi_goals.py`：多点导航任务主逻辑
- `cmd_vel_relay_node`：速度转发
- `adjust_server`：姿态微调服务
- `formula.py`：识别辅助逻辑
- `block_judge` / `block_judge_rage`：普通模式 / 狂暴模式分区控制

#### 多区块策略
`block_judge.cpp` 中将场地按中点 `(mid_point_x, mid_point_y)` 分为 4 个区域：

- Block 1
- Block 2
- Block 3
- Block 4

机器人根据当前 `map -> base_link` 坐标判断自己位于哪个区块，并控制何时重新启用识别。

#### 多目标点配置
`multi_goal.launch` 中给出了 4 个区块的识别点、目标点和终点坐标，例如：

- `goalList_X_Block_1`
- `goalList_Y_Block_1`
- `goalList_Yaw_Block_1`
- ...
- `goalList_*_Block_4`

同时，为应对当年国赛新规则的改变（区块识别结果会产生的链式影响），我们在launch里加上了可以进行绝境争分的**狂暴模式**，这个版本启动后会抓住官方规则中的漏洞（毕竟官方规则可没说识别必须按顺序来执行嘻嘻），实时证明这个模式确实能比别人的跑法要快约20s左右。

这也是我们仔细设计的比赛逻辑，不是简单的单点导航，而是**场地分区块末端制导解耦合多任务巡航序列**。

#### `navigation_multi_goals.py`
该脚本可视为比赛总控脚本，主要负责：

- 调用 `move_base` 进行导航
- 监听 `/ar_pose_marker`
- 监听 `/object_position`
- 调用 `/adjust` 服务
- 调用 `/block_judge` 服务
- 通过话题切换识别/等待状态
- 播放语音提示音频
- 根据识别结果决定下一步动作

这是我们整个巡航赛任务编排的“大脑”。

### D. 视觉感知模块

#### `track_tag`
AR Tag 跟踪控制模块。

关键文件：

- `launch/ar_track_camera.launch`
- `launch/track_command.launch`
- `launch/usb_cam_with_calibration.launch`
- `src/ar_track.cpp`

功能逻辑：

- 订阅 `/ar_pose_marker`
- 根据 AR Tag 偏差计算角速度/线速度
- 向 `/cmd_vel` 发布控制命令
- 条件满足时向 `/shoot` 发送指令

这部分主要是**目标点精确接近/瞄准的视觉闭环模块**。

#### `abot_find`（包名 `find_object_2d`）
这是完整引入的目标识别包，支持：

- 2D 目标识别
- 3D 目标识别
- 训练目标模板
- ROS 话题输出检测结果

关键 launch：

- `find_object_2d.launch`
- `find_object_3d.launch`
- `train.launch`

#### `color_pkg`
包含两个例程的赛题（没啥用）：

- `line_follower.launch`：颜色/线跟随
- `fire_detector.launch`：火焰检测

#### `face_pkg`
面向人脸与行人识别：

- `face_rec.launch`
- `face_detector.launch`
- `detect_people.launch`


#### `tracker_pkg`
这部分是目标跟踪实验包，含：

- `follower.launch`
- `kcf_tracker.launch`
- `lk_tracker.launch`

对应脚本：

- `follower.py`
- `kcf_kalman_tracker.py`
- `lk_tracker.py`

#### `cam_track`
将摄像头目标偏差通过 PID 参数转为控制指令。

- `launch/cam_track.launch`
- `param/PID.yaml`
- `src/cam_track_node.cpp`

---

### E. 语音与外设扩展

#### `robot_voice`
包含：

- `iat_publish.cpp`
- `tts_subscribe.cpp`
- `voice_assistant.cpp`

这部分主要用于：

- 语音识别结果发布
- TTS 播报
- 语音助手控制入口

#### `shoot_cmd`
这是发射控制模块，包含：

- `SerialPort.cpp`
- `shoot_control.cpp`
- `control_center.cpp`

控制机器人炮台的发射（视觉巡航赛用不上）。

---

## 主要的话题 / 服务 / 数据流

### 典型话题

- `/cmd_vel`
- `/cmd_vel_original`
- `/cmd_vel_origin`
- `/wheel_odom`
- `/odom`
- `/imu/data`
- `/scan`
- `/ar_pose_marker`
- `/object_position`
- `/formula_id`
- `/enable_identify`
- `/shoot`
- `/target_pose`
- `/move_base/status`

### 典型服务

- `/adjust`：到点精调
- `/block_judge`：比赛区块流转判断

### TF 关系

主要 TF 包括：

- `map`
- `odom`
- `base_footprint`
- `base_link`
- `laser_link`
- `imu_link`

---

## 主要包说明

| 包名 | 作用 | 备注 |
|---|---|---|
| `abot_bringup` | 底盘驱动、里程计、启动入口 | 硬件核心 |
| `abot_imu` | IMU 驱动与原始消息 | 与 `imu_filter_madgwick` 配合 |
| `abot_model` | URDF / Gazebo | 仿真支持 |
| `lidar_filters` | 激光数据过滤 | 雷达预处理 |
| `find_object_2d` | 目标识别 | 外部包整合 |
| `jie_ware` | 自定义激光定位 | 项目特色模块 |
| `pid_follow_planner` | PID 路径跟踪 | 自定义局部规划 |
| `robot_slam` | 建图、导航、任务编排 | 核心包 |
| `track_tag` | AR Tag 跟踪与触发 | 比赛实战模块 |
| `color_pkg` | 颜色赛题 | 线跟随/火焰检测 |
| `face_pkg` | 人脸识别 | 含训练样本 |
| `tracker_pkg` | 视觉跟踪 | KCF/LK/特征跟踪 |
| `robot_voice` | 语音功能 | TTS / ASR |
| `shoot_cmd` | 发射机构控制 | 外设控制 |

---

## 快速开始

### 1）编译

```bash
cd ~/Abot_Cruise_ws
catkin_make
source devel/setup.bash
```

### 2）建图

```bash
./1-gmapping.sh
```

或手动执行：

```bash
roslaunch abot_bringup robot_with_imu.launch
roslaunch robot_slam gmapping.launch
roslaunch robot_slam view_mapping.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 3）保存地图

```bash
roslaunch robot_slam save_map.launch
```

### 4）运行导航赛任务

```bash
./2-navigation.sh
```

或分步执行：

```bash
roslaunch track_tag usb_cam_with_calibration.launch
roslaunch find_object_2d find_object_2d.launch
roslaunch track_tag ar_track_camera.launch
roslaunch abot_bringup robot_with_imu.launch
roslaunch robot_slam navigation.launch
roslaunch robot_slam view_nav.launch
roslaunch robot_slam multi_goal.launch
```

---


## 可能的移植注意事项

我这里总结了一些常见的小注意事项，要是能正常运行则不用管，如果有报错，那就很可能是这部分的问题：

1. **部分脚本依赖图形终端**，`1-gmapping.sh` 与 `2-navigation.sh` 使用旧版 `gnome-terminal -e` 调用方式，新系统上可能需要修改。
2. **Python 兼容性不统一**，直接在 Noetic 下运行可能报错。
3. **存在比赛专用硬件假设**，包括底盘串口、发射机构、雷达、USB 摄像头、摇杆等。
4. **语音模块可能依赖外部 SDK**，建议按照官方说明的安装方法。
5. **部分参数针对比赛场地硬编码**，换场地必须重新标定目标点与中点坐标。

---

## 🙏 Acknowledgments / 致谢

本项目在开发过程中，使用了以下优秀的开源项目/功能包，特此向原作者表示感谢：

* **[JIE WARE ROS 工具集 (jie_ware)](https://github.com/6-robot/jie_ware)**
  * **Author:** 6-Robot (阿杰)
  * **License:** GPL-2.0 License
  * **Usage in this project:** 本项目使用了该工具集提供的基础 2D 激光雷达点云滤波与代价地图清除功能，并在其基础上针对赛事的高速移动场景进行了深度的参数调优与二次开发。
