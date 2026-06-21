# RoboMaster EP 建图与导航工作空间

基于 ROS Noetic 的 DJI RoboMaster EP 自动建图（SLAM）与自主导航系统。

支持外接 HiPNUC HI12 AHRS 模块替换 EP 内置 IMU，解决里程计漂移问题（详见 [docs/hi12_installation_plan.md](docs/hi12_installation_plan.md)）。

## 工作空间结构

```
EP_navigation_Ros1/
└── src/
    ├── rplidar_ros/              思岚 RPLIDAR A2 激光雷达驱动
    ├── rm_ep_driver/             RoboMaster EP ROS 驱动节点 + HI12 驱动
    ├── rm_ep_description/        EP 机器人 URDF 模型
    └── rm_ep_navigation/         建图与导航配置包
```

## 功能包说明

### 1. rm_ep_driver — EP 驱动节点

封装 DJI RoboMaster SDK，桥接 ROS 与 EP 硬件（基于 ROS2 驱动移植，坐标映射与 ROS2 一致）。

| 数据 | 话题 | 方向 | 说明 |
|------|------|------|------|
| 里程计 | `/odom` | 发布 | 底盘编码器推算，frame_id=`odom`，child=`base_link` |
| IMU | `/imu` | 发布 | 姿态 + 角速度 + 加速度（默认由 HI12 提供，可通过参数切回 EP 内置 IMU），frame_id=`imu_link` |
| 速度指令 | `/cmd_vel` | 订阅 | 转为 EP 全向麦轮控制 |
| HI12 IMU 数据 | `/imu` | 发布 | hi12_imu_node.py，读取 HI12 串口输出，发布标准 IMU 消息 |

驱动支持两种速度控制模式：
- **底盘速度模式**（默认）：`drive_speed(x, y, z)`，直接发送底盘速度
- **麦轮速度模式**：`twist_to_wheel_speeds:=true`，将 twist 转换为四轮 RPM

### 2. rm_ep_description — 机器人模型

URDF/XACRO 模型，定义 TF 树：

```
map ──(gmapping/amcl)──► odom ──(EKF)──► base_link ──┬── laser_link
                                                      ├── imu_link
                                                      ├── chassis_base_link
                                                      │   └── arm → camera
                                                      └── wheels (4个麦轮)
```

**重要**：底盘驱动不发布 TF，由 EKF (robot_localization) 统一发布 `odom→base_link`。

### 3. rm_ep_navigation — 建图与导航

| 模式 | launch 文件 | 核心算法 |
|------|------------|----------|
| 建图 | `mapping.launch` | gmapping SLAM |
| 导航 | `navigation.launch` | AMCL 定位 + TEB 全向规划 |
| 里程融合 | 内置 | robot_localization EKF（IMU + 里程计） |

## 环境依赖

### 系统要求

| 项目 | 版本 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| Python | 3.8+ |

### ROS 包依赖

```
ros-noetic-gmapping
ros-noetic-amcl
ros-noetic-move-base
ros-noetic-map-server
ros-noetic-robot-state-publisher
ros-noetic-joint-state-publisher-gui
ros-noetic-robot-localization
ros-noetic-teb-local-planner
```

### Python 依赖

```
pip3 install robomaster
```

## 安装

### 1. 安装 ROS 依赖

```bash
sudo apt install -y \
  ros-noetic-gmapping \
  ros-noetic-amcl \
  ros-noetic-move-base \
  ros-noetic-map-server \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-localization \
  ros-noetic-teb-local-planner
```

### 2. 安装 Python SDK

```bash
# 官方源
pip3 install robomaster

# 国内镜像（更快）
pip3 install robomaster -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 3. 编译

```bash
cd ~/EP_navigation_Ros1
catkin_make
source devel/setup.bash
```

## 使用方法

### 底盘控制

```bash
source ~/EP_navigation_Ros1/devel/setup.bash

# 启动底盘驱动（USB 模式，默认）
roslaunch rm_ep_driver rm_ep_chassis_bringup.launch

# 指定 SN
roslaunch rm_ep_driver rm_ep_chassis_bringup.launch ep_sn:=3JKDH3B001891M

# WiFi 直连模式
roslaunch rm_ep_driver rm_ep_chassis_bringup.launch ep_conn_type:=ap

# 路由器模式
roslaunch rm_ep_driver rm_ep_chassis_bringup.launch ep_conn_type:=sta
```

底盘驱动 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式：`rndis`(USB) / `ap`(WiFi直连) / `sta`(路由器) |
| `ep_ip` | (空) | EP IP 地址（留空则通过 SN 自动发现） |
| `enable_cmd_vel` | `true` | 是否启用 `/cmd_vel` |
| `odom_rate` | `20` | 里程计发布频率 (Hz) |
| `enable_imu` | `false` | 是否启用 EP 内置 IMU（使用 HI12 时禁用） |

### HI12 外置 IMU

项目默认使用 HiPNUC HI12 AHRS 模块替代 EP 内置 IMU。HI12 通过 USB-TTL 连接上位机，提供 9 轴融合姿态数据。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hi12_port` | `/dev/hi12_imu` | HI12 串口设备 |
| `hi12_baud` | `115200` | HI12 波特率 |
| `use_hi12` | `true` | 是否使用 HI12（false 则切回 EP 内置 IMU） |

更多安装和配置细节见：[HI12 安装方案](docs/hi12_installation_plan.md)。

```bash
# 切回 EP 内置 IMU
roslaunch rm_ep_navigation mapping.launch use_hi12:=false enable_imu:=true
```

### EP 连接模式

| 模式 | 参数 | 说明 |
|------|------|------|
| USB | `ep_conn_type:=rndis` | USB 线直连，无需 WiFi，**默认模式** |
| WiFi 直连 | `ep_conn_type:=ap` | 电脑连接 EP 自带 WiFi 热点 |
| 路由器 | `ep_conn_type:=sta` | EP + 电脑连接同一路由器 |

### 建图

```bash
source ~/EP_navigation_Ros1/devel/setup.bash

# 启动建图（USB 模式）
roslaunch rm_ep_navigation mapping.launch

# 指定 SN 或其他连接模式
roslaunch rm_ep_navigation mapping.launch ep_sn:=YOUR_EP_SN ep_conn_type:=sta

# 键盘遥控（另开终端）
roslaunch rm_ep_driver teleop_keyboard.launch
```

键盘布局（麦轮全向控制）：

```
  u  i  o      前左转 前 前右转
  j  k  l  =>  左转   停 右转
  m  ,  .      后左转 后 后右转
```

空格急停，r 切换速度档位。

地图满意后保存：

```bash
# 另开终端，指定名称
rosrun rm_ep_navigation save_map.sh 教室

# 不指定名称则用时间自动命名（如 20260621_153045）
rosrun rm_ep_navigation save_map.sh
```

建图 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式：`rndis`(USB) / `ap`(直连) / `sta`(路由器) |
| `ep_ip` | (空) | EP IP 地址 |
| `serial_port` | `/dev/ttyUSB0` | 雷达串口号 |
| `lidar_frame` | `laser_link` | 激光雷达 TF 帧名 |
| `rviz` | `true` | 是否启动 RVIZ |

### 导航

```bash
source ~/EP_navigation_Ros1/devel/setup.bash

# 加载地图并启动导航
roslaunch rm_ep_navigation navigation.launch \
  map_file:=~/EP_navigation_Ros1/src/rm_ep_navigation/maps/教室/教室.yaml

# 在 RVIZ 中使用 "2D Nav Goal" 工具点击目标点即可
```

导航 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式 |
| `ep_ip` | (空) | EP IP 地址 |
| `map_file` | `maps/default_map.yaml` | 地图文件路径 |
| `serial_port` | `/dev/ttyUSB0` | 雷达串口号 |
| `lidar_frame` | `laser_link` | 激光雷达 TF 帧名 |
| `rviz` | `true` | 是否启动 RVIZ |

### 地图保存脚本

```bash
# 保存为指定名称
rosrun rm_ep_navigation save_map.sh 教室

# 不指定名称则用时间自动命名（如 20260621_153045）
rosrun rm_ep_navigation save_map.sh
```

地图保存在 `maps/<名称>/` 子文件夹下，如 `maps/教室/教室.yaml` 和 `maps/教室/教室.pgm`。

## 配置调优

所有参数配置文件位于 `rm_ep_navigation/config/`：

| 文件 | 用途 |
|------|------|
| `ekf.yaml` | 里程计 + IMU EKF 融合 |
| `gmapping_params.yaml` | gmapping SLAM 参数 |
| `amcl_params.yaml` | AMCL 定位参数 |
| `costmap_common_params.yaml` | 通用代价地图 |
| `global_costmap_params.yaml` | 全局代价地图 |
| `local_costmap_params.yaml` | 局部代价地图 |
| `teb_local_planner_params.yaml` | TEB 全向规划器 |
| `move_base_params.yaml` | move_base 框架参数 |

驱动参数配置文件位于 `rm_ep_driver/config/`：

| 文件 | 用途 |
|------|------|
| `rm_ep_params.yaml` | EP 连接参数（SN、连接类型、IMU、TF 帧名等） |

### 关键调参项

**速度限制** (`teb_local_planner_params.yaml`)：

```yaml
max_vel_x: 0.8       # 前向最大速度 (m/s)
max_vel_y: 0.3       # 横向最大速度 (m/s)，麦轮特有
max_vel_theta: 1.0   # 最大旋转速度 (rad/s)
```

**机器人足迹** (`costmap_common_params.yaml` + `teb_local_planner_params.yaml`)：

```yaml
footprint: [[-0.14, -0.11], [-0.14, 0.11], [0.14, 0.11], [0.14, -0.11]]
```

**障碍物安全距离** (`teb_local_planner_params.yaml`)：

```yaml
min_obstacle_dist: 0.15   # 最小障碍物距离 (m)
inflation_radius: 0.30    # 膨胀半径 (m)，在 costmap_common_params 中
```

**EKF 融合策略** (`ekf.yaml`)：

- **odom**：绝对位置 X,Y + 世界坐标系速度 vx,vy + 角速度 vyaw
- **IMU（HI12）**：绝对 Yaw 角（磁力计提供）+ 角速度 vyaw + 加速度 ax,ay
- `imu0_relative: false`：使用绝对航向（HI12 有磁力计，无需相对模式）

**AMCL 定位** (`amcl_params.yaml`)：

- 粒子数：100 ~ 2000（自适应）
- 激光模型：`likelihood_field`
- 里程计模型：`omni-corrected`（适配麦轮全向运动）

## SDK 坐标系注意事项

RoboMaster SDK 坐标系与 ROS REP-103 标准的差异：

- **y 轴方向相反**：SDK y 正=右，ROS y 正=左
- **yaw 方向相反**：SDK 顺时针正，ROS 逆时针正

驱动中的映射（与 ROS2 一致）：

| 数据 | 映射 |
|------|------|
| 位置 | `x=px, y=-py` |
| 速度 | `vx=vgx, vy=-vgy`（世界坐标系） |
| 姿态 | `yaw=-yaw_deg, pitch=-pitch_deg, roll=roll_deg` |
| IMU | `acc_y=-acc_y, acc_z=-acc_z, gyro_y=-gyro_y, gyro_z=-gyro_z` |
| cmd_vel | `x=x, y=-y, z=-z` |

**修改任何坐标映射时必须保持 odom 和 cmd_vel 一致。**

> **注意**：以上坐标映射仅适用于 EP SDK 获取的数据。外置 HI12 IMU 不经过 EP SDK，直接输出标准物理量，**无需任何坐标变换**。只要硬件安装时 HI12 坐标系与 `base_link` 对齐即可。

SDK 使用 `is` 比较字符串，驱动必须使用 SDK 常量对象：

```python
from robomaster import conn as rm_conn
conn_type_map = {
    'ap': rm_conn.CONNECTION_WIFI_AP,
    'sta': rm_conn.CONNECTION_WIFI_STA,
    'rndis': rm_conn.CONNECTION_USB_RNDIS,
}
```

## 硬件连接

1. EP 通过 USB 线连接电脑（RNDIS 模式，默认），或通过 WiFi 连接同一路由器（STA 模式）
2. RPLIDAR A2 通过 USB 连接电脑，默认串口 `/dev/ttyUSB0`（安装时线缆朝车头，0° 与 ROS X 轴一致）
3. HI12 外置 IMU 通过 USB-TTL 模块连接电脑，默认 `/dev/hi12_imu`（详见 [HI12 安装方案](docs/hi12_installation_plan.md)）
4. 如需指定其他串口，在 launch 中添加 `serial_port:=/dev/ttyUSB1`

## 常见问题

**Q: 驱动节点启动失败，提示 "RoboMaster SDK 不可用"**

```bash
pip3 install robomaster
```

**Q: EP 连接不上**

确认 EP 已开机。USB 模式确认 USB 线已连接；WiFi 直连模式确认电脑已连接 EP 热点。检查 SN 号是否正确，或尝试指定 IP：

```bash
roslaunch rm_ep_navigation mapping.launch ep_ip:=192.168.x.x
```

**Q: 雷达不工作**

```bash
# 检查串口设备
ls /dev/ttyUSB*
# 检查权限
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

**Q: 里程计漂移严重**

EP 麦轮在光滑地面容易打滑。项目默认使用外置 HI12 IMU 提供更准确的航向参考。如使用 EP 内置 IMU，建图时尽量低速平稳移动。

**Q: HI12 IMU 不工作**

```bash
# 检查串口设备（确认 USB-TTL 已连接）
ls /dev/hi12_imu /dev/ttyUSB*
# 检查 IMU 数据
rostopic echo /imu

**Q: 导航时 TEB 报错**

确认 `ros-noetic-teb-local-planner` 已安装：

```bash
dpkg -l | grep teb-local-planner
```

**Q: TF 树异常**

```bash
# 检查当前 TF 树
rosrun tf view_frames
# 查看具体两个帧之间的变换
rosrun tf tf_echo odom base_link
```

## RVIZ 快捷键

| 操作 | 快捷键 |
|------|--------|
| 设置初始位姿 | 工具栏 "2D Pose Estimate" |
| 设置导航目标 | 工具栏 "2D Nav Goal" |
| 旋转视角 | 鼠标左键拖拽 |
| 平移视角 | 鼠标中键拖拽 |
| 缩放 | 滚轮 |
