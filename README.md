# RoboMaster EP 建图与导航工作空间

基于 ROS Noetic 的 DJI RoboMaster EP 自动建图（SLAM）与自主导航系统。

支持外接 HiPNUC HI12 AHRS 模块替换 EP 内置 IMU，解决里程计漂移问题（详见 [docs/hi12_installation_plan.md](docs/hi12_installation_plan.md)）。

## 工作空间结构

```
EP_navigation_Ros1/
└── src/
    ├── rplidar_ros/              思岚 RPLIDAR A2 激光雷达驱动
    ├── rm_ep_driver/             RoboMaster EP ROS 驱动节点 + HI12 驱动 + D435i 深度相机
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
                                                      ├── camera_link
                                                      │   └── d435i_link (RealSense D435i, 可选)
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
# D435i 深度相机（可选；不用 D435i 可不装）
ros-noetic-realsense2-camera
ros-noetic-depthimage-to-laserscan
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
  ros-noetic-teb-local-planner \
  ros-noetic-realsense2-camera \
  ros-noetic-depthimage-to-laserscan
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

更多安装和配置细节见：[HI12 安装方案](docs/hi12_installation_plan.md)。HiPNUC 协议文档下载：https://download.hipnuc.com/products/#attitude

```bash
# 切回 EP 内置 IMU
roslaunch rm_ep_navigation mapping.launch use_hi12:=false enable_imu:=true
```

### D435i 深度相机（可选）

RealSense D435i（RGB + 深度 + IMU）作为可选感知扩展。默认 `use_d435i:=false`，不启用时与建图/导航完全无关。

**硬件接线（关键）**：
- D435i **必须直连上位机的 USB 口**，不能插 EP 底盘的 Type-C 口（那个口连底盘主控板，数据到不了上位机；底盘是封闭固件，无法转发 USB 设备）
- 必须接 **USB 3.0 口**（蓝色）；USB 2.0 下 RGB 不可用、设备反复掉线

**验证 USB 3.0**（接好后）：
```bash
lsusb -t | grep -A1 5000M     # D435i 应在 5000M 链路（USB 2.0 是 480M）
lsusb | grep 8086             # 应为 8086:0b3a（USB 3.0 模式 PID；USB 2.0 是 0ad6）
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_d435i` | `false` | 建图/导航 launch 内嵌启动 D435i |

```bash
# 建图/导航附带 D435i
roslaunch rm_ep_navigation mapping.launch use_d435i:=true
roslaunch rm_ep_navigation navigation.launch use_d435i:=true map_file:=...

# 独立调试 D435i（自动起 URDF + 驱动 + RViz，配置好看 scan）
roslaunch rm_ep_driver d435i_bringup.launch
```

D435i 的深度可转 LaserScan（`/d435i/scan`），作为前方约 86° 的补充感知。

**3D 点云**（已集成）：`d435i_bringup.launch` 自动启动 `depth_to_pointcloud.py`，从深度图生成 PointCloud2（`/camera/depth/points`），带体素降采样（5cm）和跳帧优化。RViz 中可直接查看 3D 点云。

```bash
# 独立启动 D435i（含 2D scan + 3D 点云 + RViz）
roslaunch rm_ep_driver d435i_bringup.launch

# 调整降采样参数
roslaunch rm_ep_driver d435i_bringup.launch voxel_size:=0.1  # 10cm 体素
roslaunch rm_ep_driver d435i_bringup.launch skip_frames:=3   # 每 3 帧发布
```

> 📷 看 RGB 画面建议用 `rqt_image_view /camera/color/image_raw`（轻量）；RViz 主要看 `/d435i/scan` 和 `/camera/depth/points`（Jetson 上 RViz 渲染实时图像会卡）。

更详细的故障排查见 [docs/details.md](docs/details.md)。

#### 3D 彩色点云 + Octomap 建图

D435i 支持生成**累积彩色点云**和 **Octomap 3D 八叉树地图**，可直接在 WebRop 浏览器或 HoloLens2 上查看（不需要 SSH 跑 RViz）。

**3D 彩色点云**（`/d435i/cloud_map`，map 帧，10cm 体素，2Hz）：
- `depth_to_pointcloud.py`：深度图 → xyzrgb 彩色点云（TF 投影取色，无彩色时退化 xyz）。
- `cloud_to_map.py`：点云投到 map 帧（tf2 坐标变换）、体素降采样、**累积拼帧**（多帧拼出完整环境）、**颜色平均**（多次观测取均值，更准更稳定）。
- `d435i_bringup.launch` 自动带起上述两个节点（`use_d435i:=true` 时）。

**Octomap 3D 建图**（`octomap_server`，八叉树 3D 地图）：
- 独立启动：`roslaunch rm_ep_driver d435i_octomap.launch`（自动弹出预配置 RViz）
- 依赖：`sudo apt install ros-noetic-octomap-server ros-noetic-octomap-rviz-plugins`

```bash
# 导航 + 相机（终端1 + 终端2，正常启动）
roslaunch rm_ep_navigation navigation.launch map_name:=你的地图 rviz:=false
roslaunch rm_ep_driver d435i_bringup.launch use_description:=false rviz:=false

# Octomap 3D 建图（终端3，自动弹 RViz）
roslaunch rm_ep_driver d435i_octomap.launch             # 会弹 RViz
roslaunch rm_ep_driver d435i_octomap.launch rviz:=false  # 不弹（用 Foxglove 看）

# 键盘控制走动建图（终端4）
roslaunch rm_ep_driver teleop_keyboard.launch

# 建图完成后，保存 3D 八叉树地图（另开终端）
rosrun octomap_server octomap_saver -f $(rospack find rm_ep_navigation)/maps/3d/地图名

# 加载已有 3D 地图（只可视化，不建图）
roslaunch rm_ep_driver d435i_octomap.launch load_file:=$(rospack find rm_ep_navigation)/maps/3d/地图名.bt
```

**关键话题**：

| 话题 | 帧 | 说明 |
|---|---|---|
| `/d435i/cloud_map` | map | 累积彩色点云（10cm，2Hz，WebRop/HL2 消费） |
| `/camera/depth/points` | camera_depth_optical_frame | 深度点云（xyzrgb，5cm，原帧率，RViz 调试用） |
| `/octomap_binary` / `/octomap_full` | map | 八叉树 3D 地图 |
| `/d435i/scan` | d435i_link | 深度转 LaserScan（进 local_costmap 避障） |
| `/camera/color/image_raw/compressed` | — | D435i RGB 画面（WebRop 深度相机面板消费） |

---

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

地图目录结构：

```
rm_ep_navigation/maps/
├── 2d/                    ← 2D 占用栅格地图（gmapping 建图，AMCL 导航用）
│   ├── 教室/
│   │   ├── 教室.yaml
│   │   └── 教室.pgm
│   └── 20260621_153045/
│       ├── 20260621_153045.yaml
│       └── 20260621_153045.pgm
└── 3d/                    ← 3D 八叉树地图（Octomap 建图，未来 3D 导航用）
    └── 教室.bt
```

建图 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式：`rndis`(USB) / `ap`(直连) / `sta`(路由器) |
| `ep_ip` | (空) | EP IP 地址 |
| `serial_port` | `/dev/rplidar` | 雷达串口（udev 固定别名） |
| `lidar_frame` | `laser_link` | 激光雷达 TF 帧名 |
| `rviz` | `true` | 是否启动 RVIZ |

### 导航

```bash
source ~/EP_navigation_Ros1/devel/setup.bash

# 加载地图并启动导航
roslaunch rm_ep_navigation navigation.launch map_name:=教室 rviz:=false

# 指定绝对路径（优先级高于 map_name）
roslaunch rm_ep_navigation navigation.launch \
  map_file:=~/EP_navigation_Ros1/src/rm_ep_navigation/maps/2d/教室/教室.yaml

# 在 RVIZ 中使用 "2D Nav Goal" 工具点击目标点即可
```

导航 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式 |
| `ep_ip` | (空) | EP IP 地址 |
| `map_file` | `maps/default_map.yaml` | 地图文件路径 |
| `serial_port` | `/dev/rplidar` | 雷达串口（udev 固定别名） |
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

### 查看已有地图

```bash
# 列出所有已保存的地图（每个子文件夹即一张地图）
ls ~/EP_navigation_Ros1/src/rm_ep_navigation/maps/
```

导航时把对应文件夹名填入 `map_file` 参数即可加载，例如：

```bash
roslaunch rm_ep_navigation navigation.launch \
  map_file:=~/EP_navigation_Ros1/src/rm_ep_navigation/maps/教室/教室.yaml
```

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
2. RPLIDAR A2 通过 USB 连接电脑，固定别名 `/dev/rplidar`（安装时线缆朝车头，0° 与 ROS X 轴一致）
3. HI12 外置 IMU 通过 USB-TTL 模块连接电脑，固定别名 `/dev/hi12_imu`（详见 [HI12 安装方案](docs/hi12_installation_plan.md)）
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
