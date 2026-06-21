# AGENTS.md - RoboMaster EP 建图与导航工作空间

## 工作空间类型
ROS Noetic catkin 工作空间，用于 DJI RoboMaster EP 自动建图与导航。

## 关键命令

```bash
# 编译（必须在工作空间根目录执行）
catkin_make

# 每次新终端必须 source
source ~/EP_navigation_Ros1/devel/setup.bash

# 启动底盘驱动（USB 模式，默认）
roslaunch rm_ep_driver rm_ep_chassis_bringup.launch

# 启动建图
roslaunch rm_ep_navigation mapping.launch

# 启动导航
roslaunch rm_ep_navigation navigation.launch map_file:=/path/to/map.yaml

# 保存地图（不指定名称则用时间自动命名）
rosrun rm_ep_navigation save_map.sh [地图名称]

# 键盘遥控
roslaunch rm_ep_driver teleop_keyboard.launch
```

## 包结构与职责

| 包 | 路径 | 语言 | 说明 |
|---|---|---|---|
| `rm_ep_driver` | `src/rm_ep_driver/` | Python 3.8+ | EP 底盘驱动，发布 `/odom`、`/imu`（默认由 HI12 提供），订阅 `/cmd_vel`；含 HI12 驱动节点、键盘遥控 |
| `rm_ep_navigation` | `src/rm_ep_navigation/` | 纯配置 | 建图(gmapping)、导航(AMCL+TEB)、EKF融合（HI12 提供绝对航向） |
| `rm_ep_description` | `src/rm_ep_description/` | 纯配置 | URDF 模型与 STL 网格，定义 TF 树 |
| `rplidar_ros` | `src/rplidar_ros/` | C++ (C++11) | RPLIDAR A2 激光雷达驱动，自带 SDK 源码编译 |

## 入口点

- **驱动主节点**: `src/rm_ep_driver/scripts/rm_ep_driver_node.py` (RmEpDriver 类)
- **HI12 驱动节点**: `src/rm_ep_driver/scripts/hi12_imu_node.py` (读取 HI12 串口，发布 `/imu`)
- **键盘遥控**: `src/rm_ep_driver/scripts/ep_teleop_keyboard.py` (发布 `/cmd_vel`)
- **底盘 launch**: `src/rm_ep_driver/launch/rm_ep_chassis_bringup.launch` (新建图/导航流程使用)
- **旧版底盘 launch**: `src/rm_ep_driver/launch/rm_ep_bringup.launch` (通过 yaml 加载参数)
- **建图 launch**: `src/rm_ep_navigation/launch/mapping.launch`
- **导航 launch**: `src/rm_ep_navigation/launch/navigation.launch`

## 配置文件

- `src/rm_ep_driver/config/rm_ep_params.yaml` - EP 连接参数、HI12 串口参数、IMU 开关
- `src/rm_ep_navigation/config/ekf.yaml` - EKF 融合（odom + HI12 IMU，绝对航向模式）
- `src/rm_ep_navigation/config/gmapping_params.yaml` - gmapping SLAM 参数
- `src/rm_ep_navigation/config/amcl_params.yaml` - AMCL 定位参数（omni-corrected 里程计模型）
- `src/rm_ep_navigation/config/costmap_common_params.yaml` - 通用代价地图
- `src/rm_ep_navigation/config/teb_local_planner_params.yaml` - TEB 全向规划器

## 硬件依赖

- DJI RoboMaster EP（需安装 `pip3 install robomaster`）
- RPLIDAR A2 激光雷达（串口 `/dev/ttyUSB0`，波特率 256000）
- HiPNUC HI12 AHRS 外置 IMU（通过 USB-TTL 连接，默认 `/dev/hi12_imu`，详见 `docs/hi12_installation_plan.md`）
- EP 连接模式：`rndis`(USB，默认) / `ap`(WiFi直连) / `sta`(路由器)

## TF 树结构

```
map ──(gmapping/amcl)──► odom ──(EKF)──► base_link ──┬── laser_link
                                                      ├── imu_link
                                                      ├── chassis_base_link
                                                      │   └── arm → camera
                                                      └── wheels (4个麦轮)
```

**重要**：底盘驱动不发布 TF，由 EKF 统一发布 `odom→base_link`。

## SDK 坐标系

SDK 坐标系与 ROS REP-103 差异：y 轴方向相反，yaw 方向相反。驱动映射（与 ROS2 一致）：
- 位置：`x=px, y=-py`
- 速度：`vx=vgx, vy=-vgy`（世界坐标系）
- 姿态：`yaw=-yaw_deg, pitch=-pitch_deg, roll=roll_deg`
- cmd_vel：`x=x, y=-y, z=-z`

**修改任何坐标映射时必须保持 odom 和 cmd_vel 一致。**

> 以上坐标映射仅适用于 EP SDK 数据。HI12 外置 IMU 不经过 SDK，直接输出标准物理量，无需坐标变换。

SDK 使用 `is` 比较字符串，必须使用 SDK 常量对象（`rm_conn.CONNECTION_USB_RNDIS` 等）。

## 重要约定

- 所有 Python 脚本使用 `#!/usr/bin/env python3`
- 无测试框架、无 lint 配置、无 CI 流程
- 构建产物 (`build/`, `devel/`) 已在 `.gitignore` 排除
- 地图文件保存在 `src/rm_ep_navigation/maps/<名称>/` 子文件夹
- 驱动节点基于 ROS2 移植，坐标映射与 ROS2 完全一致

## 常见问题

- **SDK 未安装**: `pip3 install robomaster`
- **串口权限**: `sudo usermod -a -G dialout $USER` 后重新登录
- **EP 连接失败**: 检查 SN 号，或尝试指定 IP `ep_ip:=192.168.x.x`
- **TF 树异常**: `rosrun tf view_frames` 检查，确认 EKF 节点运行正常
