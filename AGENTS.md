# AGENTS.md - RoboMaster EP 建图与导航工作空间

## 工作空间类型
ROS Noetic catkin 工作空间，用于 DJI RoboMaster EP 自动建图与导航。

## 关键命令

```bash
# 编译（必须在工作空间根目录执行）
catkin_make

# 每次新终端必须 source
source ~/catkin_ws/devel/setup.bash

# 启动底盘驱动
roslaunch rm_ep_driver rm_ep_bringup.launch

# 启动建图（需要指定 EP SN）
roslaunch rm_ep_navigation mapping.launch ep_sn:=YOUR_SN

# 启动导航
roslaunch rm_ep_navigation navigation.launch ep_sn:=YOUR_SN map_file:=/path/to/map.yaml

# 保存地图
rosrun rm_ep_navigation save_map.sh [地图名称]
```

## 包结构与职责

| 包 | 路径 | 说明 |
|---|---|---|
| `rm_ep_driver` | `src/rm_ep_driver/` | EP 底盘驱动，发布 `/odom`、`/imu`，订阅 `/cmd_vel` |
| `rm_ep_navigation` | `src/rm_ep_navigation/` | 建图(gmapping)、导航(AMCL+TEB)、EKF融合 |
| `rm_ep_description` | `src/rm_ep_description/` | URDF 模型，定义 TF 树 |
| `rplidar_ros` | `src/rplidar_ros/` | RPLIDAR A2 激光雷达驱动 |

## 入口点

- **驱动主节点**: `src/rm_ep_driver/scripts/rm_ep_driver_node.py` (RmEpDriver 类)
- **建图 launch**: `src/rm_ep_navigation/launch/mapping.launch`
- **导航 launch**: `src/rm_ep_navigation/launch/navigation.launch`
- **底盘 launch**: `src/rm_ep_driver/launch/rm_ep_bringup.launch`

## 配置文件

- `src/rm_ep_driver/config/rm_ep_params.yaml` - EP 连接参数（SN、连接类型）
- `src/rm_ep_navigation/config/` - 导航参数（EKF、gmapping、AMCL、costmap、TEB）

## 硬件依赖

- DJI RoboMaster EP（需安装 `pip3 install robomaster`）
- RPLIDAR A2 激光雷达（串口 `/dev/ttyUSB0`）
- EP 连接模式：`ap`(直连) 或 `sta`(路由器)

## TF 树结构

```
map ──(gmapping/amcl)──► odom ──(EKF)──► base_link ──┬── laser_link
                                                      └── imu_link
```

## 重要约定

- 所有 Python 脚本使用 `#!/usr/bin/env python3`
- 无测试框架、无 lint 配置、无 CI 流程
- 构建产物 (`build/`, `devel/`) 已在 `.gitignore` 排除
- 地图文件保存在 `src/rm_ep_navigation/maps/`

## 常见问题

- **SDK 未安装**: `pip3 install robomaster`
- **串口权限**: `sudo usermod -a -G dialout $USER` 后重新登录
- **EP 连接失败**: 检查 SN 号，或尝试指定 IP `ep_ip:=192.168.x.x`
