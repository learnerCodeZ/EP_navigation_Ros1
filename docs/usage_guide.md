# 使用指南

本文档从零开始，指导你完成环境搭建、硬件连接、编译、建图和导航的完整流程。

---

## 一、环境准备

### 1.1 系统要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | Noetic |
| Python | 3.8+ |

### 1.2 安装 ROS Noetic

如尚未安装 ROS Noetic，参考 [官方安装指南](http://wiki.ros.org/noetic/Installation/Ubuntu)。

安装完成后设置环境：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.3 安装 ROS 依赖包

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

### 1.4 安装 Python 依赖

```bash
pip3 install robomaster pyserial

# 国内镜像加速
pip3 install robomaster pyserial -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 1.5 获取代码

```bash
cd ~
git clone https://github.com/learnerCodeZ/EP_navigation_Ros1.git EP_navigation_Ros1
cd EP_navigation_Ros1
```

### 1.6 编译

```bash
cd ~/EP_navigation_Ros1
catkin_make
source devel/setup.bash

# 建议写入 bashrc，避免每次手动 source
echo "source ~/EP_navigation_Ros1/devel/setup.bash" >> ~/.bashrc
```

---

## 二、硬件连接

### 2.1 连接清单

| 设备 | 连接方式 | 串口设备 |
|------|----------|----------|
| RoboMaster EP | USB 线（RNDIS 模式）或 WiFi | — |
| RPLIDAR A2 激光雷达 | USB | `/dev/ttyUSB0` |
| HiPNUC HI12 AHRS | USB-TTL 模块 + 杜邦线 | `/dev/hi12_imu` |

### 2.2 EP 连接

默认使用 USB 线直连（RNDIS 模式），将 USB 线一端插 EP，另一端插电脑即可。

如使用 WiFi 连接，需根据模式选择：

| 模式 | 说明 | 参数 |
|------|------|------|
| USB 直连 | USB 线连接，无需 WiFi | `ep_conn_type:=rndis`（默认） |
| WiFi 直连 | 电脑连接 EP 的 WiFi 热点 | `ep_conn_type:=ap` |
| 路由器 | EP 和电脑连接同一路由器 | `ep_conn_type:=sta` |

### 2.3 激光雷达连接

将 RPLIDAR A2 通过 USB 连接电脑，确认设备识别：

> **安装方向**：雷达线缆朝车头安装，0° 扫描方向与 ROS X 轴正方向一致，无需翻转补偿。

```bash
ls /dev/ttyUSB*
```

为确保雷达和 HI12 同时插入时设备号不漂移，需配置 udev 固定别名：

```bash
# 查看雷达序列号
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial

# 创建雷达 udev 规则（替换雷达的实际序列号）
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<雷达序列号>", SYMLINK+="rplidar", MODE="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules

sudo udevadm control --reload-rules
sudo udevadm trigger

# 确认别名存在
ls -l /dev/rplidar
```

如果提示权限不足：

```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 2.4 HI12 外置 IMU 连接

1. 用杜邦线连接 HI12 和 USB-TTL 模块：

   | USB-TTL | HI12 |
   |---------|------|
   | 5V | VCC |
   | GND | GND |
   | RX | TX |
   | TX | RX |

2. 将 USB-TTL 插入电脑 USB 口

3. 确认设备识别并配置权限：

   ```bash
   ls /dev/ttyUSB*
   sudo usermod -a -G dialout $USER
   ```

4. 配置 udev 固定别名（推荐，避免设备号漂移）：

   ```bash
   # CP2102 芯片
   echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="hi12_imu", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hipnuc-hi12.rules

   # CH340 芯片
   # echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="hi12_imu", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hipnuc-hi12.rules

   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

5. 确认别名生效：

   ```bash
   ls -l /dev/hi12_imu
   ```

> 详细的 HI12 安装方案（安装位置、坐标系对齐、磁力计校准）见 [hi12_installation_plan.md](hi12_installation_plan.md)。

---

## 三、快速启动

### 3.1 启动前检查

- [ ] EP 已开机，USB 线/WiFi 已连接
- [ ] 激光雷达 USB 已连接
- [ ] HI12 的 USB-TTL 已连接
- [ ] 已执行 `source ~/EP_navigation_Ros1/devel/setup.bash`

### 3.2 建图

```bash
roslaunch rm_ep_navigation mapping.launch
```

RVIZ 会自动打开，显示激光扫描和建图过程。用遥控器或键盘控制 EP 遍历环境：

```bash
# 键盘控制（另开终端）
roslaunch rm_ep_driver teleop_keyboard.launch
```

键盘布局（麦轮全向控制）：

```
  u  i  o      前左转 前 前右转
  j  k  l  =>  左转   停 右转
  m  ,  .      后左转 后 后右转
```

空格急停，r 切换速度档位。

建图完成后保存地图：

```bash
# 另开终端，指定名称
rosrun rm_ep_navigation save_map.sh 教室

# 不指定名称则用时间自动命名（如 20260621_153045）
rosrun rm_ep_navigation save_map.sh
```

地图保存在 `maps/<名称>/` 子文件夹下，如 `maps/教室/教室.yaml` 和 `maps/教室/教室.pgm`。

### 3.3 导航

```bash
roslaunch rm_ep_navigation navigation.launch \
  map_file:=~/EP_navigation_Ros1/src/rm_ep_navigation/maps/教室/教室.yaml
```

RVIZ 打开后：

1. 点击工具栏 **"2D Pose Estimate"**，在地图上点击机器人当前位置并拖动指定方向（初始化位姿）
2. 点击工具栏 **"2D Nav Goal"**，在地图上点击目标位置并拖动指定到达方向
3. EP 将自动规划路径并导航到目标点

---

## 四、Launch 参数

### 4.1 通用参数（建图/导航共用）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式：`rndis`(USB) / `ap`(WiFi直连) / `sta`(路由器) |
| `ep_ip` | (空) | EP IP 地址，留空则通过 SN 自动发现 |
| `serial_port` | `/dev/ttyUSB0` | 激光雷达串口 |
| `rviz` | `true` | 是否启动 RVIZ |
| `use_hi12` | `true` | 是否使用 HI12 外置 IMU |
| `hi12_port` | `/dev/hi12_imu` | HI12 串口设备 |
| `hi12_baud` | `115200` | HI12 波特率 |
| `imu_frame` | `imu_link` | IMU TF 帧名 |

### 4.2 导航专属参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_file` | `maps/default_map.yaml` | 地图文件路径 |

### 4.3 底盘驱动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_cmd_vel` | `true` | 是否接收 `/cmd_vel` 速度指令 |
| `enable_imu` | `false` | 是否启用 EP 内置 IMU（使用 HI12 时为 false） |
| `odom_rate` | `20` | 里程计发布频率 (Hz) |
| `cmd_vel_timeout` | `0.5` | 速度指令超时自动停车 (秒) |

---

## 五、常用场景

### 5.1 指定 EP 连接模式

```bash
# WiFi 直连模式
roslaunch rm_ep_navigation mapping.launch ep_conn_type:=ap

# 路由器模式，指定 EP IP
roslaunch rm_ep_navigation mapping.launch ep_conn_type:=sta ep_ip:=192.168.1.10
```

### 5.2 指定激光雷达串口

当雷达不是 `/dev/ttyUSB0` 时（例如 HI12 占用了 ttyUSB0）：

```bash
roslaunch rm_ep_navigation mapping.launch serial_port:=/dev/ttyUSB1
```

### 5.3 不启动 RVIZ（远程 SSH 场景）

```bash
roslaunch rm_ep_navigation mapping.launch rviz:=false
```

### 5.4 切换回 EP 内置 IMU

如果 HI12 未连接或需要回退到 EP 内置 IMU：

```bash
roslaunch rm_ep_navigation mapping.launch use_hi12:=false enable_imu:=true
```

> 注意：`use_hi12:=false` 会同时将 `enable_imu` 设为 `true`，即启用 EP 内置 IMU 发布 `/imu`。

### 5.5 单独测试底盘驱动

```bash
roslaunch rm_ep_driver rm_ep_chassis_bringup.launch
```

### 5.6 查看 IMU 数据

```bash
# 查看实时数据
rostopic echo /imu

# 查看发布频率
rostopic hz /imu
```

### 5.7 查看 TF 树

```bash
rosrun tf view_frames
# 生成 frames.pdf，用浏览器打开
xdg-open frames.pdf
```

---

## 六、数据流说明

### 6.1 话题

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/odom` | `nav_msgs/Odometry` | rm_ep_driver | EP 里程计 |
| `/imu` | `sensor_msgs/Imu` | hi12_imu_node | HI12 姿态数据（默认） |
| `/scan` | `sensor_msgs/LaserScan` | rplidar_ros | 激光雷达扫描 |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop / move_base | 速度指令 |
| `/odometry/filtered` | `nav_msgs/Odometry` | ekf_localization | EKF 融合后的里程计 |

### 6.2 TF 树

```
map ──(amcl/gmapping)──► odom ──(EKF)──► base_link ──┬── laser_link
                                                      ├── imu_link
                                                      ├── chassis_base_link
                                                      └── wheels (4个麦轮)
```

- `map → odom`：由 AMCL（导航模式）或 gmapping（建图模式）发布
- `odom → base_link`：由 EKF (robot_localization) 统一发布
- `base_link → *`：由 robot_state_publisher 根据 URDF 发布

### 6.3 EKF 融合策略

| 数据源 | 融合内容 |
|--------|----------|
| `/odom` (EP 里程计) | 绝对位置 X, Y + 世界坐标系速度 vx, vy + 角速度 vyaw |
| `/imu` (HI12) | 绝对 Yaw 角（磁力计）+ 角速度 vyaw + 加速度 ax, ay |

HI12 有磁力计提供绝对航向，EKF 使用 `imu0_relative: false`，无需像 EP 内置 IMU 那样将上电方向视为 0 度。

---

## 七、配置文件

所有配置文件位于 `src/rm_ep_navigation/config/` 和 `src/rm_ep_driver/config/`。

| 文件 | 用途 |
|------|------|
| `rm_ep_params.yaml` | EP 连接参数、IMU 开关、HI12 串口 |
| `ekf.yaml` | EKF 融合策略 |
| `gmapping_params.yaml` | gmapping SLAM 参数 |
| `amcl_params.yaml` | AMCL 定位参数 |
| `costmap_common_params.yaml` | 通用代价地图 |
| `global_costmap_params.yaml` | 全局代价地图 |
| `local_costmap_params.yaml` | 局部代价地图 |
| `teb_local_planner_params.yaml` | TEB 局部规划器 |
| `move_base_params.yaml` | move_base 框架参数 |

---

## 八、常见问题

### 编译失败

```bash
cd ~/EP_navigation_Ros1
rm -rf build/ devel/
catkin_make
```

### EP 连接不上

- 确认 EP 已开机
- USB 模式确认 USB 线已连接
- WiFi 模式确认电脑已连接 EP 热点或同一路由器
- 检查 SN 号是否正确，或尝试指定 IP：`ep_ip:=192.168.x.x`

### 串口权限不足

```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### HI12 不工作

```bash
# 检查串口设备
ls /dev/hi12_imu /dev/ttyUSB*
# 检查 IMU 数据
rostopic echo /imu
# 检查发布频率
rostopic hz /imu
```

### 里程计漂移

EP 麦轮在光滑地面容易打滑。使用 HI12 外置 IMU 可提供更准确的航向参考。建图时尽量低速平稳移动。

### TEB 报错

```bash
dpkg -l | grep teb-local-planner
# 如未安装
sudo apt install ros-noetic-teb-local-planner
```

### TF 树异常

```bash
rosrun tf view_frames
rosrun tf tf_echo odom base_link
```
