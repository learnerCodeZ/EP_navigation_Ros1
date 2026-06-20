# HiPNUC HI12 AHRS 外置 IMU 安装方案

## 一、为什么需要安装外置 IMU

### 1.1 当前问题

当前系统使用 RoboMaster EP 内置 IMU，通过 SDK 获取加速度计和陀螺仪数据，经坐标映射后发布到 `/imu` 话题。**实际使用中发现以下问题**：

- **yaw 角漂移**：EP 内置 IMU 缺乏磁力计校正，yaw 角随时间累积漂移严重，导致 EKF 融合后的方向估计不准确
- **电机干扰**：EP 底盘内部 IMU 距离电机和电调极近，大电流产生的磁场严重干扰 IMU 的加速度计和陀螺仪读数，电机启动/变速时数据出现跳变
- **麦轮打滑加剧**：EP 麦轮在光滑地面容易打滑，里程计漂移大，EKF 高度依赖 IMU 的 yaw 角来修正方向，但 IMU 本身不可靠时，整个位姿估计链崩溃
- **无绝对方向参考**：EP IMU 没有磁力计，无法提供绝对航向，每次上电 yaw=0 只能提供相对方向，长时间运行后方向偏差无法自愈

这些问题在项目配置中已有所体现：
- `ekf.yaml` 中 `imu0_relative: true` 被迫开启，因为 EP IMU 无法提供绝对航向
- `details.md` 中记录"里程计漂移严重，EP 麦轮在光滑地面容易打滑"
- EKF 只能融合 IMU 的 yaw 姿态和 vyaw 角速度，但数据质量差导致融合效果不佳

### 1.2 为什么选择 HiPNUC HI12

| 对比项 | EP 内置 IMU | HiPNUC HI12 |
|--------|-------------|-------------|
| 传感器类型 | 6 轴（加速度计+陀螺仪） | 9 轴（加速度计+陀螺仪+磁力计） |
| 姿态解算 | 无（需自行滤波） | 内置 EKF/卡尔曼滤波算法，直接输出四元数/欧拉角 |
| 绝对航向 | 无磁力计，无法提供 | 有磁力计，可提供绝对航向 |
| 安装位置 | 底盘内部，紧邻电机 | 可外置安装，远离干扰源 |
| 输出频率 | 20 Hz（受 SDK 限制） | 最高 200 Hz |
| 通信接口 | SDK 内部获取 | UART 串口，标准协议 |
| 数据质量 | 电机干扰大，跳变明显 | 工业级传感器，低噪声，抗干扰 |

**核心优势**：HI12 自带姿态融合算法，输出的是已经过卡尔曼滤波的稳定姿态数据，无需在 ROS 端做额外滤波；同时磁力计提供绝对航向，从根本上解决 yaw 漂移问题。

---

## 二、HiPNUC HI12 产品规格

> 以下规格来自 HiPNUC 官方资料，最终以实物和官方文档为准。
> 官方下载页：https://download.hipnuc.com/products/#attitude

### 2.1 基本参数

| 参数 | 值 |
|------|-----|
| 产品型号 | HI12 |
| 产品类型 | AHRS（航姿参考系统） |
| 传感器 | 9 轴（3 轴加速度计 + 3 轴陀螺仪 + 3 轴磁力计） |
| 姿态解算 | 内置 EKF 融合算法，输出四元数和欧拉角 |
| 通信接口 | UART（TTL 电平，3.3V） |
| 默认波特率 | 115200 bps（可配置 9600~921600） |
| 数据输出频率 | 1~200 Hz（可配置） |
| 供电电压 | 3.3V ~ 5.0V |
| 工作电流 | < 30 mA |
| 工作温度 | -40°C ~ +85°C |

### 2.2 性能指标

| 指标 | 值 |
|------|-----|
| 姿态角精度（静态） | Roll/Pitch < 0.3°, Yaw < 1.0° |
| 陀螺仪零偏稳定性 | < 10°/h |
| 加速度计零偏稳定性 | < 0.5 mg |
| 磁力计精度 | 支持硬磁/软磁校准 |
| 启动时间 | < 0.5s |

### 2.3 物理尺寸

| 参数 | 值 |
|------|-----|
| 尺寸 | 约 22mm x 22mm x 8mm |
| 重量 | 约 5g |

### 2.4 引脚定义

HI12 通常提供以下引脚（以实际模块排针为准）：

| 引脚 | 功能 | 连接到 |
|------|------|--------|
| VCC | 电源输入（3.3~5V） | 上位机 USB-TTL 模块的 5V/3.3V |
| GND | 地线 | 上位机 GND |
| TX | 串口发送 | 上位机 USB-TTL 的 RX |
| RX | 串口接收 | 上位机 USB-TTL 的 TX |

### 2.5 通信协议

HI12 使用 HiPNUC 自定义二进制协议，通过 UART 输出数据帧：

- 帧头：`0x5A` + `0xA5`
- 包含：帧长度、数据类型标识、数据载荷、校验和
- 数据类型包括：四元数、欧拉角、加速度、角速度、磁力计等
- 支持通过串口指令配置输出内容、频率和波特率

---

## 三、硬件安装方案

### 3.1 所需材料

| 材料 | 数量 | 说明 |
|------|------|------|
| HiPNUC HI12 模块 | 1 | 外置 IMU |
| USB-TTL 串口模块（CP2102/CH340） | 1 | 将 HI12 的 UART 转为 USB，连接上位机 |
| 杜邦线（母对母） | 4 | 连接 HI12 和 USB-TTL |
| 双面胶/魔术贴 | 少量 | 固定模块 |
| 热缩管 | 少量 | 绝缘保护 |

### 3.2 接线图

```
上位机 (Ubuntu)                        HI12 模块
┌─────────────┐                  ┌─────────────┐
│             │                  │             │
│  USB 口 ◄───┼──── USB 线 ────►│  USB-TTL    │
│             │    (CP2102)      │             │
│             │                  │  5V  ──────►│ VCC
│             │                  │  GND ──────►│ GND
│             │                  │  RX  ◄─────│ TX
│             │                  │  TX  ─────►│ RX
│             │                  │             │
└─────────────┘                  └─────────────┘
```

**接线步骤**：

1. 将 USB-TTL 模块插入上位机 USB 口
2. 用杜邦线连接 USB-TTL 和 HI12：
   - USB-TTL 的 **5V** → HI12 的 **VCC**
   - USB-TTL 的 **GND** → HI12 的 **GND**
   - USB-TTL 的 **RX** → HI12 的 **TX**
   - USB-TTL 的 **TX** → HI12 的 **RX**
3. 确认上位机识别串口设备：`ls /dev/ttyUSB*`

### 3.3 安装位置

**关键原则：远离电机和电调，减少电磁干扰**。

推荐安装位置：EP 底盘**顶部**，激光雷达**旁边或对面**，尽量远离底盘中心电机区域。

```
             EP 顶部俯视图
    ┌──────────────────────────┐
    │                          │
    │   ★ RPLIDAR A2           │
    │   (前方)                  │
    │                          │
    │        [底盘中心/电机]     │
    │                          │
    │              ★ HI12      │
    │              (后方偏上)    │
    │                          │
    └──────────────────────────┘
```

安装时注意：
- HI12 模块的坐标系方向应与 `base_link` 对齐（模块上的箭头指向机器人前方）
- 如果无法对齐，需要在 URDF 中设置正确的 `rpy` 旋转
- 使用双面胶或魔术贴固定，确保行驶中不会松动
- 线缆沿底盘边缘走线，避免缠绕运动部件

### 3.4 串口权限配置

```bash
# 查看串口设备
ls /dev/ttyUSB*

# 将当前用户加入 dialout 组（如果尚未加入）
sudo usermod -a -G dialout $USER

# 重新登录后生效，或临时生效：
sudo chmod 666 /dev/ttyUSB1

# 为 HI12 创建固定别名（推荐，避免设备号漂移）
# 创建 /etc/udev/rules.d/99-hipnuc-hi12.rules：
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="hi12_imu", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hipnuc-hi12.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# 之后 HI12 将固定映射为 /dev/hi12_imu
```

> 注意：上面 `idVendor` 和 `idProduct` 需要根据你使用的 USB-TTL 芯片型号修改。CP2102 对应 `10c4:ea60`，CH340 对应 `1a86:7523`。可通过 `lsusb` 查看。

---

## 四、软件修改方案

### 4.1 修改概览

| 序号 | 修改内容 | 文件 | 说明 |
|------|----------|------|------|
| 1 | 新建 HI12 ROS 驱动节点 | `src/rm_ep_driver/scripts/hi12_imu_node.py` | 读取 HI12 串口数据，发布 `/imu` |
| 2 | 修改 EP 驱动节点 | `src/rm_ep_driver/scripts/rm_ep_driver_node.py` | 禁用 IMU 数据发布，保留 odom |
| 3 | 修改 EKF 配置 | `src/rm_ep_navigation/config/ekf.yaml` | 启用绝对航向，调整融合策略 |
| 4 | 修改 URDF 模型 | `src/rm_ep_description/urdf/rm_ep.urdf.xacro` | 更新 imu_link 位置（如果 HI12 安装位置偏移） |
| 5 | 修改 launch 文件 | `src/rm_ep_navigation/launch/mapping.launch` | 加入 HI12 节点启动 |
| 6 | 修改 launch 文件 | `src/rm_ep_navigation/launch/navigation.launch` | 加入 HI12 节点启动 |
| 7 | 修改驱动配置 | `src/rm_ep_driver/config/rm_ep_params.yaml` | 添加 HI12 参数 |

### 4.2 新建 HI12 ROS 驱动节点

**文件**：`src/rm_ep_driver/scripts/hi12_imu_node.py`

核心功能：
- 通过 UART 串口读取 HI12 数据帧
- 解析 HiPNUC 二进制协议，提取四元数/欧拉角、角速度、加速度
- 将数据转换为 `sensor_msgs/Imu` 消息发布到 `/imu` 话题
- `frame_id` 设置为 `imu_link`

节点参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/hi12_imu` | HI12 串口设备 |
| `baud` | `115200` | 波特率 |
| `frame_id` | `imu_link` | IMU TF 帧名 |
| `publish_rate` | `50` | 发布频率 (Hz) |

关键实现要点：
1. **协议解析**：参考 HiPNUC 官方通信协议文档（下载页：https://download.hipnuc.com/products/#attitude），解析帧头 `0x5A 0xA5`，根据数据类型标识提取对应数据
2. **四元数转换**：HI12 输出的四元数顺序可能为 `[w, x, y, z]` 或 `[x, y, z, w]`，需对照协议文档确认，ROS `sensor_msgs/Imu` 使用 `[x, y, z, w]`
3. **坐标系对齐**：HI12 的坐标系约定可能与 ROS 不同，需要测试并确认是否需要旋转
4. **协方差设置**：HI12 自带姿态解算，orientation_covariance 可以设置较小值（表示高置信度）

### 4.3 修改 EP 驱动节点

**文件**：`src/rm_ep_driver/scripts/rm_ep_driver_node.py`

**改动**：添加 `enable_imu` 参数，默认改为 `False`，关闭 IMU 数据发布。

```python
# _load_params() 中添加：
self.enable_imu = rospy.get_param("~enable_imu", False)

# _start_data_streams() 中条件订阅 IMU：
if self.enable_imu:
    chassis.sub_imu(freq=freq, callback=self._imu_callback)

# _publish_state_estimation() 中条件发布 IMU：
if self.enable_imu and self._imu_data is not None:
    # ... 现有的 IMU 数据映射和发布逻辑 ...
```

EP 驱动仍保留完整的 IMU 功能代码，通过参数切换即可恢复使用 EP 内置 IMU。

### 4.4 修改 EKF 配置

**文件**：`src/rm_ep_navigation/config/ekf.yaml`

**核心改动**：HI12 提供绝对航向（磁力计），不再需要 `imu0_relative: true`。

```yaml
# 修改前（EP IMU，无绝对航向）：
imu0_config: [false, false, false,
               false, false, true,    # 只融合 yaw
               false, false, false,
               false, false, true,    # vyaw
               true,  true,  false]   # ax, ay
imu0_relative: true  # 上电 yaw=0

# 修改后（HI12，有绝对航向）：
imu0_config: [false, false, false,
               false, false, true,    # 融合 yaw（绝对航向）
               false, false, false,
               false, false, true,    # vyaw
               true,  true,  false]   # ax, ay
imu0_relative: false  # HI12 提供绝对航向，不再需要相对模式
```

由于 HI12 的姿态数据更可靠，还可以适当降低过程噪声中 yaw 相关的值，让 EKF 更信任 IMU：

```yaml
# yaw 过程噪声从 0.06 降低
process_noise_covariance:
  # ... 第6行第6列（yaw）: 0.06 → 0.02
```

### 4.5 修改 URDF 模型

**文件**：`src/rm_ep_description/urdf/rm_ep.urdf.xacro`

如果 HI12 安装位置与 EP 原内置 IMU 不同（当前 `imu_link` 在底盘中心 `(0, 0, chassis_height/2)`），需要更新 `imu_joint` 的 `origin`：

```xml
<!-- 如果 HI12 安装在底盘后方偏上（举例） -->
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <!-- 修改 xyz 为 HI12 实际安装位置 -->
  <origin xyz="-0.08 0 0.10" rpy="0 0 0"/>
</joint>
```

**如果 HI12 的坐标系方向与 base_link 不同**（模块朝向偏转），需设置 `rpy`。例如 HI12 箭头朝后：

```xml
<origin xyz="-0.08 0 0.10" rpy="0 0 3.14159"/>
```

具体值需根据实际安装确定。

### 4.6 修改 launch 文件

**文件**：`src/rm_ep_navigation/launch/mapping.launch` 和 `navigation.launch`

添加 HI12 节点启动：

```xml
<!-- HI12 外置 IMU 节点 -->
<node name="hi12_imu_node" pkg="rm_ep_driver" type="hi12_imu_node.py" output="screen">
  <param name="port" value="$(arg hi12_port)"/>
  <param name="baud" value="$(arg hi12_baud)"/>
  <param name="frame_id" value="$(arg imu_frame)"/>
  <param name="publish_rate" value="50"/>
</node>
```

添加 launch 参数：

```xml
<arg name="hi12_port" default="/dev/hi12_imu"/>
<arg name="hi12_baud" default="115200"/>
```

EP 驱动 include 中添加 `enable_imu` 参数：

```xml
<include file="$(find rm_ep_driver)/launch/rm_ep_chassis_bringup.launch">
  <!-- ... 现有参数 ... -->
  <arg name="enable_imu" value="false"/>  <!-- 禁用 EP 内置 IMU -->
</include>
```

### 4.7 修改驱动配置

**文件**：`src/rm_ep_driver/config/rm_ep_params.yaml`

添加 HI12 相关参数：

```yaml
# HI12 外置 IMU 参数
hi12_port: "/dev/hi12_imu"    # HI12 串口设备
hi12_baud: 115200             # HI12 波特率

# EP 内置 IMU 开关（使用 HI12 时设为 false）
enable_imu: false
```

---

## 五、使用方法

### 5.1 建图模式

```bash
roslaunch rm_ep_navigation mapping.launch
```

HI12 节点将自动启动，发布 `/imu` 话题，EKF 融合 HI12 的绝对航向数据。

### 5.2 导航模式

```bash
roslaunch rm_ep_navigation navigation.launch map_file:=/path/to/map.yaml
```

### 5.3 切换回 EP 内置 IMU

如果需要回退到 EP 内置 IMU：

```bash
# 启动时指定参数
roslaunch rm_ep_navigation mapping.launch use_hi12:=false enable_imu:=true
```

### 5.4 验证 HI12 数据

```bash
# 查看 IMU 话题数据
rostopic echo /imu

# 查看 IMU 发布频率
rostopic hz /imu

# 查看 TF 树（确认 imu_link 位置正确）
rosrun tf view_frames

# 对比 EP IMU 和 HI12 数据（同时启用两个，发布到不同话题）
rostopic echo /imu  # HI12
rostopic echo /imu_ep  # EP（需修改发布话题名）
```

---

## 六、磁力计校准

HI12 的磁力计在室内环境受金属结构、电机磁场影响，使用前**必须校准**。

### 6.1 校准步骤

1. 将 EP 放置在使用环境中（电机断电或低速）
2. 使用 HiPNUC 官方上位机软件（下载页获取）进行磁力计校准
3. 校准时缓慢旋转 EP，覆盖所有姿态
4. 校准数据保存在 HI12 模块内部，断电不丢失

### 6.2 室内注意事项

- 室内有大量金属结构（桌腿、墙面钢筋等），磁力计精度会受影响
- 如果室内环境磁干扰严重，可以考虑**关闭磁力计融合**，仅使用陀螺仪积分获取相对航向
- 在 HI12 上位机中可配置是否启用磁力计
- 如果关闭磁力计，EKF 配置应改回 `imu0_relative: true`

---

## 七、风险与回退

| 风险 | 影响 | 应对 |
|------|------|------|
| HI12 串口驱动不稳定 | IMU 数据中断 | EP 驱动保留完整 IMU 代码，参数切换即可回退 |
| 磁力计室内不可靠 | yaw 仍有偏差 | 关闭磁力计，仅用陀螺仪（比 EP IMU 仍好） |
| HI12 安装位置不当 | 坐标系偏差 | URDF 中精确设置 imu_joint 的 origin |
| USB-TTL 接触不良 | 数据丢帧 | 选择带螺丝固定的 USB-TTL 模块 |
| HI12 协议解析错误 | 数据错误 | 严格参照官方协议文档，充分测试 |

**回退方案**：所有修改都通过参数控制，将 `use_hi12:=false` 和 `enable_imu:=true` 即可完全恢复到原始方案。

---

## 八、参考资料

- HiPNUC 官方下载页：https://download.hipnuc.com/products/#attitude
- HiPNUC 通信协议文档：从上述下载页获取
- HiPNUC 上位机软件：从上述下载页获取
- ROS sensor_msgs/Imu：http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
- robot_localization EKF 配置：https://github.com/cra-ros-pkg/robot_localization
