# 里程计漂移与 IMU 融合学习笔记

## 1. 核心事实：Odom 漂移不可避免

Odom 通过轮子转动积分推算位置，每次测量的微小误差会随时间累积：

```
真实移动: 1.00 m → 轮子估计: 1.05 m → 单次误差 0.05 m → 累积越来越大
```

工程目标不是消除漂移，而是：
- **减少漂移速度**（IMU 融合、低速运动、防打滑）
- **持续修正漂移**（SLAM 建图/定位闭环）

## 2. 麦轮漂移的特殊性

EP 使用麦克纳姆轮，比普通轮更容易漂移：
- 麦轮与地面是点接触，光滑地面容易打滑
- 横向运动时轮子既有滚动又有滑动
- 急停/急转时轮子可能空转

**实际对策**：建图时低速平稳移动；导航时限制加速度（当前 `acc_lim_x=0.5, acc_lim_theta=1.0`）。

## 3. HI12 IMU 能修正什么

HI12 是 9 轴 AHRS（3 轴陀螺仪 + 3 轴加速度计 + 3 轴磁力计），自带 EKF 姿态解算。

**最大的价值：修正 Yaw 朝向**

| 场景 | 无 IMU | 有 HI12 |
|------|--------|---------|
| 实际转 90° | 轮速计可能报 92° | HI12 测量约 90° |
| 直行 5m | Yaw 可能偏几度 | Yaw 基本不偏 |
| 建图效果 | 地图越来越歪 | 地图方向稳定 |

**具体作用**：
- 地图不会越来越歪（gmapping 依赖准确的朝向）
- AMCL 重定位更稳定
- EKF 融合后的 `odom→base_link` TF 更准确

## 4. 当前项目的融合架构

```
EP 底盘 → /odom (位置+速度) ─┐
                              ├→ EKF (robot_localization) → odom→base_link TF
HI12 IMU → /imu (姿态+角速度)┘
                                       ↓
RPLIDAR A2 → /scan ─→ gmapping/amcl → map→odom TF
```

**关键设计决策**：

### 为什么用外置 HI12 不用 EP 内置 IMU？

| 问题 | EP 内置 IMU | HI12 外置 IMU |
|------|-------------|---------------|
| Yaw 漂移 | 严重（几分钟偏几十度） | 极小（5 分钟静态 0°） |
| 电机干扰 | 大（IMU 在电机旁边） | 小（安装位置远离电机） |
| 磁力计 | 无 | 有，提供绝对航向 |
| 姿态解算 | 无（原始数据） | 自带 EKF，输出融合四元数 |

### EKF 融合策略（ekf.yaml）

- **odom 数据源**：绝对位置 X,Y + 世界坐标系速度 vx,vy + 角速度 vyaw
- **IMU 数据源**：绝对 Yaw（磁力计提供）+ 角速度 vyaw + 加速度 ax,ay
- `imu0_relative: false`：HI12 有磁力计，提供绝对航向，不需要相对模式
- Yaw 过程噪声 0.03（较信任 HI12 的航向数据）

### 坐标系对齐

HI12 安装时 X 轴朝车头、Y 轴朝左、Z 轴朝上，与 ROS REP-103 一致。HI12 输出标准物理量，**无需任何符号取反或旋转变换**。

## 5. 漂移问题排查清单

当发现 odom 偏移严重时，按顺序检查：

### 5.1 Odom 本身是否正常

```bash
# 静止时观察，position 不应该持续变化
rostopic echo /odom -n 5
```

如果静止时 x/y 持续变化 → odom 数据源本身有问题，优先解决。

### 5.2 IMU 是否被正确使用

```bash
# 确认 IMU 话题存在且有数据
rostopic list | grep imu
rostopic echo /imu -n 1
```

确认 EKF 配置中 `imu0_config` 融合了 orientation 和 angular_velocity。

### 5.3 IMU 坐标方向是否正确

IMU X 轴必须朝车头。如果方向不对，会导致：
- 地图旋转
- 方向漂移
- SLAM 效果变差

如果无法重新安装，通过 URDF 中 `base_link→imu_link` 的 TF 增加旋转补偿。

### 5.4 TF 树是否完整

```bash
rosrun tf view_frames
# 检查链路: map → odom → base_link → laser_link / imu_link
```

缺少任何一个环节，SLAM 都无法正常工作。

### 5.5 磁力计是否受干扰

在金属环境（工厂、机房）中，磁力计可能被干扰。症状：
- 静止时 Yaw 缓慢漂移
- 转动后 Yaw 回不到正确值

解决方案：
- 用 HiPNUC 上位机做磁力计校准
- 校准后仍不稳定 → 关闭磁力计，EKF 改回 `imu0_relative: true`

## 6. 从旧版到新版的配置演进

项目中经历了一次配置修正，核心变更：

| 配置项 | 旧版（错误） | 新版（修正后） | 原因 |
|--------|-------------|---------------|------|
| AMCL 里程模型 | `diff-corrected` | `omni-corrected` | 麦轮是全向底盘 |
| AMCL 激光参数 | `z_hit=0.5, z_rand=0.5` | `z_hit=0.8, z_rand=0.05` | 减少随机噪声权重 |
| EKF IMU 融合 | 全量融合 | 只融合 yaw+vyaw+ax/ay | 避免冗余数据冲突 |
| 雷达波特率 | 115200 | 256000 | A2 的正确波特率 |
| 默认连接 | WiFi (ap) | USB (rndis) | USB 更稳定 |
| 驱动坐标映射 | 无 | 完整映射 | SDK 坐标系 ≠ ROS 坐标系 |

这些改动解决了"建图歪、定位飘、导航振荡"三大问题。

## 7. 关键调试命令速查

```bash
# 查看传感器数据
rostopic echo /imu -n 1       # IMU 数据
rostopic echo /odom -n 1      # 里程计数据
rostopic hz /scan             # 雷达频率

# 查看 TF
rosrun tf view_frames         # 生成 TF 树 PDF
rosrun tf tf_echo odom base_link  # 查看具体变换

# 导航调试
rostopic echo /move_base/status  # 导航状态
rostopic echo /cmd_vel -n 5      # 速度指令
rqt_graph                        # 节点关系图
```
