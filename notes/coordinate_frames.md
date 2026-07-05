---
name: coordinate-frames-explanation
description: ROS 导航中 odom、map、base_link 坐标系的关系与数据来源
metadata:
  type: reference
---

# ROS 坐标系关系说明

## odom 坐标系

**数据来源：EP 底盘里程计 + HI12 外置 IMU**，通过 `robot_localization` 的 EKF 节点融合。

- EP 底盘回报的 **vbx/vby/vyaw**（体坐标系速度、角速度）→ 积分得到位置和朝向变化
- HI12 IMU 的 **陀螺仪角速度** → 提供平滑的相对旋转
- EKF 融合后输出 `odom→base_link` 的变换

**特点：**
- odom 帧是**连续的、无跳变的** — 位置和朝向只靠积分，不会突然变化
- 初始位置为 `(0,0,0)`，机器启动时 odom 和 map 重合
- 但积分误差会随时间累积（odom 会慢慢漂移）

## 外置 IMU (HI12) 决定了什么

- **陀螺仪（角速度）**：决定 odom 中 `base_link` 朝向的变化率。EKF 用 IMU 的 `imu0.angular_velocity` 和 `imu0.orientation` 作为相对旋转参考（因为设了 `imu0_relative: true`，不用磁力计绝对值）
- **加速度计**：提供重力方向参考，帮助估计俯仰/横滚
- **被禁用的是磁力计**：因为 EP 金属底盘和电机磁场严重干扰 HI12 磁力计，所以不用它修正绝对航向

如果没有 HI12 IMU，EKF 只能靠 EP 底盘的角速度积分，精度差很多。

## map 坐标系

**数据来源：AMCL（自适应蒙特卡洛定位）**

- AMCL 加载预先建好的**地图**（pgm + yaml），然后用激光雷达实时扫描数据 `/scan` 匹配到地图上
- AMCL 不断计算：**"当前激光扫描最可能对应地图的哪个位置和朝向"**
- AMCL 通过 `map→odom` 变换，把 odom 的漂移修正回来

**关键：**
- AMCL 输出的 `map→odom` 变换是**离散的、可能跳变的** — 每次重定位都会调整
- `map` 帧本身相对于地图图片是固定的（左下角 (0,0) 对应图片原点）
- 所以规划导航路径用 `map` 帧下的目标点 → 经 `map → odom → base_link` 转换给底盘

## 整体流程

```
激光雷达  ──→  AMCL  ──→  map → odom 变换（修正漂移）
EP里程计  ─┐
           ├→  EKF  ──→  odom → base_link 变换（连续局部位姿）
HI12 IMU  ─┘

move_base：
  global_plan:   map 帧下规划路径
  local_plan:    odom 帧下跟踪路径（TEB 局部规划器）
```

简单说：**odom 来自积分导航（平滑但会漂），map 来自激光定位（准确但会跳），map 修正 odom 的漂移。**
