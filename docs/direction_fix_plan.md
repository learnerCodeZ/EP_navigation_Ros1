# 小车前后左右方向反了 — 诊断与修复方案

> **日期**: 2026-06-21
> **状态**: 方案 A 已改好并推送，明天先测试方案 A

---

## 现象

1. **前后反了**: 小车物理往前走，RVIZ 里小车往后走
2. **左右反了**: 障碍物在车左边的，RVIZ 里显示在右边

## 一句话总结

前后反 + 左右反 = 180° 旋转。**最可能的根因是 RPLIDAR 的 `inverted` 参数出错**（方案 A，已修好）。`inverted` 控制雷达点云数据的方向——RPLIDAR 硬件 0° 指向线缆（车头），但驱动内部做了 `PI - angle` 翻转，把 0° 变到了车尾。`inverted=true` 补偿这个翻转，上次改成 `false` 导致点云 180° 反向，前后左右全乱。另外两个原因（EKF 配置、HI12 安装方向）是备选，大概率用不到。

---

## 根因分析（按概率从高到低）

### 原因 1（90% 概率）：RPLIDAR `inverted` 参数错误 — 已修好

上次改雷达方向时的遗留问题：

| 参数 | 之前（线缆朝后） | 改后 | 应该的值 |
|------|-----------------|------|---------|
| URDF `rpy` | `0 0 π` | `0 0 0` ✅ | `0 0 0` |
| `inverted` | `true` | `false` ❌ | `true` |

RPLIDAR A2 硬件 0° 指向线缆方向（车头），驱动内部 `PI - angle` 把 0° 翻到车尾。`inverted=true` 再翻一次，双重翻转后点云方向正确。

上次只去了 URDF 的翻转但没恢复 `inverted` 的翻转 → 点云 180° 反转 → gmapping 建图反了 → RVIZ 前后左右全反。

已于 2026-06-21 把 `inverted` 从 `false` 改回 `true`，**待实际验证**。

### 原因 2（8% 概率）：EKF 坐标系不匹配

EKF 直接使用 EP 的绝对位置（`odom0_differential: false`）和 HI12 的磁航向（`imu0_relative: false`），如果 EP 上电朝向和磁北不一致，两者坐标系会有夹角。但这个问题通常只导致偏移角度，不太会恰好 180°。

### 原因 3（2% 概率）：HI12 模块装反

HI12 要求 X 朝前、Y 朝左。如果模块转了个方向，磁航向偏移。

---

## 修复方案（按顺序尝试）

### 方案 A（已改，明天先测试这个）

**当前状态**：`inverted=true` 已改好并推送，明天 `git pull` 后直接测试。

验证步骤：
```bash
roslaunch rm_ep_navigation mapping.launch
```
遥控小车往前走，RVIZ 里观察点云方向：
- ✅ 点云往车头方向延伸 → 修好了
- ❌ 点云还是反方向 → 继续方案 B

### 方案 B：EKF 配置改 `odom0_differential: true`

**要改的文件**: [ekf.yaml](../src/rm_ep_navigation/config/ekf.yaml)

**改动**: 第 29 行 `odom0_differential: false` → `true`

**原理**: 让 EKF 只用里程计的位置**增量**（相邻帧差值），不直接用绝对位置。增量按 IMU 的磁航向旋转，解耦两个坐标系。

**副作用**: 无。这是标准的 wheel odom + IMU 融合方式。

### 方案 C：odom 位置 px 取反（终极手段）

**要改的文件**: [rm_ep_driver_node.py](../src/rm_ep_driver/scripts/rm_ep_driver_node.py)

**改动**: 第 347 行 `position.x = px` → `position.x = -px`
同时第 372 行 `velocity.x = vel[0]` → `velocity.x = -vel[0]`

**原理**: 如果 EP SDK 返回的位置数据和速度数据方向本身不一致，直接对 X 取反修正。

**注意**: 不能只改 cmd_vel 不改 odom，否则建图和导航都会有问题。要改就 odom 和 cmd_vel 一起看。

### 左右问题排查：检查 HI12 硬件方向

看一下 HI12 模块上的 XYZ 标注：
- X 应该朝车头
- Y 应该朝车左
- Z 应该朝上

如果方向不对，松开模块重新安装即可。

---

## 关键文件路径

| 文件 | 作用 |
|------|------|
| `src/rm_ep_driver/scripts/rm_ep_driver_node.py` | EP 驱动：odom 发布 (line 340-375) + cmd_vel 接收 (line 428-468) |
| `src/rm_ep_driver/scripts/hi12_imu_node.py` | HI12 IMU 驱动：直接透传 HI12 数据到 /imu，无坐标变换 |
| `src/rm_ep_navigation/config/ekf.yaml` | EKF 融合：odom + IMU → /odometry/filtered + TF |
| `src/rm_ep_description/urdf/rm_ep.urdf.xacro` | URDF 模型：TF 树 + laser_link 位置 (line 93-97) |
| `src/rplidar_ros/src/node.cpp` | RPLIDAR 驱动：`PI-angle` 重映射 (line 71-78) + `inverted` 逻辑 (line 89) |
| `src/rm_ep_navigation/launch/mapping.launch` | 建图 launch：雷达 inverted 参数 (line 26) |
| `src/rm_ep_navigation/launch/navigation.launch` | 导航 launch：雷达 inverted 参数 (line 27) |
