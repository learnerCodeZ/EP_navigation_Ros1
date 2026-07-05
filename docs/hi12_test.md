# HI12 IMU 单独测试指南

## 一、测试前准备

### 1.1 所需材料

- HiPNUC HI12 模块（已焊接排针）
- USB-TTL 串口模块（CP2102 或 CH340）
- 杜邦线 4 根（母对母）
- 电脑（Windows 跑上位机 / Ubuntu 跑 ROS 节点，双系统均可）

### 1.2 接线

```
USB-TTL          HI12
───────          ────
5V      ──────►  VCC
GND     ──────►  GND
RX      ◄─────  TX
TX      ─────►  RX
```

> TX/RX 交叉接：USB-TTL 的 RX 接 HI12 的 TX，反之亦然。

### 1.3 软件准备

| 软件 | 用途 | 获取方式 |
|------|------|----------|
| HiPNUC 上位机 (Windows) | 查看原始数据、配置模块、磁力计校准 | https://download.hipnuc.com/products/#attitude |
| Ubuntu 串口工具 | 验证 Linux 下串口通信 | `sudo apt install minicom` |
| ROS 工作空间 | 跑 hi12_imu_node | 本项目已编译 |

---

## 二、测试 1：Windows 上位机验证

**目的**：确认 HI12 硬件正常、通信协议正确、数据输出内容。

### 2.1 步骤

1. 将 USB-TTL 插入 Windows 电脑
2. 打开设备管理器，确认 COM 口编号（如 COM3）
3. 打开 HiPNUC 上位机，选择对应 COM 口，波特率 115200，点击连接
4. 观察上位机是否正常显示数据

### 2.2 预期结果

| 检查项 | 预期 |
|--------|------|
| 连接状态 | 显示"已连接"，无报错 |
| 四元数 | 静止时 w≈1.0, x≈0, y≈0, z≈0（误差 <0.02） |
| 欧拉角 | 静止时 Roll≈0°, Pitch≈0°, Yaw≈某固定值（受地磁影响不为0） |
| 加速度 | 静止时 ax≈0, ay≈0, az≈9.8 m/s²（或 1g） |
| 角速度 | 静止时三轴接近 0（<0.5°/s） |
| 数据刷新 | 持续更新，无卡顿或断流 |

### 2.3 需记录的信息

在上位机中记录以下信息，后续 ROS 测试要用：

- [ ] **四元数顺序**：上位机显示的是 `[w,x,y,z]` 还是 `[x,y,z,w]`
- [ ] **输出频率**：上位机配置的输出频率（默认 100Hz？200Hz？）
- [ ] **输出内容**：帧中包含哪些数据项（四元数/欧拉角/加速度/角速度/磁力计）
- [ ] **陀螺仪量程**：当前配置（±2000°/s？±500°/s？）
- [ ] **加速度量程**：当前配置（±8g？±4g？）
- [ ] **波特率**：当前配置（115200？）

### 2.4 方向验证

1. 将 HI12 平放在桌面上（丝印朝上），模块箭头/标记朝向你认为的"前方"
2. 记录上位机显示的 Yaw 值（记为 Y0）
3. 将模块**顺时针旋转 90°**（从上方俯视）
4. 记录 Yaw 值（记为 Y1）

| 检查项 | 预期 |
|--------|------|
| Y1 - Y0 | 约 -90°（HI12 逆时针为正，顺时针旋转 Yaw 减小） |
| 或 Y1 - Y0 | 约 +90°（取决于 HI12 固件约定，记录即可） |

> 此结果决定了 ROS 驱动中是否需要 yaw 取反。如果 HI12 顺时针旋转 90° 时 Yaw 减小约 90°，则符合 ROS REP-103（逆时针为正），无需取反。

---

## 三、测试 2：Ubuntu 串口原始数据验证

**目的**：确认 Linux 下串口设备可用，原始字节流正确。

### 3.1 步骤

1. 将 USB-TTL 插入 Ubuntu 电脑（或重启到 Ubuntu）
2. 检查串口设备：

```bash
ls /dev/ttyUSB*
# 应看到 /dev/ttyUSB0 或类似设备

# 查看USB-TTL芯片信息
lsusb
# CP2102: ID 10c4:ea60
# CH340:  ID 1a86:7523
```

3. 配置串口权限：

```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效，或临时：
sudo chmod 666 /dev/ttyUSB0
```

4. 用 minicom 查看原始数据：

```bash
minicom -D /dev/ttyUSB0 -b 115200
```

### 3.2 预期结果

| 检查项 | 预期 |
|--------|------|
| 串口设备存在 | `ls /dev/ttyUSB*` 显示设备 |
| minicom 有数据 | 屏幕持续显示乱码（二进制数据，正常） |
| 帧头可见 | 按 Ctrl+A 再按 H 打开十六进制显示，应看到 `5A A5` 反复出现 |

5. 用 Python 快速验证帧解析：

```python
#!/usr/bin/env python3
"""快速验证 HI12 帧头和基本结构"""
import serial, struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
buf = bytearray()
frame_count = 0

for _ in range(500):
    data = ser.read(64)
    buf.extend(data)
    while len(buf) >= 6:
        idx = buf.find(b'\x5A\xA5')
        if idx < 0:
            buf.clear()
            break
        if idx > 0:
            del buf[:idx]
        if len(buf) < 6:
            break
        payload_len = struct.unpack('<H', buf[2:4])[0]
        frame_len = 6 + payload_len
        if frame_len > 256:
            del buf[:2]
            continue
        if len(buf) < frame_len:
            break
        frame_count += 1
        frame = bytes(buf[:frame_len])
        del buf[:frame_len]
        # 打印前 3 帧的 sub_item ID
        if frame_count <= 3:
            payload = frame[6:]
            items = []
            off = 0
            while off < len(payload):
                item_id = payload[off]
                off += 1
                if item_id == 0x00:
                    break
                items.append(f'0x{item_id:02X}')
                # 跳过数据（简化，不精确跳过）
                break  # 只看第一个 item
            print(f'帧 #{frame_count}: payload_len={payload_len}, 首个item={items}')

print(f'共解析 {frame_count} 帧')
ser.close()
```

### 3.3 预期结果

| 检查项 | 预期 |
|--------|------|
| 帧头 `5A A5` | 持续检测到 |
| 帧数量 | 1~2 秒内解析到几十帧（取决于输出频率） |
| Item ID | 包含 `0xD1`（四元数）、`0xB2`（陀螺仪）、`0xA2`（加速度）等 |

---

## 四、测试 3：ROS hi12_imu_node 单独测试

**目的**：验证 ROS 驱动节点能正确解析数据并发布 `/imu`。

### 4.1 步骤

1. 确认工作空间已编译：

```bash
cd ~/EP_navigation_Ros1
catkin_make
source devel/setup.bash
```

2. 启动 HI12 节点（不启动其他节点）：

```bash
rosrun rm_ep_driver hi12_imu_node.py _port:=/dev/ttyUSB0 _baud:=115200 _publish_rate:=50
```

3. 另开终端检查：

```bash
# 检查话题是否发布
rostopic list | grep imu

# 检查发布频率
rostopic hz /imu

# 查看数据
rostopic echo /imu
```

### 4.2 预期结果

#### 话题检查

| 检查项 | 预期 |
|--------|------|
| 话题存在 | `/imu` 出现在 `rostopic list` |
| 发布频率 | 稳定在 50 Hz 左右（±5Hz） |
| 无报错 | 终端无 "帧校验失败" 或 "数据解析失败" 日志 |

#### 静止数据（HI12 平放桌面）

| 检查项 | 预期 |
|--------|------|
| orientation.w | ≈ 1.0（误差 <0.02） |
| orientation.x | ≈ 0（误差 <0.02） |
| orientation.y | ≈ 0（误差 <0.02） |
| orientation.z | ≈ 某固定小值（取决于朝向） |
| angular_velocity 各轴 | ≈ 0（<0.1 rad/s） |
| linear_acceleration.x | ≈ 0 |
| linear_acceleration.y | ≈ 0 |
| linear_acceleration.z | ≈ 9.8 m/s² |
| header.frame_id | `imu_link` |

#### 方向验证（ROS 中）

1. 平放 HI12，记录当前 yaw：

```bash
# 用 Python 快速从四元数算 yaw
rostopic echo /imu/orientation -n 1
```

记下 `(x, y, z, w)`，用以下公式算 yaw：
```
yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
```

2. **将模块绕 Z 轴逆时针旋转 90°**（从上方俯视）

| 检查项 | 预期 |
|--------|------|
| yaw 变化 | 增加约 +90°（≈1.57 rad） |

3. **再逆时针旋转 90°**（共 180°）

| 检查项 | 预期 |
|--------|------|
| yaw 变化 | 相比初始增加约 +180°（≈3.14 rad） |

4. **顺时针旋转 90°**（回到起始方向）

| 检查项 | 预期 |
|--------|------|
| yaw 变化 | 回到初始值附近（误差 <5°） |

> 如果逆时针旋转时 yaw 减小而不是增加，说明 yaw 方向与 ROS 相反，需要在驱动中取反。

#### 加速度验证

1. HI12 平放：az ≈ 9.8
2. HI12 竖放（模块前方朝上）：ax ≈ 9.8, az ≈ 0
3. HI12 侧放（模块左方朝上）：ay ≈ 9.8, az ≈ 0

| 检查项 | 预期 |
|--------|------|
| 重力方向正确 | 重力始终指向下方，对应轴读数 ≈9.8 |
| 各轴独立 | 翻转后只有对应轴变化，其他轴接近 0 |

---

## 五、测试 4：四元数顺序验证

**目的**：确认 HI12 输出四元数的字节顺序与代码假设一致。

### 5.1 背景

当前代码假设 HI12 四元数输出为 `[w, x, y, z]`（4 个 float32，小端序）：

```python
# hi12_imu_node.py 第 273-274 行
self._quaternion = struct.unpack('<4f', data[:16])
# 发布时：quat[1]->x, quat[2]->y, quat[3]->z, quat[0]->w
```

如果实际顺序是 `[x, y, z, w]`，则 x 和 w 互换，会导致方向完全错误。

### 5.2 步骤

1. HI12 平放，用 Windows 上位机记录四元数值（记为 W_up, X_up, Y_up, Z_up）
2. 用 ROS 节点记录同姿态下的四元数：

```bash
rostopic echo /imu/orientation -n 1
```

3. 对比两边的值

### 5.3 预期结果

| 场景 | 上位机 | ROS /imu (orientation) |
|------|--------|----------------------|
| 平放 | w≈1, x≈0, y≈0, z≈小值 | w≈1, x≈0, y≈0, z≈同上位机 |

**如果 w 和 x 互换了**：说明 HI12 输出顺序是 `[x,y,z,w]` 而非 `[w,x,y,z]`，需要修改代码：

```python
# 修改 _parse_item 中 ITEM_ID_QUAT 的解析
# 从: self._quaternion = struct.unpack('<4f', data[:16])
# 改为: vals = struct.unpack('<4f', data[:16])
#       self._quaternion = (vals[3], vals[0], vals[1], vals[2])  # 重组为 (w,x,y,z)
```

---

## 六、测试 5：长时间稳定性

**目的**：验证 HI12 静态和动态下的数据稳定性。

### 6.1 静态漂移测试

1. HI12 平放桌面，保持完全静止
2. 记录初始 yaw 值
3. 等待 5 分钟
4. 记录最终 yaw 值

| 检查项 | 预期 |
|--------|------|
| 5 分钟 yaw 漂移 | < 3°（HI12 有磁力计，应比纯陀螺仪好很多） |
| 角速度零漂 | 三轴 < 0.5°/s |

### 6.2 动态恢复测试

1. 记录静止 yaw（Y0）
2. 拿起 HI12 旋转几圈后放回原位
3. 等待 2 秒，记录恢复后 yaw（Y1）

| 检查项 | 预期 |
|--------|------|
| Y1 - Y0 | < 5°（有磁力计绝对参考应快速恢复） |

---

## 七、测试 6：磁力计校准

**目的**：校准磁力计，提高室内 yaw 精度。

### 7.1 步骤

1. 回到 Windows，打开 HiPNUC 上位机
2. 找到磁力计校准功能（通常在"设置"或"校准"菜单）
3. 按照提示操作：
   - 点击"开始校准"
   - 缓慢旋转 HI12，覆盖所有姿态（水平转、竖直转、翻滚）
   - 校准完成后点击"保存"
4. 校准数据保存在 HI12 内部，断电不丢失

### 7.2 校准前后对比

回到 Ubuntu 运行 ROS 节点，对比校准前后：

| 检查项 | 校准前 | 校准后预期 |
|--------|--------|-----------|
| 静止 yaw 稳定性 | 可能跳变 | 平稳，无突变 |
| 旋转 360° 回原位 | yaw 偏差可能 >10° | yaw 偏差 <5° |
| 磁力计原始值分布 | 椭圆（偏心） | 接近球形（居中） |

---

## 八、测试 7：与 EKF 联合测试

**目的**：验证 HI12 数据接入 EKF 后的效果。

> 此测试需要 EP 底盘同时运行，是集成测试，放到最后做。

### 8.1 步骤

1. 启动完整导航栈：

```bash
roslaunch rm_ep_navigation mapping.launch
```

2. 检查 EKF 输出：

```bash
rostopic echo /odometry/filtered -n 1
rostopic hz /odometry/filtered
```

3. 检查 TF 树：

```bash
rosrun tf view_frames
# 查看生成的 frames.pdf，确认 odom → base_link 链路正常
```

4. 在 RViz 中观察机器人方向是否跟随实际转向

### 8.2 预期结果

| 检查项 | 预期 |
|--------|------|
| /odometry/filtered 频率 | ≈30 Hz（EKF 输出频率） |
| TF 树 | odom → base_link 连续，无跳变 |
| RViz 机器人方向 | 转弯时方向同步变化，无明显滞后 |
| 原地旋转 360° | 回到原位后方向误差 <10° |

---

## 九、问题排查速查

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| `rostopic hz /imu` 显示 0Hz | 串口未连接或权限不足 | `ls /dev/ttyUSB*`，`sudo chmod 666 /dev/ttyUSB0` |
| 频率远低于 50Hz | 波特率不匹配或输出频率配置低 | 检查 HI12 波特率配置，用上位机调高输出频率 |
| 四元数 w 始终接近 0 | 四元数顺序错误 | 按测试 4 验证，修改解析代码 |
| yaw 旋转方向相反 | HI12 yaw 正方向与 ROS 相反 | 在 `_publish_callback` 中对 `quat[3]`（z 分量）取反 |
| yaw 静止时持续漂移 | 磁力计未校准或受干扰 | 做磁力计校准，或远离金属物体测试 |
| "帧校验失败" 日志频繁 | 串口波特率错误 | 确认 HI12 和驱动使用相同波特率 |
| 加速度 az ≈ -9.8 | Z 轴方向相反 | 确认 HI12 丝印朝上安装，或在驱动中取反 az |
| 数据偶尔中断 | USB-TTL 接触不良 | 更换 USB 线或 USB-TTL 模块 |

---

## 十、测试记录表

完成每个测试后打勾并记录关键数值：

| 测试 | 状态 | 关键结果 |
|------|------|----------|
| 测试 1：Windows 上位机 | ☐ | 四元数顺序：____，输出频率：____Hz |
| 测试 2：Ubuntu 串口原始数据 | ☐ | 帧头检测：____，帧率：____ |
| 测试 3：ROS 节点单独测试 | ☐ | /imu 频率：____Hz，静态 yaw 漂移：____°/min |
| 测试 4：四元数顺序验证 | ☐ | 上位机 w=____，ROS w=____，顺序是否一致：____ |
| 测试 5：长时间稳定性 | ☐ | 5分钟漂移：____° |
| 测试 6：磁力计校准 | ☐ | 校准前偏差：____°，校准后偏差：____° |
| 测试 7：EKF 联合测试 | ☐ | 360° 旋转回位误差：____° |

---

## 十一、Ubuntu 实车测试结果

> 日期：2026-07-05
> 环境：HI12 已装车，USB-TTL (CP2102) 接入 Ubuntu，udev 别名 `/dev/hi12_imu`

### 11.1 串口设备确认

```bash
ls -la /dev/hi12_imu
```

结果：

```
lrwxrwxrwx 1 root root 7 1月   1  2000 /dev/hi12_imu -> ttyUSB1
```

- [x] 串口设备存在

### 11.2 ROS 节点启动

```bash
rosrun rm_ep_driver hi12_imu_node.py _port:=/dev/hi12_imu _baud:=115200 _publish_rate:=50
```

启动输出：

```
[INFO] [1783240444.249385]: HI12 串口已连接: /dev/hi12_imu @ 115200 bps
[INFO] [1783240444.257493]: HI12 IMU 驱动已启动 (port=/dev/hi12_imu, baud=115200, rate=50 Hz)
```

- [x] 无报错，串口连接成功

### 11.3 话题与频率

```bash
rostopic hz /imu
```

结果：0 Hz（无数据发布）

- [ ] 频率稳定在 50Hz 左右（±5Hz）— **失败**

**问题**：节点连接串口成功，但 `/imu` 无数据发布。

### 11.4 串口原始数据验证

单独读取串口原始数据，确认硬件有输出：

```python
import serial
ser = serial.Serial('/dev/hi12_imu', 115200, timeout=3)
data = ser.read(1024)
# 帧头 5A A5 在第 0 字节
# payload_len = 76 字节
```

- [x] 帧头 `5A A5` 存在
- [x] 串口有持续数据输出

### 11.5 帧解析问题定位与修复

**发现关键 bug**：旧代码使用 sub_item 格式（item_id+data+0x00 结束），读到 0x91(UID) 后碰 0x00 就停，后面 70 字节全丢。

**查阅官方协议文档**（`imu_cum_cn.pdf`）后发现：HI12 出厂默认输出 **HI91 定长帧**（76 字节 payload），是固定偏移的 C struct 风格，不是 sub_item 动态结构。

**HI91 payload 结构**（来自 `imu_cum_cn.pdf` 第 30 页）：

| 偏移 | 名称 | 类型 | 大小 | 单位 |
|------|------|------|------|------|
| 0 | tag | uint8 | 1 | 0x91 |
| 1 | main_status | uint16 | 2 | 状态字 |
| 3 | temperature | int8 | 1 | °C |
| 4 | air_pressure | float32 | 4 | Pa |
| 8 | system_time | uint32 | 4 | ms |
| 12 | acc_b | float32×3 | 12 | **G**（1G≈9.8m/s²）|
| 24 | gyr_b | float32×3 | 12 | **deg/s** |
| 36 | mag_b | float32×3 | 12 | μT |
| 48 | roll | float32 | 4 | deg |
| 52 | pitch | float32 | 4 | deg |
| 56 | yaw | float32 | 4 | deg |
| 60 | quat | float32×4 | 16 | **WXYZ** |

**关键单位差异**（来自文档"解码陷阱速览"）：
- HI91 加速度单位是 **G**（不是 m/s²），需 ×9.80665
- HI91 角速度单位是 **deg/s**（不是 rad/s），需 ×π/180
- HI91 四元数顺序是 **WXYZ**（ROS 需要 XYZW）

**实测数据验证**（对照上表）：

| 偏移 | 实测值 | 文档含义 | 判定 |
|------|--------|---------|------|
| 0-3 | 0x91, 0x00... | tag=0x91, status | ✅ |
| 12-15 | ~0.0000 | acc_x (G) | ✅ 平放≈0 |
| 16-19 | ~0.009 | acc_y (G) | ✅ 小偏移 |
| 20-23 | **~0.997** | **acc_z (G)** | ✅ ≈1G≈9.8m/s² |
| 24-35 | 小值 | gyr_b (deg/s) | ✅ 静止≈0 |
| 36-47 | 固定 | mag_b (μT) | ✅ |
| 48-59 | 固定 | roll/pitch/yaw (deg) | ✅ |
| 60-63 | ~-0.69 | quat_w | ✅ |
| 64-67 | ~0.04 | quat_x | ✅ |
| 68-71 | ~-0.03 | quat_y | ✅ |
| 72-75 | ~0.72 | quat_z | ✅ |

**修复**：重写 `hi12_imu_node.py`，改为 HI91 固定偏移解析，单位转换 G→m/s²、deg/s→rad/s。

### 11.6 修复后重新测试

重新启动节点（同步更新后的 `hi12_imu_node.py` 到 Ubuntu 后）：

```bash
rosrun rm_ep_driver hi12_imu_node.py _port:=/dev/hi12_imu _baud:=115200 _publish_rate:=50
```

#### 话题频率

```bash
rostopic hz /imu
```

结果：**average rate: 50.000 Hz**，std dev 0.00018s，稳定 ✅

#### 平放静止数据

```bash
rostopic echo /imu -n 1
```

实测值：

| 字段 | 实测值 | 判定 |
|------|--------|------|
| orientation (x,y,z,w) | (0.040, -0.035, 0.718, -0.694) | 范数≈1.0 ✅ |
| orientation.w | -0.694 | w 为负（q 与 -q 等价，非 bug）|
| angular_velocity 三轴 | ~0.002, 0.006, 0.003 rad/s | ✅ 静止接近零 |
| linear_acceleration.z | +9.767 m/s² | ✅ 重力方向正确 |
| linear_acceleration.y | -1.044 m/s² | 模块倾斜约 6°（9.8×sin(-6°)≈-1.02）|
| frame_id | imu_link | ✅ |

#### Yaw 方向验证

1. 平放初始 yaw：

```
orientation: x=0.0396 y=-0.0349 z=0.7177 w=-0.6944
yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)) = -1.6034 rad = -91.87°
```

2. 车**逆时针旋转 90°**（俯视，车头向左转）后：

```
orientation: x=0.0231 y=0.0045 z=0.0217 w=-0.9995
yaw = -0.0432 rad = -2.47°
```

3. 变化量：Y1 - Y0 = -2.47 - (-91.87) = **+89.4°**

| 判定 | 条件 | 实测 |
|------|------|------|
| ✅ 方向正确 | 逆时针转 yaw 增加 ≈ +90° | +89.4° |

**Yaw 方向符合 ROS REP-103（逆时针为正），无需在驱动中取反。**

> 注：HI12 是 AHRS，输出绝对航向（相对磁北），故静止时 yaw 不为 0（实测约 -92°，取决于朝向）。EKF 中 `imu0_relative: false` 已正确配置。

#### 5 分钟静态漂移测试

车平放不动，记录初始和 5 分钟后的 yaw：

```bash
# 初始
python3 -c "import rospy,math;from sensor_msgs.msg import Imu;rospy.init_node('yaw_check',anonymous=True);msg=rospy.wait_for_message('/imu',Imu,timeout=2);x,y,z,w=msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));print(f'Y0 = {math.degrees(yaw):.2f} deg')"
```

结果：

| 时间点 | yaw | 说明 |
|--------|-----|------|
| 初始 (Y0) | -0.79° | |
| 5 分钟后 (Y1) | -0.79° | |

**漂移量：0°**

| 判定 | 条件 | 实测 |
|------|------|------|
| ✅ 极佳 | < 3° | 0° |

室内磁力计表现极佳，无需校准。

### 11.7 测试结论

- [x] 串口设备识别正常（`/dev/hi12_imu` → `ttyUSB1`）
- [x] ROS 节点启动无报错
- [x] `/imu` 话题稳定 50 Hz
- [x] 四元数顺序正确（HI91 WXYZ → ROS XYZW，范数=1）
- [x] 加速度 Z 轴方向正确（平放 z≈+9.8 m/s²）
- [x] Yaw 方向正确（逆时针转 90° → yaw +89.4°，符合 REP-103）
- [x] 静态角速度接近零

**关键修复**：原代码使用旧版 sub_item 动态格式解析，与 HI12 出厂默认的 HI91 定长帧不匹配，导致读到 0x00 就停、70 字节数据全丢。查阅 `imu_cum_cn.pdf` 后改为 HI91 固定偏移解析，并修正单位转换（acc G→m/s²、gyr deg/s→rad/s）。

**待办（后续可选）**：
- [x] 5 分钟静态漂移测试（Y0=-0.79°, Y1=-0.79°, 漂移=0°，远低于 3° 阈值）
- [ ] 磁力计校准（当前室内表现已极佳，暂不需要）
- [ ] EKF 联合测试（`/odometry/filtered` 频率与方向跟随）
