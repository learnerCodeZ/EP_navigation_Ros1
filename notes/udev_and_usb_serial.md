---
name: udev-and-usb-serial
description: udev 规则配置、USB 串口设备识别与固定别名
metadata:
  type: reference
---

# udev 规则与 USB 串口设备

## 为什么需要 udev 规则

Linux 下 USB 串口设备的编号是动态的：

- 插拔顺序不同 → `ttyUSB0` 和 `ttyUSB1` 会互换
- 重启后编号可能变化
- 程序里写死 `/dev/ttyUSB0` 不可靠

**udev 规则的作用**：根据设备的唯一标识（厂商ID、产品ID、序列号），自动创建固定名称的软链接，比如 `/dev/rplidar`、`//dev/hi12_imu`。

## 当前项目的 udev 规则

文件位置：`/etc/udev/rules.d/` 下

```
# RPLIDAR A2 激光雷达
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="4faef7fdf8028c4eaed0c8cb06c33e9b", SYMLINK+="rplidar", MODE="0666"

# HI12 IMU
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="384f390688cdf01193eb80c5e9520995", SYMLINK+="hi12_imu", MODE="0666"
```

## 规则字段解释

| 字段 | 含义 | 示例 |
|------|------|------|
| `KERNEL=="ttyUSB*"` | 匹配内核设备名 | 所有 ttyUSB 设备 |
| `ATTRS{idVendor}` | USB 厂商 ID | `10c4` = Silicon Labs |
| `ATTRS{idProduct}` | USB 产品 ID | `ea60` = CP210x UART Bridge |
| `ATTRS{serial}` | USB 设备唯一序列号 | 每个设备不同，这是区分同芯片设备的关键 |
| `SYMLINK+="rplidar"` | 创建软链接 `/dev/rplidar` | 程序中用这个固定名称 |
| `MODE="0666"` | 权限：所有用户可读写 | 避免 sudo 才能访问 |

## 关键概念

### idVendor / idProduct

- `10c4:ea60` = Silicon Labs CP210x 系列 USB 转串口芯片
- RPLIDAR A2 和 HI12 IMU **都用了同一款芯片**（CP210x），所以光靠 idVendor + idProduct 无法区分
- **必须用 serial（序列号）来区分**同芯片的不同设备

### serial（序列号）

- 每个 USB 设备出厂时烧录的唯一编号
- 即使型号相同，序列号也不同
- 是区分同型号多设备的核心字段

### ttyUSB

- Linux 对 USB 转串口设备的默认命名
- 编号按插入顺序分配：第一个 `ttyUSB0`，第二个 `ttyUSB1`
- 拔插后编号可能变化，**不要在程序里写死**

### 软链接（SYMLINK）

- udev 规则创建的是软链接，不是真正的设备文件
- `/dev/rplidar -> ttyUSB0`，程序打开 `/dev/rplidar` 实际访问的是 `ttyUSB0`
- 每次设备插入，udev 重新匹配规则、更新软链接

## 查看设备信息的命令

```bash
# 查看当前所有 USB 串口设备
ls -la /dev/ttyUSB*

# 查看软链接指向
ls -la /dev/rplidar /dev/hi12_imu

# 查看设备的详细信息（厂商、产品、序列号）
udevadm info --name=/dev/ttyUSB0 --query=all | grep -i "vendor\|model\|serial"

# 查看所有 USB 设备
lsusb

# 查看设备供电情况
lsusb -t
```

## 添加新设备的步骤

```bash
# 1. 只插新设备，查看其序列号
udevadm info --name=/dev/ttyUSB0 --query=all | grep SERIAL_SHORT
# 输出：E: ID_SERIAL_SHORT=4faef7fdf8028c4eaed0c8cb06c33e9b

# 2. 写 udev 规则
sudo nano /etc/udev/rules.d/99-my-device.rules
# 内容：
# KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="你的序列号", SYMLINK+="你的别名", MODE="0666"

# 3. 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. 重新插拔设备，验证
ls -la /dev/你的别名
```

## 常见问题

### 两个设备指向同一个 ttyUSB

原因：换 USB 口后旧软链接没刷新。解决：

```bash
sudo udevadm trigger
ls -la /dev/rplidar /dev/hi12_imu
```

### 设备权限不够

症状：`Permission denied`。解决：确保 udev 规则里有 `MODE="0666"`，或将用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
# 重新登录生效
```

### RPLIDAR 和 HI12 用了同款芯片

两者都是 `10c4:ea60`（CP210x），必须靠 `ATTRS{serial}` 区分。如果规则里只写 idVendor + idProduct 而不写 serial，两个设备会创建相同的软链接，后到的覆盖先到的。
