#!/usr/bin/env python3
"""
HiPNUC HI12 AHRS 外置 IMU ROS1 驱动节点

通过 UART 串口读取 HI12 数据帧，解析 HiPNUC 二进制协议，
提取四元数、角速度、加速度，发布 sensor_msgs/Imu 到 /imu 话题。

坐标系对齐：只要硬件安装时 HI12 模块 X 轴朝机器人前方、Y 轴朝左、Z 轴朝上，
输出的数据天然符合 ROS REP-103 标准，无需任何符号取反或旋转变换。

通信协议：
  帧头: 0x5A 0xA5
  帧结构: [0x5A, 0xA5, frame_len_H, frame_len_L, data_id_H, data_id_L, payload..., checksum]
  frame_len: 从 frame_len_H/L 到帧尾的总字节数
  data_id: 数据类型标识
  checksum: 从 frame_len 开始到 checksum 前一字节的所有字节累加和取低 8 位

数据类型 ID:
  0x0090 - 四元数 (4 x float32: w, x, y, z)
  0x00A0 - 欧拉角 (3 x float32: roll, pitch, yaw 单位: rad)
  0x00B0 - 陀螺仪 (3 x float32: gx, gy, gz 单位: rad/s)
  0x00C0 - 加速度计 (3 x float32: ax, ay, az 单位: m/s^2)
  0x00D0 - 磁力计 (3 x float32: mx, my, mz)
"""

import struct
import threading

import rospy
import serial
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


# HiPNUC 协议常量
FRAME_HEADER = b'\x5A\xA5'

# 数据类型 ID
DATA_ID_QUATERNION = 0x0090
DATA_ID_EULER = 0x00A0
DATA_ID_GYROSCOPE = 0x00B0
DATA_ID_ACCELEROMETER = 0x00C0
DATA_ID_MAGNETOMETER = 0x00D0

# 数据类型 -> payload 字节数
DATA_SIZES = {
    DATA_ID_QUATERNION: 16,    # 4 x float32
    DATA_ID_EULER: 9,         # 3 x float32 (含 1 byte seq)
    DATA_ID_GYROSCOPE: 12,    # 3 x float32
    DATA_ID_ACCELEROMETER: 12, # 3 x float32
    DATA_ID_MAGNETOMETER: 12,  # 3 x float32
}


class HI12ImuDriver:
    """HiPNUC HI12 AHRS ROS1 驱动"""

    def __init__(self):
        rospy.init_node('hi12_imu_node', anonymous=False)

        # 加载参数
        self.port = rospy.get_param('~port', '/dev/hi12_imu')
        self.baud = rospy.get_param('~baud', 115200)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50)

        # 串口
        self.ser = None
        self._lock = threading.Lock()

        # 最新解析数据
        self._quaternion = None   # (w, x, y, z)
        self._euler = None       # (roll, pitch, yaw) rad
        self._gyro = None        # (gx, gy, gz) rad/s
        self._accel = None       # (ax, ay, az) m/s^2

        # ROS 发布者
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

        # IMU 消息模板
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id

        # 设置协方差（HI12 自带 EKF 姿态融合，姿态精度较高）
        # orientation_covariance: 对角线 [roll, pitch, yaw]
        # 姿态角精度约 0.3°~1.0°，设较小值表示高置信度
        self.imu_msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.02
        ]
        # angular_velocity_covariance
        self.imu_msg.angular_velocity_covariance = [
            0.0001, 0, 0,
            0, 0.0001, 0,
            0, 0, 0.0001
        ]
        # linear_acceleration_covariance
        self.imu_msg.linear_acceleration_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01
        ]

        # 连接串口
        self._connect_serial()

        # 启动读取线程
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        # 发布定时器
        rospy.Timer(rospy.Duration(1.0 / max(1, self.publish_rate)),
                     self._publish_callback)

        rospy.loginfo("HI12 IMU 驱动已启动 (port=%s, baud=%d, rate=%d Hz)",
                       self.port, self.baud, self.publish_rate)

    def _connect_serial(self):
        """连接 HI12 串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            rospy.loginfo("HI12 串口已连接: %s @ %d bps", self.port, self.baud)
        except serial.SerialException as e:
            rospy.logerr("无法打开 HI12 串口 %s: %s", self.port, e)
            raise

    def _read_loop(self):
        """串口读取线程 - 持续读取并解析 HI12 数据帧"""
        buf = bytearray()

        while self._running and not rospy.is_shutdown():
            try:
                # 读取可用数据
                if self.ser is None or not self.ser.is_open:
                    rospy.logwarn_throttle(5.0, "HI12 串口未打开，尝试重连...")
                    self._reconnect()
                    continue

                data = self.ser.read(self.ser.in_waiting or 64)
                if not data:
                    continue

                buf.extend(data)

                # 尝试解析完整帧
                while len(buf) >= 4:
                    # 查找帧头
                    header_idx = buf.find(FRAME_HEADER)
                    if header_idx < 0:
                        buf.clear()
                        break

                    # 丢弃帧头前的垃圾数据
                    if header_idx > 0:
                        del buf[:header_idx]

                    # 需要至少 frame_len(2) + data_id(2) = 6 字节 (不含帧头)
                    if len(buf) < 6:
                        break

                    # 解析 frame_len (大端序)
                    frame_len = struct.unpack('>H', buf[2:4])[0]

                    # 检查帧是否完整
                    if len(buf) < frame_len:
                        break

                    # 提取一帧
                    frame = bytes(buf[:frame_len])
                    del buf[:frame_len]

                    # 校验
                    if not self._verify_checksum(frame):
                        rospy.logwarn_throttle(5.0, "HI12 帧校验失败，丢弃")
                        continue

                    # 解析 data_id 和 payload
                    data_id = struct.unpack('>H', frame[4:6])[0]
                    payload = frame[6:-1]  # 去掉末尾的 checksum byte

                    self._parse_data(data_id, payload)

            except serial.SerialException as e:
                rospy.logwarn_throttle(5.0, "HI12 串口读取异常: %s", e)
                self._reconnect()
            except Exception as e:
                rospy.logwarn_throttle(5.0, "HI12 数据处理异常: %s", e)

    def _verify_checksum(self, frame):
        """校验帧 checksum

        checksum = 从 frame_len 的第 1 字节到 checksum 前一字节的所有字节累加和 & 0xFF
        帧结构: [0x5A, 0xA5, frame_len_H, frame_len_L, ..., checksum]
        frame[2:-1] 就是从 frame_len 到 checksum 前一字节
        """
        if len(frame) < 5:
            return False
        expected = sum(frame[2:-1]) & 0xFF
        actual = frame[-1]
        return expected == actual

    def _parse_data(self, data_id, payload):
        """根据 data_id 解析 payload 数据"""
        try:
            if data_id == DATA_ID_QUATERNION:
                # 四元数: w, x, y, z (4 x float32, 小端序)
                if len(payload) >= 16:
                    self._quaternion = struct.unpack('<4f', payload[:16])

            elif data_id == DATA_ID_EULER:
                # 欧拉角: roll, pitch, yaw (3 x float32, 小端序, 单位 rad)
                if len(payload) >= 12:
                    self._euler = struct.unpack('<3f', payload[:12])

            elif data_id == DATA_ID_GYROSCOPE:
                # 角速度: gx, gy, gz (3 x float32, 小端序, 单位 rad/s)
                if len(payload) >= 12:
                    self._gyro = struct.unpack('<3f', payload[:12])

            elif data_id == DATA_ID_ACCELEROMETER:
                # 加速度: ax, ay, az (3 x float32, 小端序, 单位 m/s^2)
                if len(payload) >= 12:
                    self._accel = struct.unpack('<3f', payload[:12])

            elif data_id == DATA_ID_MAGNETOMETER:
                pass  # 磁力计数据暂不使用
        except struct.error as e:
            rospy.logwarn_throttle(5.0, "HI12 数据解析失败 (data_id=0x%04X): %s", data_id, e)

    def _publish_callback(self, event):
        """定时发布 IMU 消息"""
        with self._lock:
            quat = self._quaternion
            gyro = self._gyro
            accel = self._accel

        # 至少需要四元数或欧拉角 + 角速度 + 加速度
        if quat is None or gyro is None or accel is None:
            return

        msg = self.imu_msg
        msg.header.stamp = rospy.Time.now()

        # 四元数: HI12 输出 [w, x, y, z]，ROS Imu 需要 [x, y, z, w]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]
        msg.orientation.w = quat[0]

        # 角速度 (rad/s): HI12 输出已经符合 ROS 坐标系，无需变换
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        # 线加速度 (m/s^2): HI12 输出已经符合 ROS 坐标系，无需变换
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        self.imu_pub.publish(msg)

    def _reconnect(self):
        """尝试重新连接串口"""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass

        rospy.loginfo_throttle(10.0, "尝试重新连接 HI12 串口 %s...", self.port)
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            rospy.loginfo("HI12 串口重连成功")
        except serial.SerialException:
            self.ser = None
            rospy.logwarn_throttle(10.0, "HI12 串口重连失败")

    def shutdown(self):
        """关闭驱动"""
        rospy.loginfo("HI12 IMU 驱动正在关闭...")
        self._running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        rospy.loginfo("HI12 IMU 驱动已关闭")


if __name__ == '__main__':
    driver = None
    try:
        driver = HI12ImuDriver()
        rospy.on_shutdown(driver.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("HI12 IMU 驱动异常退出: %s", e)
        if driver is not None:
            driver.shutdown()
