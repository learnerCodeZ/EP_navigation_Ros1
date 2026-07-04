#!/usr/bin/env python3
"""
HiPNUC HI12 AHRS 外置 IMU ROS1 驱动节点

通过 UART 串口读取 HI12 数据帧，解析 HiPNUC 二进制协议，
提取四元数、角速度、加速度，发布 sensor_msgs/Imu 到 /imu 话题。

坐标系对齐：只要硬件安装时 HI12 模块 X 轴朝机器人前方、Y 轴朝左、Z 轴朝上，
输出的数据天然符合 ROS REP-103 标准，无需任何符号取反或旋转变换。

通信协议 (基于 HiPNUC 官方 SDK):
  帧头: 0x5A 0xA5
  帧结构: [0x5A, 0xA5, payload_len_L, payload_len_H, CRC_L, CRC_H, sub_items...]
  payload_len: 小端序，后续 payload 字节数 (不含帧头4字节和CRC2字节)
  CRC16-CCITT: 校验范围 [帧头(2) + payload_len(2)] + [payload]
  sub_items: [item_id(1byte), data..., item_id(1byte), data..., 0x00(end)]

数据类型 Item ID:
  0x91 - UID (4 bytes)
  0xA0 - 原始加速度 (3 x int16)
  0xA1 - 校准加速度 (3 x int16)
  0xA2 - 滤波加速度 (3 x int16) — 加速度计 float 版本也是此 ID
  0xA5 - 线性加速度 (3 x float32)
  0xB0 - 原始陀螺仪 (3 x int16)
  0xB1 - 校准陀螺仪 (3 x int16)
  0xB2 - 滤波陀螺仪 (3 x int16) — 陀螺仪 float 版本也是此 ID
  0xC0 - 原始磁力计 (3 x int16)
  0xD0 - 欧拉角 int16 (3 x int16, 单位 0.01°)
  0xD1 - 四元数 (4 x float32: w, x, y, z)
  0xD9 - 欧拉角 float (3 x float32, 单位 rad)
"""

import struct
import threading

import rospy
import serial
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# HiPNUC 协议常量
FRAME_HEADER = b'\x5A\xA5'

# Item ID (单字节)
ITEM_ID_UID = 0x91
ITEM_ID_ACC_RAW = 0xA0
ITEM_ID_ACC_CAL = 0xA1
ITEM_ID_ACC_FILTERED = 0xA2
ITEM_ID_ACC_LINEAR = 0xA5
ITEM_ID_GYO_RAW = 0xB0
ITEM_ID_GYO_CAL = 0xB1
ITEM_ID_GYO_FILTERED = 0xB2
ITEM_ID_MAG_RAW = 0xC0
ITEM_ID_EULAR_INT = 0xD0
ITEM_ID_QUAT = 0xD1
ITEM_ID_EULAR_FLOAT = 0xD9
ITEM_ID_END = 0x00

# Item ID -> 数据字节数 (不含 item_id 本身)
# int16 x3 = 6, float32 x3 = 12, float32 x4 = 16
ITEM_SIZES = {
    ITEM_ID_UID: 4,
    ITEM_ID_ACC_RAW: 6,
    ITEM_ID_ACC_CAL: 6,
    ITEM_ID_ACC_FILTERED: 6,
    ITEM_ID_ACC_LINEAR: 12,
    ITEM_ID_GYO_RAW: 6,
    ITEM_ID_GYO_CAL: 6,
    ITEM_ID_GYO_FILTERED: 6,
    ITEM_ID_MAG_RAW: 6,
    ITEM_ID_EULAR_INT: 6,
    ITEM_ID_QUAT: 16,
    ITEM_ID_EULAR_FLOAT: 12,
}


def crc16_ccitt(data, crc=0):
    """CRC16-CCITT 校验"""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


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
        self._gyro = None        # (gx, gy, gz) rad/s
        self._accel = None       # (ax, ay, az) m/s^2

        # ROS 发布者
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

        # IMU 消息模板
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id

        # 设置协方差
        self.imu_msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.02
        ]
        self.imu_msg.angular_velocity_covariance = [
            0.0001, 0, 0,
            0, 0.0001, 0,
            0, 0, 0.0001
        ]
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
        """串口读取线程 - 持续读取并解析 HI12 数据帧

        帧结构: [0x5A, 0xA5, payload_len_L, payload_len_H, CRC_L, CRC_H, payload...]
        payload 由多个 sub_item 组成: [item_id, data..., item_id, data..., 0x00]
        """
        buf = bytearray()

        while self._running and not rospy.is_shutdown():
            try:
                if self.ser is None or not self.ser.is_open:
                    rospy.logwarn_throttle(5.0, "HI12 串口未打开，尝试重连...")
                    self._reconnect()
                    continue

                data = self.ser.read(self.ser.in_waiting or 64)
                if not data:
                    continue

                buf.extend(data)

                # 解析完整帧
                while len(buf) >= 6:
                    # 查找帧头
                    header_idx = buf.find(FRAME_HEADER)
                    if header_idx < 0:
                        buf.clear()
                        break

                    # 丢弃帧头前的垃圾数据
                    if header_idx > 0:
                        del buf[:header_idx]

                    # 需要: header(2) + payload_len(2) + crc(2) = 6 字节
                    if len(buf) < 6:
                        break

                    # 解析 payload_len (小端序)
                    payload_len = struct.unpack('<H', buf[2:4])[0]

                    # 帧总长 = header(2) + payload_len(2) + crc(2) + payload
                    frame_len = 6 + payload_len

                    if frame_len > 256:
                        # 数据异常，丢弃帧头重同步
                        del buf[:2]
                        continue

                    # 检查帧是否完整
                    if len(buf) < frame_len:
                        break

                    # 提取一帧
                    frame = bytes(buf[:frame_len])
                    del buf[:frame_len]

                    # CRC 校验: header(2) + payload_len(2) + payload
                    crc_received = struct.unpack('<H', frame[4:6])[0]
                    crc_calc = crc16_ccitt(frame[0:4])
                    crc_calc = crc16_ccitt(frame[6:], crc_calc)

                    if crc_calc != crc_received:
                        rospy.logwarn_throttle(5.0, "HI12 帧校验失败 (calc=0x%04X, recv=0x%04X)，丢弃",
                                               crc_calc, crc_received)
                        continue

                    # 解析 sub_items
                    payload = frame[6:]
                    self._parse_sub_items(payload)

            except serial.SerialException as e:
                rospy.logwarn_throttle(5.0, "HI12 串口读取异常: %s", e)
                self._reconnect()
            except Exception as e:
                rospy.logwarn_throttle(5.0, "HI12 数据处理异常: %s", e)

    def _parse_sub_items(self, payload):
        """解析 payload 中的 sub_items

        格式: [item_id, data..., item_id, data..., 0x00(end)]
        """
        offset = 0
        while offset < len(payload):
            item_id = payload[offset]
            offset += 1

            if item_id == ITEM_ID_END:
                break

            if item_id not in ITEM_SIZES:
                # 未知 item，跳过
                break

            data_size = ITEM_SIZES[item_id]
            if offset + data_size > len(payload):
                break

            item_data = payload[offset:offset + data_size]
            offset += data_size

            self._parse_item(item_id, item_data)

    def _parse_item(self, item_id, data):
        """解析单个 sub_item 数据"""
        try:
            if item_id == ITEM_ID_QUAT:
                # 四元数: w, x, y, z (4 x float32, 小端序)
                if len(data) >= 16:
                    self._quaternion = struct.unpack('<4f', data[:16])

            elif item_id == ITEM_ID_EULAR_FLOAT:
                # 欧拉角 float: roll, pitch, yaw (3 x float32, 小端序, 单位 rad)
                pass  # 有四元数时不需要欧拉角

            elif item_id == ITEM_ID_EULAR_INT:
                # 欧拉角 int16: roll, pitch, yaw (3 x int16, 单位 0.01°)
                pass  # 有四元数时不需要欧拉角

            elif item_id in (ITEM_ID_GYO_RAW, ITEM_ID_GYO_CAL, ITEM_ID_GYO_FILTERED):
                # 陀螺仪 int16: gx, gy, gz (3 x int16)
                if len(data) >= 6:
                    raw = struct.unpack('<3h', data[:6])
                    # int16 原始值需要根据量程转换，HI12 默认陀螺仪量程 ±2000°/s
                    # 但我们优先使用 float 版本，int16 作为 fallback
                    if self._gyro is None:
                        # 粗略转换: raw / 32768 * 2000 * (pi/180)
                        self._gyro = tuple(v / 32768.0 * 2000.0 * 0.017453293 for v in raw)

            elif item_id == ITEM_ID_ACC_LINEAR:
                # 线性加速度 float: ax, ay, az (3 x float32, m/s^2)
                if len(data) >= 12:
                    self._accel = struct.unpack('<3f', data[:12])

            elif item_id in (ITEM_ID_ACC_RAW, ITEM_ID_ACC_CAL, ITEM_ID_ACC_FILTERED):
                # 加速度计 int16: ax, ay, az (3 x int16)
                if len(data) >= 6 and self._accel is None:
                    raw = struct.unpack('<3h', data[:6])
                    # int16 原始值，HI12 默认加速度量程 ±8g
                    self._accel = tuple(v / 32768.0 * 8.0 * 9.80665 for v in raw)

        except struct.error as e:
            rospy.logwarn_throttle(5.0, "HI12 数据解析失败 (item=0x%02X): %s", item_id, e)

    def _publish_callback(self, event):
        """定时发布 IMU 消息"""
        with self._lock:
            quat = self._quaternion
            gyro = self._gyro
            accel = self._accel

        if quat is None or gyro is None or accel is None:
            return

        msg = self.imu_msg
        msg.header.stamp = rospy.Time.now()

        # 四元数: HI12 输出 [w, x, y, z]
        # HI12 坐标系 Y 轴朝右 (与 ROS REP-103 Y 朝左相反)
        # 导致 yaw 方向相反，需要取反 yaw
        w, x, y, z = quat

        # 转欧拉角，取反 yaw，再转回四元数
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        yaw = -yaw
        q_ros = quaternion_from_euler(roll, pitch, yaw)

        msg.orientation.x = q_ros[0]
        msg.orientation.y = q_ros[1]
        msg.orientation.z = q_ros[2]
        msg.orientation.w = q_ros[3]

        # 角速度 (rad/s): gyro_z 取反，匹配翻转后的 yaw 方向
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = -gyro[2]

        # 线加速度 (m/s^2): ay 取反，匹配翻转后的 Y 轴方向
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = -accel[1]
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
