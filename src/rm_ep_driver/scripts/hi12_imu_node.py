#!/usr/bin/env python3
"""
HiPNUC HI12 AHRS 外置 IMU ROS1 驱动节点

通过 UART 串口读取 HI12 数据帧，解析 HiPNUC HI91 二进制协议，
提取四元数、角速度、加速度，发布 sensor_msgs/Imu 到 /imu 话题。

坐标系对齐：只要硬件安装时 HI12 模块 X 轴朝机器人前方、Y 轴朝左、Z 轴朝上，
输出的数据天然符合 ROS REP-103 标准，无需任何符号取反或旋转变换。

通信协议 (HiPNUC HI91 定长帧，详见 imu_cum_cn.pdf 第 30 页):
  协议文档下载: https://download.hipnuc.com/products/#attitude
  帧头: 0x5A 0xA5
  帧结构: [SOF(2), payload_len(2,小端), CRC(2,小端), payload(76)]
  payload_len: 固定 76 字节
  CRC-16/XMODEM: 多项式 0x1021, 初值 0x0000, 无反转, 校验范围 SOF+LEN+payload
  payload 固定结构 (HI91):
    偏移 0:  tag          uint8   (0x91)
    偏移 1:  main_status  uint16
    偏移 3:  temperature  int8    °C
    偏移 4:  air_pressure float32 Pa
    偏移 8:  system_time  uint32  ms
    偏移 12: acc_b        float32×3  G      (XYZ, 1G≈9.8m/s²)
    偏移 24: gyr_b        float32×3  deg/s  (XYZ)
    偏移 36: mag_b        float32×3  μT     (XYZ)
    偏移 48: roll         float32    deg
    偏移 52: pitch        float32    deg
    偏移 56: yaw          float32    deg
    偏移 60: quat         float32×4  WXYZ
"""

import struct
import threading

import rospy
import serial
from sensor_msgs.msg import Imu


# HiPNUC 协议常量
FRAME_HEADER = b'\x5A\xA5'
HI91_TAG = 0x91
HI91_PAYLOAD_LEN = 76
HI91_FRAME_LEN = 6 + HI91_PAYLOAD_LEN  # 82


def crc16_xmodem(data, crc=0):
    """CRC-16/XMODEM: 多项式 0x1021, 初值 0x0000, 无输入/输出反转"""
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

        self.port = rospy.get_param('~port', '/dev/hi12_imu')
        self.baud = rospy.get_param('~baud', 115200)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50)

        self.ser = None
        self._lock = threading.Lock()

        # 最新解析数据
        self._quat = None       # (w, x, y, z)
        self._gyro_rad = None   # (gx, gy, gz) rad/s
        self._accel_ms2 = None  # (ax, ay, az) m/s^2

        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id

        # HI91 自带 EKF 融合，姿态置信度高
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

        self._connect_serial()

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self._timer = rospy.Timer(rospy.Duration(1.0 / max(1, self.publish_rate)),
                    self._publish_callback)

        rospy.loginfo("HI12 IMU 驱动已启动 (port=%s, baud=%d, rate=%d Hz)",
                      self.port, self.baud, self.publish_rate)

    def _connect_serial(self):
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
        """串口读取线程 - 持续读取并解析 HI91 数据帧"""
        buf = bytearray()

        while self._running and not rospy.is_shutdown():
            try:
                if self.ser is None or not self.ser.is_open:
                    rospy.logwarn_throttle(5.0, "HI12 串口未打开，尝试重连...")
                    self._reconnect()
                    continue

                data = self.ser.read(self.ser.in_waiting or 64)
                if not data or not isinstance(data, bytes):
                    continue

                buf.extend(data)

                while len(buf) >= 6:
                    header_idx = buf.find(FRAME_HEADER)
                    if header_idx < 0:
                        buf.clear()
                        break

                    if header_idx > 0:
                        del buf[:header_idx]

                    if len(buf) < 6:
                        break

                    payload_len = struct.unpack('<H', buf[2:4])[0]

                    if payload_len != HI91_PAYLOAD_LEN:
                        # 非 HI91 帧（如 HI83 变长帧），跳过帧头重同步
                        del buf[:2]
                        continue

                    if len(buf) < HI91_FRAME_LEN:
                        break

                    frame = bytes(buf[:HI91_FRAME_LEN])
                    del buf[:HI91_FRAME_LEN]

                    # CRC 校验: SOF(2) + LEN(2) + payload
                    crc_received = struct.unpack('<H', frame[4:6])[0]
                    crc_calc = crc16_xmodem(frame[:4])
                    crc_calc = crc16_xmodem(frame[6:], crc_calc)

                    if crc_calc != crc_received:
                        rospy.logwarn_throttle(5.0, "HI12 帧校验失败 (calc=0x%04X, recv=0x%04X)，丢弃",
                                               crc_calc, crc_received)
                        continue

                    self._parse_hi91_payload(frame[6:])

            except serial.SerialException as e:
                rospy.logwarn_throttle(5.0, "HI12 串口读取异常: %s", e)
                self._reconnect()
            except Exception as e:
                rospy.logwarn_throttle(5.0, "HI12 数据处理异常: %s", e)

    def _parse_hi91_payload(self, payload):
        """解析 HI91 payload (76 字节固定结构)

        偏移 0:  tag uint8        偏移 24: gyr_b float32×3 (deg/s)
        偏移 1:  main_status u16  偏移 36: mag_b float32×3 (μT)
        偏移 3:  temperature i8   偏移 48: roll  float32 (deg)
        偏移 4:  air_pressure f32 偏移 52: pitch float32 (deg)
        偏移 8:  system_time u32  偏移 56: yaw   float32 (deg)
        偏移 12: acc_b float32×3  偏移 60: quat  float32×4 (WXYZ)
        """
        try:
            tag = payload[0]
            if tag != HI91_TAG:
                rospy.logwarn_throttle(5.0, "HI12 非 HI91 帧 (tag=0x%02X)，跳过", tag)
                return

            # acc_b: G -> m/s² (×9.80665)
            ax_g, ay_g, az_g = struct.unpack('<3f', payload[12:24])
            # gyr_b: deg/s -> rad/s (×pi/180)
            gx_dps, gy_dps, gz_dps = struct.unpack('<3f', payload[24:36])
            # quat: WXYZ
            qw, qx, qy, qz = struct.unpack('<4f', payload[60:76])

            with self._lock:
                self._quat = (qw, qx, qy, qz)
                self._gyro_rad = (gx_dps * 0.017453293,
                                  gy_dps * 0.017453293,
                                  gz_dps * 0.017453293)
                self._accel_ms2 = (ax_g * 9.80665,
                                   ay_g * 9.80665,
                                   az_g * 9.80665)

        except struct.error as e:
            rospy.logwarn_throttle(5.0, "HI12 数据解析失败: %s", e)

    def _publish_callback(self, event):
        """定时发布 IMU 消息"""
        if rospy.is_shutdown():
            return
        with self._lock:
            quat = self._quat
            gyro = self._gyro_rad
            accel = self._accel_ms2

        if quat is None or gyro is None or accel is None:
            return

        msg = self.imu_msg
        msg.header.stamp = rospy.Time.now()

        # HI91 四元数顺序 WXYZ，ROS Imu 需要 [x, y, z, w]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]
        msg.orientation.w = quat[0]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        self.imu_pub.publish(msg)

    def _reconnect(self):
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
        rospy.loginfo("HI12 IMU 驱动正在关闭...")
        self._running = False
        if self._timer is not None:
            self._timer.shutdown()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        if self._read_thread is not None:
            self._read_thread.join(timeout=0.5)
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
