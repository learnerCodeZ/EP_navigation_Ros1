#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cloud_to_map.py
订阅 /camera/depth/points (camera_depth_optical_frame，由 depth_to_pointcloud.py 发布)，
用 tf2 查 camera_depth_optical_frame→map，手动旋转+平移投到 map 帧，
10cm 体素降采样 + 2Hz 限速，发布 /d435i/cloud_map (PointCloud2, map 帧)，供 WebRop/HL2 显示。

设计：
- 手动 TF 变换（不用 tf2_sensor_msgs），只依赖 tf2_ros + numpy，部署省心。
- 限速处理：整个「查 TF + 投影 + 降采样 + 发布」按 rate_hz 节流，避免 15fps 原始流压垮浏览器。
- 降采样：map 帧下做体素去重，10cm 室内典型 ~3-8k 点，rosbridge 传得动。
- stamp 用原始深度图时间戳（相机在动，必须用当时位姿）；stamp 为 0 时退化为最新 TF。
"""
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def quat_to_rotmat(qx, qy, qz, qw):
    """四元数 → 3x3 旋转矩阵（右手，ROS REP-103）。"""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ], dtype=np.float64)


class CloudToMap:
    def __init__(self):
        self.voxel = float(rospy.get_param("~voxel_size", 0.10))
        self.rate_hz = float(rospy.get_param("~rate_hz", 2.0))
        self.period = rospy.Duration(1.0 / max(self.rate_hz, 0.1))
        self.next_pub = rospy.Time.now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("/d435i/cloud_map", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.cb_cloud, queue_size=1)
        rospy.loginfo("cloud_to_map 就绪: voxel=%.2fm rate=%.1fHz（订阅 /camera/depth/points → 发布 /d435i/cloud_map）",
                      self.voxel, self.rate_hz)

    def cb_cloud(self, msg):
        # 限速：不到下一次发布时刻就跳过整帧处理
        now = rospy.Time.now()
        if now < self.next_pub:
            return
        self.next_pub = now + self.period

        # stamp 为 0（无时钟）时退化为最新 TF
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time(0)
        try:
            trans = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, stamp, rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "cloud_to_map: TF %s→map 查不到: %s", msg.header.frame_id, e)
            return

        pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                       dtype=np.float64)
        if pts.size == 0:
            return
        pts = pts.reshape(-1, 3)  # 保险：确保 (N,3)

        # 手动变换：p_map = (R · p) + t  ⇔  pts @ R.T + t
        t = trans.transform.translation
        q = trans.transform.rotation
        R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        pts_map = pts @ R.T + np.array([t.x, t.y, t.z], dtype=np.float64)

        # map 帧体素降采样
        if self.voxel > 0 and len(pts_map) > 0:
            key = np.floor(pts_map / self.voxel).astype(np.int64)
            _, idx = np.unique(key, axis=0, return_index=True)
            pts_map = pts_map[idx]

        header = rospy.Header()
        header.stamp = stamp
        header.frame_id = "map"
        cloud_out = pc2.create_cloud_xyz32(header, pts_map.tolist())
        self.pub.publish(cloud_out)


if __name__ == "__main__":
    try:
        rospy.init_node("cloud_to_map")
        CloudToMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
