#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cloud_to_map.py
订阅 /camera/depth/points（xyz 或 xyzrgb，camera_depth_optical_frame，由 depth_to_pointcloud.py 发布），
用 tf2 查 camera_depth_optical_frame→map，手动旋转+平移投到 map 帧，
10cm 体素降采样 + 2Hz 限速 + 可选累积拼帧，发布 /d435i/cloud_map (map 帧，xyzrgb 若源带颜色)，供 WebRop/HL2 显示。

H5：若源点云带 rgb，颜色随点穿过变换/降采样/累积（累积集改成 dict{体素键: (r,g,b)}）；无 rgb 时退化为 xyz。
"""
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def quat_to_rotmat(qx, qy, qz, qw):
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)]], dtype=np.float64)


def unpack_rgb(packed_float32):
    """FLOAT32 打包的 rgb（字节 [r,g,b,*]）→ (N,3) uint8。"""
    p = np.ascontiguousarray(packed_float32, dtype=np.float32)
    return p.view(np.uint8).reshape(-1, 4)[:, :3].copy()


def pack_rgb_float(rgb_u8):
    """(N,3) uint8 → (N,) FLOAT32（字节 [r,g,b,255]），用于 PointCloud2 rgb 字段。"""
    packed = np.zeros((len(rgb_u8), 4), dtype=np.uint8)
    packed[:, :3] = rgb_u8
    packed[:, 3] = 255
    return np.ascontiguousarray(packed).view(np.float32).reshape(-1).copy()


class CloudToMap:
    def __init__(self):
        self.voxel = float(rospy.get_param("~voxel_size", 0.10))
        self.rate_hz = float(rospy.get_param("~rate_hz", 2.0))
        self.period = rospy.Duration(1.0 / max(self.rate_hz, 0.1))
        self.next_pub = rospy.Time.now()
        self.accumulate = bool(rospy.get_param("~accumulate", True))
        self.max_points = int(rospy.get_param("~max_points", 20000))
        self._accum = {}  # H4+H5：{体素键(ix,iy,iz): (r,g,b) 或 None}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("/d435i/cloud_map", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.cb_cloud, queue_size=1)
        rospy.loginfo("cloud_to_map 就绪: voxel=%.2f rate=%.1fHz accumulate=%s max=%d",
                      self.voxel, self.rate_hz, self.accumulate, self.max_points)

    def cb_cloud(self, msg):
        now = rospy.Time.now()
        if now < self.next_pub:
            return
        self.next_pub = now + self.period

        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time(0)
        try:
            trans = self.tf_buffer.lookup_transform("map", msg.header.frame_id, stamp, rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "cloud_to_map: TF %s→map 查不到: %s", msg.header.frame_id, e)
            return

        # 读点：有 rgb 字段就连色一起读
        has_rgb = any(f.name == "rgb" for f in msg.fields)
        if has_rgb:
            rec = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)))
            if rec.size == 0:
                return
            pts = np.column_stack([rec["x"], rec["y"], rec["z"]]).astype(np.float64)
            rgb = unpack_rgb(rec["rgb"].astype(np.float32))
        else:
            rec = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            if rec.size == 0:
                return
            pts = np.column_stack([rec["x"], rec["y"], rec["z"]]).astype(np.float64)
            rgb = None
        pts = pts.reshape(-1, 3)

        # 变换到 map（颜色不变）
        t = trans.transform.translation
        q = trans.transform.rotation
        R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        pts_map = pts @ R.T + np.array([t.x, t.y, t.z], dtype=np.float64)

        # 体素降采样（颜色跟着保留点走）
        frame_color = {}  # {体素键: (r,g,b) 或 None}
        if self.voxel > 0 and len(pts_map) > 0:
            key = np.floor(pts_map / self.voxel).astype(np.int64)
            _, idx = np.unique(key, axis=0, return_index=True)
            pts_map = pts_map[idx]
            keys_sel = key[idx]
            if rgb is not None:
                rgb_sel = rgb[idx]
                frame_color = {tuple(keys_sel[i]): tuple(int(c) for c in rgb_sel[i]) for i in range(len(idx))}
            else:
                frame_color = {tuple(k): None for k in keys_sel.tolist()}

        # 累积拼帧
        if self.accumulate:
            self._accum.update(frame_color)  # 新体素加入；旧体素颜色更新为最新
            if len(self._accum) > self.max_points:
                items = list(self._accum.items())
                self._accum = dict(items[-self.max_points:])
            keys = list(self._accum.keys())
            pts_out = np.array(
                [[k[0] * self.voxel + self.voxel / 2.0, k[1] * self.voxel + self.voxel / 2.0,
                  k[2] * self.voxel + self.voxel / 2.0] for k in keys], dtype=np.float32)
            any_color = any(v is not None for v in self._accum.values())
            if any_color:
                rgb_out = np.array(
                    [self._accum[k] if self._accum[k] is not None else (160, 160, 160) for k in keys],
                    dtype=np.uint8)
            else:
                rgb_out = None
        else:
            pts_out = pts_map.astype(np.float32)
            rgb_out = rgb

        # 发布
        header = rospy.Header()
        header.stamp = stamp
        header.frame_id = "map"
        if rgb_out is not None and len(rgb_out) == len(pts_out):
            rgb_f = pack_rgb_float(rgb_out)
            fields = [pc2.PointField("x", 0, pc2.PointField.FLOAT32, 1),
                      pc2.PointField("y", 4, pc2.PointField.FLOAT32, 1),
                      pc2.PointField("z", 8, pc2.PointField.FLOAT32, 1),
                      pc2.PointField("rgb", 12, pc2.PointField.FLOAT32, 1)]
            pts_l = pts_out.tolist()
            rgb_l = rgb_f.tolist()
            data = [(pts_l[i][0], pts_l[i][1], pts_l[i][2], rgb_l[i]) for i in range(len(pts_l))]
            cloud_out = pc2.create_cloud(header, fields, data)
        else:
            cloud_out = pc2.create_cloud_xyz32(header, pts_out.tolist())
        self.pub.publish(cloud_out)


if __name__ == "__main__":
    try:
        rospy.init_node("cloud_to_map")
        CloudToMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
