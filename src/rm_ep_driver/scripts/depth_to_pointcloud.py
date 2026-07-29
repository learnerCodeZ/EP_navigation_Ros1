#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert D435i depth image (+color) to PointCloud2 (xyzrgb) with voxel downsampling.

订阅 /camera/depth/image_rect_raw(+camera_info) 反投影成点(depth 光学帧)；
若 /camera/color/image_raw(+camera_info) 和 TF(camera_depth_optical→camera_color_optical)
都在，用 TF + 彩色内参把每个点投到彩色图取色，发 xyzrgb；
彩色不可用(TF/图缺)时退化为 xyz（原行为），保证不会因为加颜色把点云弄挂。
"""
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2


def quat_to_rotmat(qx, qy, qz, qw):
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)]], dtype=np.float64)


class DepthToPointCloud:
    def __init__(self):
        rospy.init_node("depth_to_pointcloud")

        self.voxel_size = rospy.get_param("~voxel_size", 0.05)  # 5cm
        self.skip_frames = rospy.get_param("~skip_frames", 2)
        self.min_depth = rospy.get_param("~min_depth", 0.1)
        self.max_depth = rospy.get_param("~max_depth", 5.0)
        self.frame_count = 0

        # 深度
        self.K = None           # 深度内参 K
        self.u = None
        self.v = None
        # 彩色
        self.cK = None          # 彩色内参 K
        self.cw = 0
        self.ch = 0
        self.color = None       # HxWx3 uint8 (rgb)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=1)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.cb_depth)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cb_depth_info)
        rospy.Subscriber("/camera/color/image_raw", Image, self.cb_color)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cb_color_info)

        rospy.loginfo("DepthToPointCloud started (voxel=%.2f, skip=%d, range=%.1f-%.1f, color=auto)",
                      self.voxel_size, self.skip_frames, self.min_depth, self.max_depth)

    def cb_depth_info(self, msg):
        self.K = msg.K
        w, h = msg.width, msg.height
        self.u, self.v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

    def cb_color_info(self, msg):
        self.cK = msg.K
        self.cw, self.ch = msg.width, msg.height

    def cb_color(self, msg):
        ch = msg.step // msg.width if msg.width else 3
        if ch < 3:
            ch = 3
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
        except ValueError:
            return
        if msg.encoding and msg.encoding.lower().startswith("bgr"):
            arr = arr[..., ::-1].copy()  # bgr → rgb
        self.color = arr

    def voxel_downsample(self, points, voxel_size):
        if len(points) == 0:
            return points
        q = np.floor(points / voxel_size).astype(np.int32)
        _, idx = np.unique(q, axis=0, return_index=True)
        return points[idx]

    def cb_depth(self, msg):
        if self.K is None or self.u is None:
            return
        self.frame_count += 1
        if self.frame_count % self.skip_frames != 0:
            return

        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        fx, fy = self.K[0], self.K[4]
        cx, cy = self.K[2], self.K[5]
        mask = (depth > self.min_depth) & (depth < self.max_depth) & np.isfinite(depth)
        if not np.any(mask):
            return

        z = depth[mask]
        uv = self.u[mask]
        vv = self.v[mask]
        x = (uv - cx) * z / fx
        y = (vv - cy) * z / fy
        pts = np.stack([x, y, z], axis=-1).astype(np.float32)

        if self.voxel_size > 0:
            pts = self.voxel_downsample(pts, self.voxel_size)

        header = rospy.Header()
        header.stamp = msg.header.stamp
        header.frame_id = "camera_depth_optical_frame"

        rgb = self._sample_color(pts, msg.header.stamp)
        if rgb is None:
            cloud = point_cloud2.create_cloud_xyz32(header, pts.tolist())
        else:
            # rgb 打包成 FLOAT32（标准 rgb 字段：4 字节 = [r,g,b,255]）
            packed = np.zeros((len(pts), 4), dtype=np.uint8)
            packed[:, :3] = rgb
            packed[:, 3] = 255
            rgb_f = np.ascontiguousarray(packed).view(np.float32).reshape(-1)
            fields = [point_cloud2.PointField("x", 0, point_cloud2.PointField.FLOAT32, 1),
                      point_cloud2.PointField("y", 4, point_cloud2.PointField.FLOAT32, 1),
                      point_cloud2.PointField("z", 8, point_cloud2.PointField.FLOAT32, 1),
                      point_cloud2.PointField("rgb", 12, point_cloud2.PointField.FLOAT32, 1)]
            pts_l = pts.tolist()
            rgb_l = rgb_f.tolist()
            data = [(pts_l[i][0], pts_l[i][1], pts_l[i][2], rgb_l[i]) for i in range(len(pts_l))]
            cloud = point_cloud2.create_cloud(header, fields, data)
        self.pub.publish(cloud)

    def _sample_color(self, pts, stamp):
        """把 depth 帧的点经 TF 变到 color 帧后投影取色。缺彩色/TF 时返回 None（退化为 xyz）。"""
        if self.color is None or self.cK is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform(
                "camera_color_optical_frame", "camera_depth_optical_frame", stamp, rospy.Duration(0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        q = tr.transform.rotation
        t = tr.transform.translation
        R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        pc = pts.astype(np.float64) @ R.T + np.array([t.x, t.y, t.z], dtype=np.float64)
        fxc, fyc = self.cK[0], self.cK[4]
        cxc, cyc = self.cK[2], self.cK[5]
        valid = pc[:, 2] > 1e-3
        uc = np.full(len(pc), -1, dtype=np.int32)
        vc = np.full(len(pc), -1, dtype=np.int32)
        uc[valid] = (pc[valid, 0] * fxc / pc[valid, 2] + cxc).astype(np.int32)
        vc[valid] = (pc[valid, 1] * fyc / pc[valid, 2] + cyc).astype(np.int32)
        inbound = (uc >= 0) & (uc < self.cw) & (vc >= 0) & (vc < self.ch)
        out = np.zeros((len(pc), 3), dtype=np.uint8)
        if np.any(inbound):
            out[inbound] = self.color[vc[inbound], uc[inbound]]
        return out


if __name__ == "__main__":
    try:
        DepthToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
