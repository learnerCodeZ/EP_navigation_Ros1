#!/usr/bin/env python3
"""Convert D435i depth image to PointCloud2 with voxel downsampling."""
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2

class DepthToPointCloud:
    def __init__(self):
        rospy.init_node("depth_to_pointcloud")

        # Parameters
        self.voxel_size = rospy.get_param("~voxel_size", 0.05)  # 5cm default
        self.skip_frames = rospy.get_param("~skip_frames", 2)    # publish every N frames
        self.min_depth = rospy.get_param("~min_depth", 0.1)      # min range (m)
        self.max_depth = rospy.get_param("~max_depth", 5.0)      # max range (m)
        self.frame_count = 0

        self.pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.cb_depth)
        self.sub_info = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cb_info)
        self.K = None
        self.u = None
        self.v = None

        rospy.loginfo("DepthToPointCloud started (voxel=%.2fm, skip=%d, range=%.1f-%.1fm)",
                      self.voxel_size, self.skip_frames, self.min_depth, self.max_depth)

    def cb_info(self, msg):
        self.K = msg.K
        w, h = msg.width, msg.height
        self.u, self.v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

    def voxel_downsample(self, points, voxel_size):
        """Simple voxel grid downsampling using numpy."""
        if len(points) == 0:
            return points
        # Quantize to voxel grid
        quantized = np.floor(points / voxel_size).astype(np.int32)
        # Use unique to find voxel centers
        _, unique_idx = np.unique(quantized, axis=0, return_index=True)
        return points[unique_idx]

    def cb_depth(self, msg):
        if self.K is None or self.u is None:
            return

        self.frame_count += 1
        if self.frame_count % self.skip_frames != 0:
            return

        # Parse depth image
        dtype = np.uint16
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        depth_m = depth.astype(np.float32) / 1000.0

        # Camera intrinsics
        fx, fy = self.K[0], self.K[4]
        cx, cy = self.K[2], self.K[5]

        # Filter valid depth
        mask = (depth_m > self.min_depth) & (depth_m < self.max_depth) & np.isfinite(depth_m)

        if not np.any(mask):
            return

        # Generate 3D points (only for valid pixels)
        z = depth_m[mask]
        u_valid = self.u[mask]
        v_valid = self.v[mask]
        x = (u_valid - cx) * z / fx
        y = (v_valid - cy) * z / fy

        points = np.stack([x, y, z], axis=-1)

        # Voxel downsample
        if self.voxel_size > 0:
            points = self.voxel_downsample(points, self.voxel_size)

        # Create PointCloud2 message
        header = rospy.Header()
        header.stamp = msg.header.stamp
        header.frame_id = "camera_depth_optical_frame"
        fields = [point_cloud2.PointField("x", 0, point_cloud2.PointField.FLOAT32, 1),
                  point_cloud2.PointField("y", 4, point_cloud2.PointField.FLOAT32, 1),
                  point_cloud2.PointField("z", 8, point_cloud2.PointField.FLOAT32, 1)]
        cloud = point_cloud2.create_cloud(header, fields, points)
        self.pub.publish(cloud)

if __name__ == "__main__":
    try:
        DepthToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
