#!/usr/bin/env python3
"""Convert D435i depth image to PointCloud2."""
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2

class DepthToPointCloud:
    def __init__(self):
        rospy.init_node("depth_to_pointcloud")
        self.pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.cb_depth)
        self.sub_info = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cb_info)
        self.K = None
        rospy.loginfo("DepthToPointCloud node started")

    def cb_info(self, msg):
        self.K = msg.K

    def cb_depth(self, msg):
        if self.K is None:
            return
        dtype = np.uint16
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        depth_m = depth.astype(np.float32) / 1000.0
        fx, fy = self.K[0], self.K[4]
        cx, cy = self.K[2], self.K[5]
        h, w = depth_m.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth_m
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        mask = (z > 0.1) & (z < 10.0) & np.isfinite(z)
        points = np.stack([x[mask], y[mask], z[mask]], axis=-1)
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