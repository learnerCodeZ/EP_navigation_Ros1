# D435i Integration Progress (2026-07-25 Updated)

## Current Status: All Complete

| Component | Status | Evidence |
|---|---|---|
| USB 3.0 Connection | OK | lsusb -t 5000M, PID 0x0B3A |
| Data Stream | OK | RGB 1280x720@30, depth 848x480@30, /d435i/scan 30Hz |
| TF Chain | OK | base_link -> camera_link -> camera_depth_frame |
| 2D Costmap | OK | /d435i/scan in local_costmap |
| 3D PointCloud | OK | /camera/depth/points ~2fps, visible in RViz |
| RViz Config | OK | PointCloud2 display, Fixed Frame=base_link |

## Key Decisions

1. RGB display: use rqt_image_view (not RViz, too slow on Jetson)
2. D435i??USB 3.0 port
3. 3D pointcloud: custom depth_to_pointcloud.py (realsense enable_pointcloud doesn't work in 2.3.2)
4. voxel_grid downsampling: skipped for now (nodelet remap issue)

## 3D PointCloud Architecture

depth image -> depth_to_pointcloud.py -> /camera/depth/points (PointCloud2)
                                          |
                                         RViz

- Input: /camera/depth/image_rect_raw (848x480@30fps)
- Output: /camera/depth/points (~2fps)
- Frame: camera_depth_optical_frame
- TF: base_link -> camera_link -> camera_depth_frame -> camera_depth_optical_frame

## Todo

- [ ] voxel_grid downsampling (pcl_ros nodelet remap issue)
- [ ] scan shape verification
- [ ] PointCloud transmission to HL2 (Phase 2)
- [ ] Coordinate alignment (Phase 4, deferred)