# Phase 4: Compile Verification + Debugging Summary (Updated 2026-07-25)

## Debugging History

### Issue 1: librealsense2 packages not found
- Solution: Skip librealsense2, install ROS packages directly

### Issue 2: Launch file argument errors
- realsense2_camera 2.3.2 rs_camera.launch accepts almost no arguments
- Solution: Call rs_camera.launch with no arguments

### Issue 3: File encoding corruption (BOM)
- PowerShell Set-Content adds UTF-8 BOM
- BOM breaks Python shebang line
- Solution: Write with ASCII encoding, no BOM

### Issue 4: D435i not detected by system
- EP camera port is power-only, not USB data
- Solution: Find USB port connected to TX2 mainboard

### Issue 5: enable_pointcloud not working
- realsense2_camera 2.3.2 enable_pointcloud parameter doesn't work via:
  - rosparam (camera reads at startup only)
  - dynamic_reconfigure (service not available)
- Solution: Custom depth_to_pointcloud.py node converts depth image to PointCloud2

### Issue 6: voxel_grid nodelet not receiving data
- pcl_ros voxel_grid nodelet remap issue (subscribes to bond, not pointcloud)
- Solution: Skipped for now, use raw pointcloud directly

## Key Lessons

1. realsense2_camera 2.3.2 has minimal launch args - just call rs_camera.launch directly
2. EP camera port is NOT USB data - built-in camera uses SDK network stream
3. PowerShell encoding can corrupt XML/Python - use ASCII encoding
4. enable_pointcloud in realsense 2.3.2 doesn't work - use custom depth_to_pointcloud.py
5. pcl_ros voxel_grid nodelet remap is tricky - skip for now if issues

## Files Created/Modified

- src/rm_ep_driver/scripts/depth_to_pointcloud.py (depth image to PointCloud2)
- src/rm_ep_driver/scripts/enable_pointcloud.py (deprecated, kept for reference)
- src/rm_ep_driver/scripts/diagnose_pointcloud.py (diagnostic tool)
- src/rm_ep_driver/launch/d435i_pointcloud_downsample.launch (voxel_grid, has issues)
- src/rm_ep_driver/launch/d435i_depth_to_pointcloud.launch (depth to pointcloud)
- src/rm_ep_driver/rviz/d435i_debug.rviz (PointCloud2 display added)