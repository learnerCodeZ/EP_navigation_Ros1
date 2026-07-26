#!/bin/bash
# Enable D435i pointcloud output
sleep 2
rosrun dynamic_reconfigure dynparam set /camera/realsense2_camera enable_pointcloud true
echo "[OK] D435i pointcloud enabled"