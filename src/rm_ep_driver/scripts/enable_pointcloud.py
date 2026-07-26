#!/usr/bin/env python3
"""Enable D435i pointcloud output via dynamic_reconfigure."""
import subprocess
import time

time.sleep(2)  # Wait for camera node to initialize
result = subprocess.call([
    "rosrun", "dynamic_reconfigure", "dynparam", "set",
    "/camera/realsense2_camera", "enable_pointcloud", "true"
])
if result == 0:
    print("[OK] D435i pointcloud enabled")
else:
    print("[WARN] Failed to enable pointcloud via dynamic_reconfigure")
