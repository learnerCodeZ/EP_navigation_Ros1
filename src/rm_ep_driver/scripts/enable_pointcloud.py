#!/usr/bin/env python3
"""Enable D435i pointcloud output via dynamic_reconfigure."""
import subprocess
import time
import sys

time.sleep(2)  # Wait for camera node to initialize
subprocess.call([
    "rosrun", "dynamic_reconfigure", "dynparam", "set",
    "/camera/realsense_camera", "enable_pointcloud", "true"
])
print("D435i pointcloud enabled")
