#!/usr/bin/env python3
"""Enable D435i pointcloud via rosparam set."""
import subprocess
import time

print("[1] Waiting 3s for camera node...")
time.sleep(3)

print("[2] Setting enable_pointcloud=true via rosparam...")
r = subprocess.run(["rosparam", "set", "/camera/realsense2_camera/enable_pointcloud", "true"], capture_output=True, text=True)
print("   stdout:", r.stdout.strip())
print("   stderr:", r.stderr.strip())

print("[3] Verifying...")
r = subprocess.run(["rosparam", "get", "/camera/realsense2_camera/enable_pointcloud"], capture_output=True, text=True)
print("   Value:", r.stdout.strip())

# Check topic
r = subprocess.run(["rostopic", "list"], capture_output=True, text=True)
if "/camera/depth/points" in r.stdout:
    print("[4] /camera/depth/points EXISTS - pointcloud enabled!")
else:
    print("[4] /camera/depth/points NOT FOUND")
    print("     Camera node may need restart to pick up new param")
