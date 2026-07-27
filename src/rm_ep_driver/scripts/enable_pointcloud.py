#!/usr/bin/env python3
"""Enable D435i pointcloud - wait for camera to be ready."""
import subprocess
import time

print("[1] Waiting 5s for camera node to fully initialize...")
time.sleep(5)

# Set parameter
print("[2] Setting enable_pointcloud=true...")
r = subprocess.run(["rosparam", "set", "/camera/realsense2_camera/enable_pointcloud", "true"], capture_output=True, text=True)
print("   Result:", "OK" if r.returncode == 0 else r.stderr.strip())

# Verify
print("[3] Verifying parameter...")
r = subprocess.run(["rosparam", "get", "/camera/realsense2_camera/enable_pointcloud"], capture_output=True, text=True)
print("   Value:", r.stdout.strip())

# Check topic
r = subprocess.run(["rostopic", "list"], capture_output=True, text=True)
if "/camera/depth/points" in r.stdout:
    print("[4] SUCCESS: /camera/depth/points EXISTS")
else:
    print("[4] /camera/depth/points NOT FOUND")
    print("     NOTE: rosparam set after camera start may not work.")
    print("     Camera reads params at startup only.")
