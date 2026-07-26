#!/usr/bin/env python3
"""Diagnose D435i pointcloud issue."""
import subprocess
import time

print("=== D435i Pointcloud Diagnostic ===")

# Check 1: Is camera node running?
result = subprocess.run(["rosnode", "list"], capture_output=True, text=True)
camera_nodes = [n for n in result.stdout.split() if "realsense" in n]
print(f"1. Camera nodes: {camera_nodes if camera_nodes else \"NOT FOUND\"}")

# Check 2: Is enable_pointcloud parameter set?
result = subprocess.run(["rosparam", "get", "/camera/realsense2_camera/enable_pointcloud"], capture_output=True, text=True)
print(f"2. enable_pointcloud param: {result.stdout.strip() if result.returncode == 0 else \"NOT SET\"}")

# Check 3: Does /camera/depth/points topic exist?
result = subprocess.run(["rostopic", "list"], capture_output=True, text=True)
has_points = "/camera/depth/points" in result.stdout
print(f"3. /camera/depth/points topic: {\"EXISTS\" if has_points else \"NOT FOUND\"}")

# Check 4: Does /d435i/cloud_downsampled topic exist?
has_cloud = "/d435i/cloud_downsampled" in result.stdout
print(f"4. /d435i/cloud_downsampled topic: {\"EXISTS\" if has_cloud else \"NOT FOUND\"}")

# Try to enable pointcloud if not set
if not has_points and result.returncode == 0:
    print("5. Trying to enable pointcloud via dynamic_reconfigure...")
    try:
        proc = subprocess.run([
            ["rosrun", "dynamic_reconfigure", "dynparam", "set",
             "/camera/realsense2_camera", "enable_pointcloud", "true"],
            capture_output=True, text=True, timeout=10
        )
        print(f"   Result: {proc.stdout.strip() or proc.stderr.strip() or \"OK\"}")
    except subprocess.TimeoutExpired:
        print("   Timeout - service not available")
    except Exception as e:
        print(f"   Error: {e}")
else:
    print(\"5. Skipping enable - topic already exists or camera not running\")

print("=== Done ===")
