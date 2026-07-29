#!/usr/bin/env python3
"""Diagnose D435i pointcloud issue."""
import subprocess

print("=== D435i Pointcloud Diagnostic ===")

# Check 1: Camera nodes
r = subprocess.run(["rosnode", "list"], capture_output=True, text=True)
nodes = [n for n in r.stdout.split() if "realsense" in n]
print("1. Camera nodes:", nodes if nodes else "NOT FOUND")

# Check 2: enable_pointcloud param
r = subprocess.run(["rosparam", "get", "/camera/realsense2_camera/enable_pointcloud"], capture_output=True, text=True)
print("2. enable_pointcloud:", r.stdout.strip() if r.returncode == 0 else "NOT SET")

# Check 3: Topics
r = subprocess.run(["rostopic", "list"], capture_output=True, text=True)
print("3. /camera/depth/points:", "EXISTS" if "/camera/depth/points" in r.stdout else "NOT FOUND")
print("4. /d435i/cloud_downsampled:", "EXISTS" if "/d435i/cloud_downsampled" in r.stdout else "NOT FOUND")

# Try enable
if "/camera/depth/points" not in r.stdout and nodes:
    print("5. Enabling pointcloud...")
    r2 = subprocess.run(["rosrun", "dynamic_reconfigure", "dynparam", "set", "/camera/realsense2_camera", "enable_pointcloud", "true"], capture_output=True, text=True, timeout=10)
    print("   Result:", r2.stdout.strip() or r2.stderr.strip() or "OK")
print("=== Done ===")
