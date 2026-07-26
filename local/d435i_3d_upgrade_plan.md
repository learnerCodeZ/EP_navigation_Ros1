# D435i 3D PointCloud Plan - Status Update (2026-07-25)

## Phase Status

| Phase | Content | Status |
|---|---|---|
| Phase 0 | Enable D435i pointcloud output | DONE (via depth_to_pointcloud.py) |
| Phase 1 | Downsample + RViz verification | PARTIAL (raw pointcloud works, voxel_grid skipped) |
| Phase 2 | Transmission to HL2 | TODO |
| Phase 3 | Platform/HL2 receive + render | TODO |
| Phase 4 | Coordinate alignment | DEFERRED |
| Phase 5 | Accumulated pointcloud | DEFERRED |

## What Was Done (Phase 0 + 1)

### Phase 0: Enable PointCloud
- realsense 2.3.2 enable_pointcloud parameter doesn't work via rosparam or dynamic_reconfigure
- Solution: Custom node depth_to_pointcloud.py converts depth image to PointCloud2
- Output: /camera/depth/points (~2fps)

### Phase 1: Downsample + RViz
- voxel_grid nodelet remap issue (subscribes to bond, not pointcloud topic)
- Skipped for now - raw pointcloud works in RViz
- RViz config updated: PointCloud2 display, Fixed Frame=base_link

## Remaining Work

1. **voxel_grid downsampling**: Fix nodelet remap, reduce ~300k points to ~10k
2. **PointCloud transmission**: WebSocket/TCP/rosbridge to HL2
3. **Coordinate alignment**: Map <-> HL2 world frame (Phase 4, deferred)
4. **Accumulated pointcloud**: Multi-frame stitching (Phase 5, deferred)

## Files Changed

- src/rm_ep_driver/scripts/depth_to_pointcloud.py (new)
- src/rm_ep_driver/launch/d435i_depth_to_pointcloud.launch (new)
- src/rm_ep_driver/launch/d435i_bringup.launch (updated)
- src/rm_ep_driver/launch/realsense_d435i.launch (simplified)
- src/rm_ep_driver/rviz/d435i_debug.rviz (PointCloud2 added)