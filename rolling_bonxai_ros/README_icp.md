# Rolling sonar ICP and clean surface mapping

The sonar configuration now combines robust local ICP with the cleaning used by `pointcloud_concatenator/script/sonar_icp_core.py`: beam-range coherence and per-keyframe hit/miss confirmation. The clean evidence lives in Bonxai voxel chunks, so neither a global point cloud nor keyframe history is retained in RAM.

## Data flow

1. Queue at most 60 incoming clouds and look up the recorded pose at the cloud timestamp plus `icp.pose_time_offset`. Drop unavailable poses after `icp.max_tf_wait`, logging the frame, timestamp, and TF error. With `icp.ignore_static_body_tf: true`, ignore only the static `frame_id -> base_frame_id` placeholder, preserving dynamic poses and all sensor extrinsics. This prevents recorded static body poses from poisoning the dynamic TF cache.
2. On a keyframe (translation, rotation, or elapsed time), project the sonar onto its angular beam grid. Keep beams with enough range-consistent neighbors and discard small eight-connected components. Missing/incoherent beams provide no free-space evidence.
3. Register the coherent scan against a point-capped local snapshot of surface candidates. Carry a planar correction transform across scans, keeping depth and tilt from the prior. Gate overlap, residual, increment size, and conditioning. Rejected registrations are not integrated.
4. Give each observed voxel at most one hit per accepted keyframe, independent of point count. Existing voxels receive a miss only when a coherent beam observes a surface farther away, accounting for range tolerance and occlusion.
5. Publish voxel centroids with at least two hit keyframes and `hits/(hits+misses) >= 0.7`. Keep separate point sums/counts for centroid/color averaging. These defaults match the original clean-map voter.

`sonar_clean_map.hpp/.cpp` implement the ROS-independent cleaner and rolling evidence store. Evidence uses a 3 cm floor-indexed voxel lattice, including negative coordinates. Compact 4×4×4 leaf blocks reduce unused allocations around thin surfaces. Hits, misses, counts, coordinate sums, and RGB sums are persisted together using Bonxai serialization.

Chunks use the configured size, neighborhood radius, and transition hysteresis. Only chunks containing evidence are allocated. Leaving the window saves dirty chunks atomically before eviction; returning loads their evidence. The chunk window must cover the sonar range, projection offset, voxel tolerance, and hysteresis margin. There is no growing in-memory index of historical chunks. I/O is synchronous in this initial version, avoiding an unbounded queue of dirty chunks awaiting save; disk latency can delay processing at transitions.

The sonar launch uses 1 m chunks with radius 11 (a 23 m window), replacing 4 m chunks with radius 3 (28 m). Its reference starts shifting past 0.7 m from the current chunk center. Keep `chunk_size * (radius - hysteresis_ratio) >= max_range + projection_offset.norm() + clean.voxel`; reducing chunk size without increasing radius can exclude valid sonar returns. Smaller chunks give finer transitions but cannot force eviction of surfaces still inside the sensor window. The supplied bag spans only about 2.0 × 2.45 × 0.8 m in position, so little or no evidence eviction is expected. Periodic console logs report ICP decisions, published points, resident chunks, saved chunks, and loaded chunks. Marker snapshots clear previous IDs so evicted chunks disappear.

## ROS outputs and services

- `/global_map_clean`: confirmed points from the **resident window**, in `reconstruction`, with transient-local QoS. It publishes empty outputs too, clearing stale geometry. `clean.publish_max_points` caps visualization memory (default 200,000 points).
- `~/corrected_sensor_pose`: corrected sensor pose at processed keyframes.
- `~/icp_diagnostics`: registration decisions plus clean cells, chunk count, memory, and published point count.
- `reconstruction -> map`: correction TF at keyframes and 10 Hz.
- Existing chunk statistics/markers describe the surface chunks in surface-only mode; quick memory statistics include evidence-grid allocations.
- `~/save_clean_map` (`std_srvs/srv/Trigger`): flush dirty evidence and stream **all saved chunks**, including evicted chunks, into `clean.save_path`. Export uses one additional chunk at a time and preserves the full-resolution confirmed map. It does not assemble a global cloud in RAM.

The node flushes resident clean chunks on normal shutdown. Chunk saves and PLY replacement use temporary files plus rename. A format manifest rejects incompatible evidence geometry. This supports rolling revisits during one mapping session; restoring the pose-correction state across separate launches is not implemented. The launch file therefore chooses a new storage directory each time. Do not reuse another session's evidence with a new identity correction.

## Surface-only versus occupancy mapping

`params/sonar_icp.yaml` enables cleaning and defaults to `clean.build_occupancy: false`. This avoids also allocating a free-space occupancy grid when the desired result is a clean point cloud. Set it to true if occupied/free-space mapping is required too; the coherent scans will then feed both representations and memory statistics include both.

The earlier ICP/occupancy mode remains available with `clean.enabled: false`. The original non-ICP server configuration is unchanged by default. The first cleaner supports the original default beam coherence and occupancy-voting behavior; optional final planarity filtering and pose-graph reconstruction are not implemented.

## Build and tests

From the Rolling-Bonxai repository:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/rolling_icp_build/log build \
  --base-paths . --build-base /tmp/rolling_icp_build/build \
  --install-base /tmp/rolling_icp_build/install \
  --packages-up-to rolling_bonxai_ros --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /tmp/rolling_icp_build/install/setup.bash
ctest --test-dir /tmp/rolling_icp_build/build/rolling_bonxai_ros \
  -R '^(test_body_tf_listener|test_icp_frontend|test_sonar_clean_map)$' --output-on-failure
```

Tests cover known planar-error recovery, tilt/depth preservation, degenerate-wall rejection, missing targets, configuration validation, coherence/speckle removal, one vote per keyframe, centroid/color averaging, projection offsets, missing-beam/occlusion behavior, free-space contradiction, disk reload, and full-history export. A 160 m test creates 41 surfaces, keeps at most seven evidence chunks resident, limits target snapshots, and exports all 41 surfaces after eviction.

## Run on the supplied bag

The default sonar configuration can play the original bag directly. It requires timestamped dynamic `map -> base_link` poses and filters the conflicting static body placeholder inside this node:

```bash
source /tmp/rolling_icp_build/install/setup.bash
ros2 launch rolling_bonxai_ros sonar_icp.launch.py
# In another terminal with the same environment:
ros2 bag play /home/daniel/Documents/rosbag/bluerov_pose_20260915_162225 --clock
```

Other TF consumers still see the original bag's conflict. To clean playback for all consumers, prepare a new sonar/TF bag. This preserves recorded extrinsics and removes the conflicting static `map -> base_link` transform only after checking that its dynamic counterpart exists. The original bag is read-only.

```bash
source /tmp/rolling_icp_build/install/setup.bash
ros2 run rolling_bonxai_ros prepare_sonar_playback.py \
  /home/daniel/Documents/rosbag/bluerov_pose_20260915_162225 \
  /tmp/my_sonar_playback
ros2 launch rolling_bonxai_ros sonar_icp.launch.py
# In another terminal with the same environment:
ros2 bag play /tmp/my_sonar_playback --clock
```

A prepared copy is also available from this experiment at `/tmp/rolling_icp_comparison/playback_clean`. Use a new output directory when preparing another copy. No playback process is left running by the experiment.

Display `/global_map_clean` with RViz's fixed frame set to `reconstruction`. To compare alongside the original Python node, remap one node's clean topic to avoid two publishers mixing clouds. Launch with `storage_path:=...` to choose a fresh evidence directory.

`/rolling_bonxai/occupied_voxels` is disabled in this surface-only configuration. The clean topic starts empty until at least two accepted keyframes confirm surfaces. If it stays empty, check the console's ICP reason and TF warnings or echo `/rolling_bonxai_icp/icp_diagnostics`. The node deliberately requires dynamic body poses in this mode; for a truly static setup, disable `icp.ignore_static_body_tf` in the parameters.

To save the complete map:

```bash
ros2 service call /rolling_bonxai_icp/save_clean_map std_srvs/srv/Trigger '{}'
```

The default full export is `/tmp/rolling_clean_map.ply`. Live cloud queues and target/publication caps remain bounded. Large maps can still consume disk space; total resident memory depends on the occupied geometry within the selected window and voxel resolution.

## Reproduce the deterministic comparison and reference check

Use new output names. The preparer refuses to replace its binary input; runners refuse to reuse existing output directories. Preparation supports one SQLite3 file and direct recorded world-to-base / base-to-sensor transforms. It interpolates poses, rejects long pose gaps, and never extrapolates. Every fifth retained scan is withheld from fitting and integration.

```bash
ros2 run rolling_bonxai_ros prepare_icp_comparison.py \
  /home/daniel/Documents/rosbag/bluerov_pose_20260915_162225 \
  /tmp/my_rolling_comparison/scans.bin
ros2 run rolling_bonxai_ros compare_icp \
  /tmp/my_rolling_comparison/scans.bin /tmp/my_rolling_comparison/old icp
ros2 run rolling_bonxai_ros compare_icp \
  /tmp/my_rolling_comparison/scans.bin /tmp/my_rolling_comparison/clean icp_clean
ros2 run rolling_bonxai_ros validate_sonar_clean_reference.py \
  /tmp/my_rolling_comparison/clean \
  /home/daniel/ros2_ws/src/pointcloud_concatenator/script
ros2 run rolling_bonxai_ros report_sonar_clean.py \
  /tmp/my_rolling_comparison/old /tmp/my_rolling_comparison/clean \
  /tmp/my_rolling_comparison/report
```

The reference check additionally requires Open3D and the original `sonar_icp_core.py`; neither is required by the C++ mapping node. Offline-only accepted keyframes are streamed to disk for the oracle check, never retained as history by the live mapper. It feeds identical accepted raw scans and corrected poses to the original Python preprocessor/voter and compares coherent counts and confirmed centroids. It validates cleaning equivalence, not equivalence of the two ICP trajectories.

Results and point clouds are in `experiments/bluerov_pose_20260915_162225/clean_refactor`. The bag comparison uses the shared engine offline, waiting for I/O. A separate live synthetic ROS test checks clean-topic publication and full-map export. Complete DDS playback of this bag is not part of the measured comparison.
