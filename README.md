# concrete_block_detector

Point-cloud replacement for the legacy `concrete_block_perception` path in the
wall-assembly stack. The C++ node discovers free concrete-block hypotheses from a
`sensor_msgs/PointCloud2`. It transforms each cloud to `world`, removes an
upward-facing RANSAC ground plane, clusters the remaining points, and uses PCA
to produce a 6-DoF pose for each cluster. Partial faces are accepted provided
their observed PCA spans do not exceed the fixed cuboid dimensions; the sensor
viewpoint selects the visible-face offset when estimating the cuboid centre.
To avoid ambiguous top-only returns and thin clutter, the longest observed PCA
span must be at least `minimum_long_axis_span_m` (default 0.70 m; strictly
greater than 0.60 m), and a
second span must be at least `minimum_secondary_axis_span_m` (default 0.30 m).
The longest observed span is canonical local X (the 0.9 m block axis); the two
remaining 0.6 m axes retain cuboid symmetry.
Markers always use the fixed concrete-block
dimensions **0.9 × 0.6 × 0.6 m**.

```bash
ros2 launch concrete_block_detector concrete_block_detector.launch.py
```

That standalone launch uses the raw `PointCloud2` transport and leaves
world-model writes disabled; set `world_model_enabled:=true` only when
`world_model_node` is already running.

For the wall-assembly pipeline, launch the detector with the persistent world
model it updates:

```bash
ros2 launch concrete_block_detector wall_assembly_perception.launch.py
```

Interfaces, relative to the `concrete_block_detector` node:

- Subscribe: `points` (the launch file defaults this to `/seyond/points`)
- Publish: `poses` (`geometry_msgs/PoseArray`, `world` frame)
- Publish: `markers` (`visualization_msgs/MarkerArray`, `world` frame)
- Write: coarse, free blocks to `/world_model_node/upsert_block`

The detector refreshes `/world_model_node/get_coarse_blocks` and associates
nearby free observations with stable IDs before upserting them. This keeps the
canonical `BlockArray`, planner, and behavior tree supplied without retaining
the old image-segmentation, mask-cutout, tracking, or registration interfaces.

The wall-assembly launch uses Cloudini's `point_cloud_transport` plugin, so
the detector subscribes to `/seyond/points/cloudini` and decodes it in-process.
It does not start a `cloudini_topic_converter` bridge node. This requires the
`cloudini_ros` runtime package to be installed and sourced.

The node ignores clouds without a valid TF transform to `world` at their input
timestamp. A valid but empty/no-detection cloud publishes an empty pose array
and `DELETEALL` marker, clearing stale visualisations. It supplies coarse poses
only: it does not register/refine poses, infer missing blocks, or update blocks
that the world model marks as carried, placed, or removed.
