# concrete_block_detector

Point-cloud replacement for the legacy `concrete_block_perception` path in the
wall-assembly stack. The C++ node discovers free concrete-block hypotheses from a
`sensor_msgs/PointCloud2`. It transforms each cloud to `world`, removes an
upward-facing RANSAC ground plane, clusters the remaining points, and estimates
each pose from a ground-constrained top slab. Partial faces are accepted only
when their observed spans fit the fixed cuboid dimensions. To avoid ambiguous
top-only returns and thin clutter, the observed long span must be at least
`minimum_long_axis_span_m` (default 0.70 m) and a secondary span at least
`minimum_secondary_axis_span_m` (default 0.30 m).
Markers always use the fixed concrete-block
dimensions **0.9 × 0.6 × 0.6 m**.

```bash
ros2 launch concrete_block_detector concrete_block_detector.launch.py
```

That standalone launch uses the raw `PointCloud2` transport. It caches valid
clouds until an explicit `~/discover_blocks` request.

For the wall-assembly pipeline, launch the detector with the persistent world
model that calls it on `SCENE_DISCOVERY`:

```bash
ros2 launch concrete_block_detector wall_assembly_perception.launch.py
```

Interfaces, relative to the `concrete_block_detector` node:

- Subscribe: `points` (the launch file defaults this to `/seyond/points`)
- Publish: `poses` (`geometry_msgs/PoseArray`, `world` frame)
- Publish: `markers` (`visualization_msgs/MarkerArray`, `world` frame)
- Serve: `~/discover_blocks` (`DiscoverBlocks`) — one detection pass over the latest valid cloud

The detector does not own IDs or world state. Its cloud callback only caches a
valid world-frame cloud; `~/discover_blocks` returns anonymous coarse poses.
`world_model_node` owns association, IDs, and the single atomic world update.

`scene_bounds` optionally crops the transformed cloud before ground removal and
clustering. The shipped bounds are calibrated to the three-block staging area
in `/home/vscode/Documents/2026-07-07-grip_at_top`, excluding the crane and
distant structures that otherwise resemble partial block faces. Disable it for
a new, uncalibrated workcell or replace `min_m`/`max_m` with that workcell's
world-frame envelope.

The wall-assembly launch uses Cloudini's `point_cloud_transport` plugin, so
the detector subscribes to `/seyond/points/cloudini` and decodes it in-process.
It does not start a `cloudini_topic_converter` bridge node. This requires the
`cloudini_ros` runtime package to be installed and sourced.

The node ignores clouds without a valid TF transform to `world` at their input
timestamp. An explicit discovery with no detections publishes an empty pose
array and `DELETEALL` marker. It supplies coarse poses only: it does not
register/refine poses, infer missing blocks, or update world-model state.
