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

That standalone launch defaults to the Cloudini transport. It caches valid
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
valid world-frame cloud; `~/discover_blocks` returns anonymous fitted poses.
Top-only hypotheses are marked coarse; top-and-side hypotheses are marked
precise with conservative detector covariance.
`world_model_node` owns association, IDs, and the single atomic world update.

## Offline snapshot runner

`concrete_block_detector_snapshot_runner` runs the same C++ core on a recorded
Blockpose registration snapshot. It reads the snapshot's ASCII `cloud.pcd` and
per-snapshot `tf.yaml`, transforms the cloud into `world`, rebuilds the
ray-based evidence context, and writes JSON to stdout. For the deployed
`grip_at_top` configuration, pass the package defaults followed by the replay
overlay:

```bash
ros2 run concrete_block_detector concrete_block_detector_snapshot_runner \
  --params /workspaces/ros2_baustelle_ws/src/concrete_block_stack/concrete_block_detector/config/detector.yaml \
  --params /workspaces/ros2_baustelle_ws/src/concrete_block_stack/concrete_block_perception/config/grip_at_top_detector_scene_discovery.yaml \
  --snapshot /workspaces/ros2_baustelle_ws/src/concrete_block_stack/blockpose/data/registration_snapshots/1783428141_224458752_seq3
```

Repeat `--snapshot` to emit one JSON record per snapshot. The runner only
supports the repository's `DATA ascii`, `FIELDS x y z` PCD format and reports
an error for other encodings rather than silently changing detector input.

`scene_bounds` optionally crops the transformed cloud before ground removal and
clustering. The shipped bounds are calibrated to the three-block staging area
in `/home/vscode/Documents/2026-07-07-grip_at_top`, excluding the crane and
distant structures that otherwise resemble partial block faces. Disable it for
a new, uncalibrated workcell or replace `min_m`/`max_m` with that workcell's
world-frame envelope.

### Reviewed-snapshot regression score

`tools/reviewed_snapshot_manifest.json` contains the 23 reviewed annotation
ROIs and 61 target poses/dimensions. It intentionally contains no point clouds:
the snapshot directories remain external runner inputs. Generate runner JSON
with one `--snapshot <snapshots-dir>/<snapshot-name>` argument per manifest
entry, then score it without Blockpose modules or third-party Python packages:

```bash
python3 tools/score_snapshot_runner.py /tmp/snapshot_runner.json
python3 tools/test_score_snapshot_runner.py
```

The scorer admits a prediction only when its center lies in that snapshot's
reviewed ROI, then performs maximum-cardinality one-to-one assignment to
reviewed targets using a fixed 0.50 m center-distance compatibility threshold.
It reports actual precision, recall, and counts; it deliberately has no
hard-coded baseline expectation because those metrics are regression evidence,
not an acceptance threshold.

The wall-assembly launch uses Cloudini's `point_cloud_transport` plugin, so
the detector subscribes to `/seyond/points/cloudini` and decodes it in-process.
It does not start a `cloudini_topic_converter` bridge node. This requires the
`cloudini_ros` runtime package to be installed and sourced.

The node ignores clouds without a valid TF transform to `world` at their input
timestamp. An explicit discovery with no detections publishes an empty pose
array and `DELETEALL` marker. It fits cuboid poses from the cloud but does not
infer missing blocks or update world-model state.
