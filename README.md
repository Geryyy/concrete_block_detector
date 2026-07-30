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

## Optional pose priors

`DiscoverBlocks` accepts zero or more `PosePrior` records. The detector only
uses a prior to re-rank overlapping, point-cloud-supported hypotheses; it never
creates a block from a prior or relaxes geometric/ray-evidence gates. This is
one uniform seam for FK, RGB, wall-plan, and previously registered-block
knowledge. The detector's `pose_priors.fk.*` parameters create a timestamped
FK prior from `T_world_tcp`; the grip-at-top replay overlay enables it. The
world model can optionally forward registered poses and `goal_pose` wall-plan
poses through `scene_discovery.priors.*` (both disabled by default).

Active priors are published as translucent magenta cubes on `markers`; the
normal detected blocks remain orange.

## Module ablations and debug outlets

The deployed replay overlay exposes independent `modules.*.enabled` switches
for refinement, gripper self-filtering, FK/request priors, and the classical
RGB edge prior. A disabled module is a no-op; the numerical parameters remain
unchanged, so one YAML file supports controlled replay ablations. The node
reports each module as **enabled**, **available**, and **applied/gated** in a
versioned diagnostic JSON message.

With `debug.enabled:=true`, inspect these detector-owned request snapshots in
RViz or Foxglove (all use `world` except the RGB image):

- `/cbp/debug/scene_discovery/detector/input_cloud` — input after scene bounds
- `/cbp/debug/scene_discovery/detector/geometry_cloud` — after gripper removal
- `/cbp/debug/scene_discovery/detector/above_ground_cloud` — geometry input
- `/cbp/debug/scene_discovery/detector/markers` — gripper boxes, priors,
  candidates, rejected candidates, and final score contributions
- `/cbp/debug/scene_discovery/detector/rgb_input` — exact image selected for
  RGB evidence
- `/cbp/debug/scene_discovery/detector/diagnostics` — `std_msgs/String` JSON;
  module gates, counts, proposal gates, candidate lineage, and final evidence

The RGB module is only a bounded tie-breaker. Missing or gated RGB falls back
to geometry-only selection; it does not create poses.

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
Its schema-v2 `candidate_trace` is post-refinement, pre-NMS candidate data,
including source and separate geometry/prior/RGB contributions. It is the
offline ablation/DINO input, rather than a reconstructed Python candidate set.

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
reviewed targets using fixed center, dimension, and symmetry-aware orientation
thresholds. `tools/reviewed_snapshot_baseline.json` records the reproducible
reference result and hashes of the external inputs. It is evidence for changes,
not an accuracy promise for another workcell or bag.

The wall-assembly launch uses Cloudini's `point_cloud_transport` plugin, so
the detector subscribes to `/seyond/points/cloudini` and decodes it in-process.
It does not start a `cloudini_topic_converter` bridge node. This requires the
`cloudini_ros` runtime package to be installed and sourced.

The node ignores clouds without a valid TF transform to `world` at their input
timestamp. An explicit discovery with no detections publishes an empty pose
array and `DELETEALL` marker. It fits cuboid poses from the cloud but does not
infer missing blocks or update world-model state.
# Blockpose core dependency

`concrete_block_detector` consumes the portable, Eigen-only `blockpose_core`
CMake package. Configure with its installed prefix on `CMAKE_PREFIX_PATH`; no
source-tree include path is supported:

```bash
colcon build --packages-select concrete_block_detector \
  --cmake-args -DCMAKE_PREFIX_PATH=/path/to/blockpose-core-install
```
