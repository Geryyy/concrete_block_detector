# Blockpose-to-C++ parity fixtures

`fixtures_manifest.json` is the versioned contract for the `DetectFree` C++
port.  It intentionally compares the C++ detector to the Python prototype,
not to synthetic or annotation ground truth.  A fixture can therefore record a
prototype miss caused by occlusion without turning that miss into a C++ bug.

Generate portable fixture artefacts from the workspace root:

```bash
PYTHONPATH=src/concrete_block_stack/blockpose/python \
  python3 src/concrete_block_stack/blockpose/tools/export_cpp_parity_fixture.py \
  --manifest src/concrete_block_stack/concrete_block_detector/test/parity/fixtures_manifest.json \
  --output /tmp/blockpose_cpp_parity
```

Each case writes `<id>.npz` (world-frame points) and `<id>.json` (the Python
result).  The JSON includes accepted poses, dimensions, evidence, diagnostics,
and deterministic stage counts; timings are deliberately excluded.  The real
snapshot export is optional and reads the existing `blockpose/data` tree; no
bag or point cloud is duplicated into this ROS package.

Comparison acceptance criteria:

| Property | Requirement |
| --- | --- |
| Accepted detection count | Exact |
| Correspondence | Minimum-cost one-to-one assignment after the port applies its documented stable ordering |
| Centre error | at most 0.05 m |
| Symmetry-aware orientation error | at most 5 degrees |
| Evidence score | absolute error at most 0.02 |
| Common integer stage counts | Exact |
| Repeated and shuffled input | Byte-identical serialised C++ result |

The tolerances allow normal C++/PCL versus NumPy/SciPy floating point rounding;
they do **not** allow a different detection route (for example the legacy
whole-cluster PCA cuboid).  New fixtures must be generated with the committed
Python prototype and reviewed with their source manifest change.

`1783428141_224458752_seq3` is included as a real-data availability and
prototype-parity case.  It is not a three-block-recall assertion: the current
prototype may miss the heavily occluded object.
