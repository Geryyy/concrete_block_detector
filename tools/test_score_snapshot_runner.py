#!/usr/bin/env python3
"""Focused standard-library regression test for score_snapshot_runner."""

import unittest

from score_snapshot_runner import _rotation_error_deg, score


class SnapshotRunnerScorerTest(unittest.TestCase):
    def test_roi_and_one_to_one_matching(self):
        target = {"id": "a", "position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1], "dimensions_m": [0.9, 0.6, 0.6]}
        manifest = {"snapshots": [{"snapshot": "one", "scoring_roi": {"min": [0, 0, 0], "max": [2, 2, 2]}, "targets": [target, {**target, "id": "b", "position_m": [0.8, 0, 0]}]}]}
        identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        runner = {"snapshots": [{"snapshot": "one", "poses": [{"position": [0.4, 0, 0], "rotation": identity, "dims": [0.9, 0.6, 0.6]}, {"position": [0.8, 0, 0], "rotation": identity, "dims": [0.9, 0.6, 0.6]}, {"position": [3, 0, 0], "rotation": identity, "dims": [0.9, 0.6, 0.6]}]}]}
        result = score(runner, manifest)
        self.assertEqual((result["targets"], result["predictions_scored_roi"], result["matched"]), (2, 2, 2))
        self.assertEqual(result["false_positives"], 0)

    def test_cuboid_symmetry_and_dimension_gate(self):
        target = {"position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1], "dimensions_m": [0.9, 0.6, 0.6]}
        quarter_turn_about_long_axis = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        prediction = {"position": [0, 0, 0], "rotation": quarter_turn_about_long_axis, "dims": [0.9, 0.6, 0.6]}
        self.assertAlmostEqual(_rotation_error_deg(prediction, target), 0.0)
        prediction["dims"] = [0.9, 0.7, 0.6]
        self.assertFalse(_rotation_error_deg(prediction, target) < float("inf"))


if __name__ == "__main__":
    unittest.main()
