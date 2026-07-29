#!/usr/bin/env python3
"""Focused standard-library regression test for score_snapshot_runner."""

import unittest

from score_snapshot_runner import score


class SnapshotRunnerScorerTest(unittest.TestCase):
    def test_roi_and_one_to_one_matching(self):
        manifest = {"snapshots": [{"snapshot": "one", "scoring_roi": {"min": [0, 0, 0], "max": [2, 2, 2]}, "targets": [{"id": "a", "position_m": [0, 0, 0]}, {"id": "b", "position_m": [0.8, 0, 0]}]}]}
        runner = {"snapshots": [{"snapshot": "one", "poses": [{"position": [0.4, 0, 0]}, {"position": [0.8, 0, 0]}, {"position": [3, 0, 0]}]}]}
        result = score(runner, manifest)
        self.assertEqual((result["targets"], result["predictions_scored_roi"], result["matched"]), (2, 2, 2))
        self.assertEqual(result["false_positives"], 0)


if __name__ == "__main__":
    unittest.main()
