#!/usr/bin/env python3
"""Score snapshot-runner JSON against the checked-in reviewed targets."""

import argparse
import json
import math
from pathlib import Path
from typing import Any

MATCH_DISTANCE_M = 0.50
DIMENSION_ATOL_M = 0.03
MAX_ROTATION_DEG = 30.0


def _inside_roi(position: list[float], roi: dict[str, list[float]]) -> bool:
    return all(lo <= value <= hi for value, lo, hi in zip(position, roi["min"], roi["max"]))


def _distance(first: list[float], second: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _matmul(first: list[list[float]], second: list[list[float]]) -> list[list[float]]:
    return [
        [sum(first[row][axis] * second[axis][column] for axis in range(3)) for column in range(3)]
        for row in range(3)
    ]


def _transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[column][row] for column in range(3)] for row in range(3)]


def _quaternion_matrix(quaternion: list[float]) -> list[list[float]]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("target orientation quaternion must be nonzero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _frame_alignments(dims_from: list[float], dims_to: list[float]) -> list[list[list[float]]]:
    """Proper signed permutations which re-express ``dims_from`` as ``dims_to``."""
    matrices = []
    for permutation in ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)):
        for signs in ((a, b, c) for a in (1.0, -1.0) for b in (1.0, -1.0) for c in (1.0, -1.0)):
            matrix = [[0.0] * 3 for _ in range(3)]
            for row, source in enumerate(permutation):
                matrix[row][source] = signs[row]
            determinant = (
                matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
                - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
                + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
            )
            dimensions_match = all(
                abs(dims_from[source] - dims_to[row]) <= DIMENSION_ATOL_M
                for row, source in enumerate(permutation)
            )
            if determinant > 0.0 and dimensions_match:
                matrices.append(_transpose(matrix))
    return matrices


def _rotation_error_deg(prediction: dict[str, Any], target: dict[str, Any]) -> float:
    target_rotation = _quaternion_matrix(target["orientation_xyzw"])
    best = math.inf
    for alignment in _frame_alignments(prediction["dims"], target["dimensions_m"]):
        relative = _matmul(_transpose(_matmul(prediction["rotation"], alignment)), target_rotation)
        cosine = max(
            -1.0,
            min(1.0, (sum(relative[index][index] for index in range(3)) - 1.0) / 2.0),
        )
        best = min(best, math.degrees(math.acos(cosine)))
    return best


def _compatible(target: dict[str, Any], prediction: dict[str, Any]) -> tuple[bool, float, float]:
    distance = _distance(target["position_m"], prediction["position"])
    rotation = _rotation_error_deg(prediction, target)
    return distance <= MATCH_DISTANCE_M and rotation <= MAX_ROTATION_DEG, distance, rotation


def _maximum_matching(targets: list[dict[str, Any]], predictions: list[dict[str, Any]]) -> list[tuple[int, int, float]]:
    """Return a maximum-cardinality, deterministic one-to-one matching."""
    edges = []
    for target in targets:
        compatible = []
        for prediction_index, prediction in enumerate(predictions):
            compatible_match, distance, _ = _compatible(target, prediction)
            if compatible_match:
                compatible.append((prediction_index, distance))
        edges.append(sorted(compatible, key=lambda item: item[1]))

    assigned_target: dict[int, int] = {}

    def assign(target_index: int, seen: set[int]) -> bool:
        for prediction_index, _ in edges[target_index]:
            if prediction_index in seen:
                continue
            seen.add(prediction_index)
            incumbent = assigned_target.get(prediction_index)
            if incumbent is None or assign(incumbent, seen):
                assigned_target[prediction_index] = target_index
                return True
        return False

    for target_index in range(len(targets)):
        assign(target_index, set())
    return sorted(
        (target_index, prediction_index, _distance(targets[target_index]["position_m"], predictions[prediction_index]["position"]))
        for prediction_index, target_index in assigned_target.items())


def score(runner: dict[str, Any], manifest: dict[str, Any]) -> dict[str, Any]:
    manifest_by_name = {item["snapshot"]: item for item in manifest["snapshots"]}
    runner_by_name = {item["snapshot"]: item for item in runner["snapshots"]}
    unexpected = sorted(set(runner_by_name) - set(manifest_by_name))
    if unexpected:
        raise ValueError("runner contains snapshots absent from manifest: " + ", ".join(unexpected))

    totals = {"targets": 0, "predictions_scored_roi": 0, "matched": 0}
    snapshots = []
    for name, expected in manifest_by_name.items():
        actual = runner_by_name.get(name)
        if actual is None:
            raise ValueError("runner is missing manifest snapshot: " + name)
        predictions = [pose for pose in actual.get("poses", []) if _inside_roi(pose["position"], expected["scoring_roi"])]
        matches = _maximum_matching(expected["targets"], predictions)
        counts = {"targets": len(expected["targets"]), "predictions_scored_roi": len(predictions), "matched": len(matches)}
        for key, value in counts.items():
            totals[key] += value
        snapshots.append(
            {
                "snapshot": name,
                **counts,
                "unmatched_targets": counts["targets"] - counts["matched"],
                "unmatched_predictions": counts["predictions_scored_roi"] - counts["matched"],
            }
        )

    matched, predictions, targets = totals["matched"], totals["predictions_scored_roi"], totals["targets"]
    return {
        "schema_version": 1,
        "matcher": {
            "center_distance_compatibility_m": MATCH_DISTANCE_M,
            "dimension_sorted_atol_m": DIMENSION_ATOL_M,
            "max_rotation_deg": MAX_ROTATION_DEG,
            "assignment": "maximum_cardinality_one_to_one",
        },
        "snapshots_scored": len(snapshots),
        **totals,
        "false_positives": predictions - matched,
        "false_negatives": targets - matched,
        "precision": matched / predictions if predictions else 0.0,
        "recall": matched / targets if targets else 0.0,
        "snapshots": snapshots,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner_json", type=Path, help="JSON emitted by concrete_block_detector_snapshot_runner")
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("reviewed_snapshot_manifest.json"))
    arguments = parser.parse_args()
    try:
        result = score(json.loads(arguments.runner_json.read_text()), json.loads(arguments.manifest.read_text()))
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
