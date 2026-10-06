#!/usr/bin/env python3
import os
import json
import sys
import unittest

import cv2
import numpy as np


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts")
)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from calibration_tools import (  # noqa: E402
    accuracy_summary,
    build_checkerboard_points,
    calibrate_pinhole,
    calibration_gate,
    checkerboard_bbox_from_corners,
    detect_checkerboard_bbox,
    view_feature,
)


class CheckerboardCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.board = build_checkerboard_points(8, 6, 0.03)
        self.image_size = (1280, 720)
        self.camera_matrix = np.asarray(
            [
                [910.0, 0.0, 646.0],
                [0.0, 908.0, 358.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def test_checkerboard_uses_inner_corner_dimensions(self):
        self.assertEqual(self.board.shape, (48, 3))
        np.testing.assert_allclose(self.board[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(self.board[-1], [0.21, 0.15, 0.0])

    def test_synthetic_multiview_calibration_recovers_intrinsics(self):
        image_points = []
        features = []
        poses = [
            (-0.18, -0.10, 0.52, -0.20, 0.12, -0.08),
            (0.02, -0.12, 0.58, 0.14, -0.16, 0.06),
            (0.16, -0.08, 0.64, -0.10, -0.18, 0.14),
            (-0.16, 0.02, 0.70, 0.18, 0.10, -0.12),
            (0.00, 0.00, 0.76, -0.14, 0.16, 0.10),
            (0.14, 0.04, 0.82, 0.12, -0.10, -0.16),
            (-0.12, 0.10, 0.88, -0.08, -0.12, 0.18),
            (0.04, 0.12, 0.94, 0.16, 0.08, -0.04),
            (0.12, 0.10, 1.00, -0.12, 0.14, 0.04),
            (-0.06, -0.04, 0.62, 0.08, -0.06, 0.12),
            (0.08, -0.02, 0.72, -0.16, -0.08, -0.10),
            (-0.10, 0.06, 0.84, 0.10, 0.18, 0.08),
            (0.04, 0.08, 0.92, -0.06, 0.10, -0.14),
            (-0.04, -0.08, 0.68, 0.14, 0.04, 0.16),
            (0.10, 0.02, 0.78, -0.18, 0.06, -0.06),
        ]
        for tx, ty, tz, rx, ry, rz in poses:
            corners, _ = cv2.projectPoints(
                self.board,
                np.asarray([rx, ry, rz], dtype=np.float64),
                np.asarray([tx, ty, tz], dtype=np.float64),
                self.camera_matrix,
                np.zeros(5),
            )
            corners = corners.astype(np.float32)
            image_points.append(corners)
            features.append(view_feature(corners, self.image_size))

        result = calibrate_pinhole(
            image_points,
            self.image_size,
            self.board,
        )

        self.assertLess(result["rms"], 0.05)
        recovered = np.asarray(result["camera_matrix"])
        self.assertLess(abs(recovered[0, 0] - 910.0), 5.0)
        self.assertLess(abs(recovered[1, 1] - 908.0), 5.0)
        gate = calibration_gate(
            result,
            features,
            required_views=15,
            max_rms=0.5,
            max_view_p95=1.0,
            min_centroid_span=0.20,
            min_scale_span=0.02,
        )
        self.assertTrue(gate["passed"], gate["reasons"])

    def test_calibration_gate_rejects_clustered_views(self):
        result = {
            "rms": 0.20,
            "view_errors": [0.20] * 20,
        }
        features = [[0.50, 0.50, 0.20, 0.0]] * 20
        gate = calibration_gate(result, features, required_views=20)
        self.assertFalse(gate["passed"])
        self.assertIn("centroid coverage span", " ".join(gate["reasons"]))

    def test_checkerboard_bbox_covers_full_board_and_clamps(self):
        corners = np.asarray(
            [
                [[20.0, 15.0]],
                [[40.0, 15.0]],
                [[60.0, 15.0]],
                [[20.0, 35.0]],
                [[40.0, 35.0]],
                [[60.0, 35.0]],
            ],
            dtype=np.float32,
        )
        bbox = checkerboard_bbox_from_corners(
            corners,
            image_size=(80, 60),
            columns=3,
            rows=2,
            outer_square_margin=1.0,
        )
        self.assertEqual(bbox, (0, 0, 79, 55))

    def test_rendered_checkerboard_is_detected_as_bbox(self):
        image = np.full((600, 800, 3), 255, dtype=np.uint8)
        square = 60
        left = 100
        top = 80
        for row in range(7):
            for column in range(9):
                if (row + column) % 2 == 0:
                    x1 = left + column * square
                    y1 = top + row * square
                    image[y1:y1 + square, x1:x1 + square] = 0
        bbox = detect_checkerboard_bbox(
            image,
            columns=8,
            rows=6,
            outer_square_margin=1.0,
        )
        self.assertIsNotNone(bbox)
        self.assertLessEqual(abs(bbox[0] - left), 2)
        self.assertLessEqual(abs(bbox[1] - top), 2)
        self.assertLessEqual(abs(bbox[2] - (left + 9 * square - 1)), 2)
        self.assertLessEqual(abs(bbox[3] - (top + 7 * square - 1)), 2)


class AccuracyGateTest(unittest.TestCase):
    def test_accuracy_gate_passes_supported_three_dimensional_samples(self):
        expected = np.asarray([0.0, 0.0, 0.5])
        offsets = np.linspace(-0.006, 0.006, 19)
        points = [
            expected + np.asarray([offset, -offset / 2.0, offset / 3.0])
            for offset in offsets
        ]
        report = accuracy_summary(points, expected, attempts=20)
        self.assertTrue(report["passed"], report["reasons"])
        self.assertAlmostEqual(report["valid_rate"], 0.95)
        self.assertLess(report["median_error_m"], 0.02)
        self.assertLess(report["p95_error_m"], 0.04)

    def test_accuracy_gate_rejects_low_valid_rate_and_large_tail(self):
        expected = np.asarray([0.0, 0.0, 0.8])
        points = [
            expected + np.asarray([0.0, 0.0, error])
            for error in [0.005] * 14 + [0.08, 0.10]
        ]
        report = accuracy_summary(points, expected, attempts=20)
        self.assertFalse(report["passed"])
        reasons = " ".join(report["reasons"])
        self.assertIn("valid rate", reasons)
        self.assertIn("P95 error", reasons)

    def test_empty_accuracy_report_is_strict_json(self):
        report = accuracy_summary(
            [],
            [0.0, 0.0, 0.5],
            attempts=0,
        )
        self.assertFalse(report["passed"])
        json.dumps(report, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
