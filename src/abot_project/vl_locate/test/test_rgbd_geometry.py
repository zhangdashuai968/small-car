#!/usr/bin/env python3
import os
import sys
import unittest

import numpy as np


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from rgbd_geometry import (  # noqa: E402
    TimedFrameCache,
    compute_depth_candidate_roi,
    compute_zoom_geometry,
    depth_array_from_bytes,
    depth_points_in_rgb_bbox,
    estimate_target_point,
    image_array_from_bytes,
    select_detection,
    transform_points,
    transform_camera_matrix,
    validate_camera_matrix,
)


class ZoomGeometryTest(unittest.TestCase):
    def test_rgb_image_bytes_respect_row_step_without_cv_bridge(self):
        row0 = bytearray([1, 2, 3, 4, 5, 6, 99, 99])
        row1 = bytearray([7, 8, 9, 10, 11, 12, 99, 99])
        image = image_array_from_bytes(
            data=row0 + row1,
            width=2,
            height=2,
            step=8,
            encoding="rgb8",
        )
        self.assertEqual(image.shape, (2, 2, 3))
        self.assertEqual(image.tolist(), [
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ])

    def test_camera_matrix_tracks_crop_scale_and_padding(self):
        geometry = compute_zoom_geometry(
            width=640,
            height=480,
            crop_ratios=(0.43, 0.53, 0.75, 0.78),
        )
        self.assertEqual((geometry["x1"], geometry["y1"]), (275, 254))
        self.assertEqual((geometry["new_width"], geometry["start_x"]), (640, 0))
        self.assertGreater(geometry["start_y"], 0)

        k = [454.0, 0.0, 325.0, 0.0, 454.0, 243.0, 0.0, 0.0, 1.0]
        transformed = transform_camera_matrix(k, geometry)
        scale = geometry["scale"]
        self.assertAlmostEqual(transformed[0], 454.0 * scale)
        self.assertAlmostEqual(transformed[2], (325.0 - 275.0) * scale)
        self.assertAlmostEqual(
            transformed[5],
            (243.0 - 254.0) * scale + geometry["start_y"],
        )


class DetectionSelectionTest(unittest.TestCase):
    def test_target_class_then_highest_probability(self):
        detections = [
            {"class_name": "cup", "probability": 0.91},
            {"class_name": "bottle", "probability": 0.72},
            {"class_name": "bottle", "probability": 0.88},
        ]
        selected = select_detection(detections, "bottle")
        self.assertEqual(selected["class_name"], "bottle")
        self.assertAlmostEqual(selected["probability"], 0.88)

    def test_empty_target_class_uses_global_best(self):
        detections = [
            {"class_name": "cup", "probability": 0.91},
            {"class_name": "bottle", "probability": 0.88},
        ]
        self.assertEqual(select_detection(detections, "")["class_name"], "cup")


class TargetPointTest(unittest.TestCase):
    def test_depth_bytes_support_16uc1_padding_and_big_endian(self):
        little_rows = (
            bytearray([0xE8, 0x03, 0xD0, 0x07, 99, 99])
            + bytearray([0xB8, 0x0B, 0xA0, 0x0F, 99, 99])
        )
        little = depth_array_from_bytes(
            little_rows,
            width=2,
            height=2,
            step=6,
            encoding="16UC1",
            is_bigendian=False,
        )
        np.testing.assert_allclose(
            little,
            [[1.0, 2.0], [3.0, 4.0]],
            atol=1e-7,
        )

        big = depth_array_from_bytes(
            bytearray([0x03, 0xE8, 0x07, 0xD0]),
            width=2,
            height=1,
            step=4,
            encoding="16UC1",
            is_bigendian=True,
        )
        np.testing.assert_allclose(big, [[1.0, 2.0]], atol=1e-7)

    def test_depth_bytes_support_32fc1_and_reject_unknown_encoding(self):
        values = np.array([[0.5, np.nan], [1.25, 0.0]], dtype="<f4")
        decoded = depth_array_from_bytes(
            values.tobytes(),
            width=2,
            height=2,
            step=8,
            encoding="32FC1",
            is_bigendian=False,
        )
        self.assertAlmostEqual(decoded[0, 0], 0.5)
        self.assertTrue(np.isnan(decoded[0, 1]))
        self.assertAlmostEqual(decoded[1, 0], 1.25)
        with self.assertRaises(ValueError):
            depth_array_from_bytes(
                b"\x00\x00",
                width=1,
                height=1,
                step=2,
                encoding="mono16",
                is_bigendian=False,
            )

    def test_depth_roi_identity_geometry_is_bounded_around_rgb_bbox(self):
        camera_matrix = [
            100.0, 0.0, 5.0,
            0.0, 100.0, 4.0,
            0.0, 0.0, 1.0,
        ]
        roi = compute_depth_candidate_roi(
            rgb_bbox=(3.0, 2.0, 7.0, 6.0),
            rgb_camera_matrix=camera_matrix,
            depth_camera_matrix=camera_matrix,
            depth_to_rgb_rotation=np.eye(3),
            depth_to_rgb_translation=np.zeros(3),
            min_depth=0.5,
            max_depth=2.0,
            depth_width=10,
            depth_height=8,
            margin_px=1,
        )
        self.assertEqual(roi, (2, 1, 8, 7))

    def test_depth_roi_points_are_returned_in_rgb_frame(self):
        camera_matrix = [
            100.0, 0.0, 5.0,
            0.0, 100.0, 4.0,
            0.0, 0.0, 1.0,
        ]
        depth = np.ones((8, 10), dtype=np.float32)
        points = depth_points_in_rgb_bbox(
            depth_m=depth,
            depth_camera_matrix=camera_matrix,
            rgb_camera_matrix=camera_matrix,
            depth_to_rgb_rotation=np.eye(3),
            depth_to_rgb_translation=np.zeros(3),
            rgb_bbox=(3.0, 2.0, 7.0, 6.0),
            min_depth=0.15,
            max_depth=3.0,
            margin_px=1,
        )
        self.assertGreaterEqual(len(points), 16)
        projected_u = camera_matrix[0] * points[:, 0] / points[:, 2] + camera_matrix[2]
        projected_v = camera_matrix[4] * points[:, 1] / points[:, 2] + camera_matrix[5]
        self.assertTrue(np.all(projected_u >= 3.0))
        self.assertTrue(np.all(projected_u <= 7.0))
        self.assertTrue(np.all(projected_v >= 2.0))
        self.assertTrue(np.all(projected_v <= 6.0))

    def test_depth_points_are_transformed_into_rgb_frame_before_projection(self):
        rotation = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        translation = np.array([0.02, -0.01, 0.03], dtype=np.float64)
        depth_points = np.array(
            [[0.01, 0.02, 0.50], [-0.02, 0.03, 0.70]],
            dtype=np.float32,
        )

        rgb_points = transform_points(depth_points, rotation, translation)

        expected = np.array(
            [[0.00, 0.00, 0.53], [-0.01, -0.03, 0.73]],
            dtype=np.float64,
        )
        np.testing.assert_allclose(rgb_points, expected, atol=1e-7)

    def test_transformed_depth_cluster_localizes_in_720p_bbox(self):
        camera_matrix = [
            900.0, 0.0, 640.0,
            0.0, 900.0, 360.0,
            0.0, 0.0, 1.0,
        ]
        depth_points = np.array(
            [
                [x - 0.02, y + 0.01, 0.60]
                for x in np.linspace(-0.025, 0.025, 10)
                for y in np.linspace(-0.025, 0.025, 10)
            ],
            dtype=np.float32,
        )
        rgb_points = transform_points(
            depth_points,
            np.eye(3),
            np.array([0.02, -0.01, 0.0]),
        )

        point = estimate_target_point(
            points_xyz=rgb_points,
            camera_matrix=camera_matrix,
            bbox=(590, 310, 690, 410),
            min_depth=0.15,
            max_depth=3.0,
            central_roi_scale=0.6,
            min_points=30,
            depth_bin_size=0.02,
            min_cluster_fraction=0.10,
        )

        self.assertIsNotNone(point)
        self.assertAlmostEqual(point[0], 0.0, places=3)
        self.assertAlmostEqual(point[1], 0.0, places=3)
        self.assertAlmostEqual(point[2], 0.60, places=3)

    def test_nearest_supported_depth_cluster_wins(self):
        camera_matrix = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        object_points = np.array(
            [[x, y, 0.60] for x in np.linspace(-0.03, 0.03, 10)
             for y in np.linspace(-0.03, 0.03, 10)],
            dtype=np.float32,
        )
        background_points = np.array(
            [[x, y, 1.20] for x in np.linspace(-0.08, 0.08, 14)
             for y in np.linspace(-0.08, 0.08, 14)],
            dtype=np.float32,
        )
        invalid = np.array(
            [[float("nan"), 0.0, 0.5], [0.0, 0.0, 4.0]],
            dtype=np.float32,
        )
        points = np.vstack((object_points, background_points, invalid))

        point = estimate_target_point(
            points_xyz=points,
            camera_matrix=camera_matrix,
            bbox=(260, 180, 380, 300),
            min_depth=0.15,
            max_depth=3.0,
            central_roi_scale=0.6,
            min_points=30,
            depth_bin_size=0.02,
            min_cluster_fraction=0.10,
        )
        self.assertIsNotNone(point)
        self.assertAlmostEqual(point[2], 0.60, places=3)
        self.assertAlmostEqual(point[0], 0.0, places=3)
        self.assertAlmostEqual(point[1], 0.0, places=3)

    def test_insufficient_points_returns_none(self):
        point = estimate_target_point(
            points_xyz=np.array([[0.0, 0.0, 0.5]], dtype=np.float32),
            camera_matrix=[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            bbox=(250, 170, 390, 310),
            min_depth=0.15,
            max_depth=3.0,
            central_roi_scale=0.6,
            min_points=30,
            depth_bin_size=0.02,
            min_cluster_fraction=0.10,
        )
        self.assertIsNone(point)


class TimedFrameCacheTest(unittest.TestCase):
    def test_nearest_and_pinned_frames_are_bounded(self):
        cache = TimedFrameCache(history_size=3, pinned_size=2, ttl_sec=10.0)
        cache.add(1.00, "cloud-a")
        cache.add(1.05, "cloud-b")
        cache.add(1.20, "cloud-c")
        self.assertEqual(cache.nearest(1.04, 0.10), "cloud-b")
        self.assertTrue(cache.pin(42.0, 1.04, 0.10, now_sec=5.0))
        self.assertEqual(cache.get_pinned(42.0, now_sec=8.0), "cloud-b")

        cache.pin(43.0, 1.20, 0.10, now_sec=5.0)
        cache.pin(44.0, 1.20, 0.10, now_sec=5.0)
        self.assertIsNone(cache.get_pinned(42.0, now_sec=8.0))
        self.assertIsNone(cache.get_pinned(43.0, now_sec=20.1))


class CameraMatrixValidationTest(unittest.TestCase):
    def test_valid_720p_matrix_passes(self):
        validate_camera_matrix(
            [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0],
            width=1280,
            height=720,
        )

    def test_640p_factory_principal_point_is_rejected_for_320p(self):
        with self.assertRaises(ValueError) as error:
            validate_camera_matrix(
                [454.0, 0.0, 325.0, 0.0, 454.0, 243.0, 0.0, 0.0, 1.0],
                width=320,
                height=240,
            )
        self.assertIn("principal point", str(error.exception))

    def test_non_rigid_rotation_is_rejected(self):
        with self.assertRaises(ValueError) as error:
            transform_points(
                np.zeros((1, 3), dtype=np.float32),
                np.diag([1.0, 1.0, 2.0]),
                np.zeros(3),
            )
        self.assertIn("orthonormal", str(error.exception))


if __name__ == "__main__":
    unittest.main()
