#!/usr/bin/env python3
import os
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np


NODE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "node"))
if NODE_DIR not in sys.path:
    sys.path.insert(0, NODE_DIR)

from yolov5_runtime import (  # noqa: E402
    backend_kind,
    compute_letterbox_geometry,
    numpy_non_max_suppression,
    resolve_device,
    scale_boxes_to_original,
)


class DeviceSelectionTest(unittest.TestCase):
    def test_explicit_device_has_priority_over_legacy_use_cpu(self):
        self.assertEqual(
            resolve_device("0", legacy_use_cpu=True, cuda_available=True),
            "0",
        )

    def test_legacy_use_cpu_is_boolean_and_fail_closed(self):
        self.assertEqual(
            resolve_device(None, legacy_use_cpu=True, cuda_available=True),
            "cpu",
        )
        self.assertEqual(
            resolve_device(None, legacy_use_cpu=False, cuda_available=True),
            "0",
        )
        with self.assertRaises(RuntimeError):
            resolve_device("0", legacy_use_cpu=False, cuda_available=False)
        with self.assertRaises(ValueError):
            resolve_device("1", legacy_use_cpu=False, cuda_available=True)

    def test_only_pt_and_engine_backends_are_accepted(self):
        self.assertEqual(backend_kind("/tmp/model.pt"), "pytorch")
        self.assertEqual(backend_kind("/tmp/model.engine"), "tensorrt")
        with self.assertRaises(ValueError):
            backend_kind("/tmp/model.onnx")


class PackageContractTest(unittest.TestCase):
    def test_default_launch_uses_yolov5n_cuda_without_debug_image(self):
        package_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..")
        )
        root = ET.parse(
            os.path.join(package_root, "launch", "abot_yolov5.launch")
        ).getroot()
        args = {
            item.attrib["name"]: item.attrib.get(
                "default",
                item.attrib.get("value"),
            )
            for item in root.findall("arg")
        }
        self.assertTrue(args["weight_path"].endswith("yolov5n.pt"))
        self.assertEqual(args["device"], "0")
        self.assertEqual(args["half"], "true")
        self.assertEqual(args["input_width"], "640")
        self.assertEqual(args["input_height"], "384")
        self.assertEqual(args["publish_debug_image"], "false")

    def test_node_is_headless_and_avoids_pandas_autoshape(self):
        package_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..")
        )
        with open(
            os.path.join(package_root, "node", "yolov5_node.py"),
            "r",
        ) as handle:
            source = handle.read()
        self.assertIn("DetectMultiBackend", source)
        self.assertNotIn(".pandas()", source)
        self.assertNotIn("cv2.imshow", source)
        self.assertNotIn("torch.hub.load", source)

    def test_tensorrt_builder_is_fixed_shape_fp16(self):
        package_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..")
        )
        with open(
            os.path.join(
                package_root,
                "tools",
                "build_yolov5n_tensorrt.sh",
            ),
            "r",
        ) as handle:
            source = handle.read()
        self.assertIn("--imgsz 384 640", source)
        self.assertIn("--opset 13", source)
        self.assertIn("--fp16", source)
        self.assertIn("--workspace=512", source)
        self.assertIn("prune_yolov5_onnx_outputs.py", source)

    def test_tensorrt_path_uses_lightweight_runner_without_pytorch(self):
        package_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..")
        )
        with open(
            os.path.join(package_root, "node", "yolov5_node.py"),
            "r",
        ) as handle:
            source = handle.read()
        self.assertIn("TensorRtRunner", source)
        self.assertNotIn("\nimport torch\n", source)


class LetterboxGeometryTest(unittest.TestCase):
    def test_720p_box_round_trips_through_640x384_letterbox(self):
        geometry = compute_letterbox_geometry(
            source_width=1280,
            source_height=720,
            target_width=640,
            target_height=384,
        )
        self.assertAlmostEqual(geometry["scale"], 0.5)
        self.assertEqual(geometry["pad_x"], 0)
        self.assertEqual(geometry["pad_y"], 12)

        letterboxed = np.array([[100.0, 62.0, 300.0, 162.0]])
        restored = scale_boxes_to_original(
            letterboxed,
            geometry,
            source_width=1280,
            source_height=720,
        )
        np.testing.assert_allclose(
            restored,
            [[200.0, 100.0, 600.0, 300.0]],
            atol=1e-6,
        )

    def test_boxes_are_clipped_to_original_image(self):
        geometry = compute_letterbox_geometry(640, 480, 640, 384)
        restored = scale_boxes_to_original(
            np.array([[-20.0, -20.0, 700.0, 500.0]]),
            geometry,
            source_width=640,
            source_height=480,
        )
        np.testing.assert_allclose(restored, [[0.0, 0.0, 640.0, 480.0]])


class NumpyNmsTest(unittest.TestCase):
    def test_class_aware_nms_keeps_best_overlap_and_other_class(self):
        prediction = np.array(
            [
                [100.0, 100.0, 40.0, 40.0, 0.90, 0.90, 0.10],
                [102.0, 102.0, 40.0, 40.0, 0.80, 0.85, 0.15],
                [100.0, 100.0, 40.0, 40.0, 0.95, 0.10, 0.90],
            ],
            dtype=np.float32,
        )
        detections = numpy_non_max_suppression(
            prediction,
            conf_threshold=0.25,
            iou_threshold=0.45,
            max_det=10,
        )
        self.assertEqual(detections.shape, (2, 6))
        self.assertEqual(set(detections[:, 5].astype(int)), {0, 1})
        class_zero = detections[detections[:, 5] == 0][0]
        self.assertAlmostEqual(class_zero[4], 0.81, places=5)
        np.testing.assert_allclose(
            class_zero[:4],
            [80.0, 80.0, 120.0, 120.0],
            atol=1e-6,
        )

    def test_empty_prediction_returns_standard_shape(self):
        prediction = np.zeros((5, 7), dtype=np.float32)
        detections = numpy_non_max_suppression(
            prediction,
            conf_threshold=0.25,
            iou_threshold=0.45,
            max_det=10,
        )
        self.assertEqual(detections.shape, (0, 6))


if __name__ == "__main__":
    unittest.main()
