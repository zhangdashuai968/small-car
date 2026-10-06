#!/usr/bin/env python3
import io
import os
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SOURCE_ROOT = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", ".."))


def launch_args(path):
    root = ET.parse(path).getroot()
    return {
        element.attrib["name"]: element.attrib.get("value", element.attrib.get("default"))
        for element in root.findall("arg")
        if "name" in element.attrib
    }


class HighResolutionModeContractTest(unittest.TestCase):
    def test_720p_calibration_is_valid_and_mode_specific(self):
        path = os.path.join(
            PACKAGE_ROOT,
            "config",
            "gemini_rgb_1280x720.yaml",
        )
        with io.open(path, "r", encoding="utf-8") as handle:
            calibration = yaml.safe_load(handle)

        self.assertEqual(calibration["image_width"], 1280)
        self.assertEqual(calibration["image_height"], 720)
        matrix = calibration["camera_matrix"]["data"]
        self.assertGreater(matrix[0], 0.0)
        self.assertGreater(matrix[4], 0.0)
        self.assertGreaterEqual(matrix[2], 0.0)
        self.assertLess(matrix[2], 1280.0)
        self.assertGreaterEqual(matrix[5], 0.0)
        self.assertLess(matrix[5], 720.0)

    def test_720p_launch_disables_same_pixel_registered_chain(self):
        path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "gemini_720p_rgbd.launch",
        )
        args = launch_args(path)

        self.assertEqual(args["rgb_width"], "1280")
        self.assertEqual(args["rgb_height"], "720")
        self.assertEqual(args["rgb_frame_rate"], "30")
        self.assertEqual(args["depth_registration"], "false")
        self.assertEqual(args["depth_registered_processing"], "false")
        self.assertEqual(args["geometry_input"], "depth_image")
        self.assertEqual(
            args["depth_image_topic"],
            "/camera/depth/image_rect_raw",
        )

    def test_640p_optimized_launch_uses_raw_depth_without_point_cloud(self):
        path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "gemini_640p_optimized_rgbd.launch",
        )
        args = launch_args(path)
        self.assertEqual(args["rgb_width"], "640")
        self.assertEqual(args["rgb_height"], "480")
        self.assertEqual(args["rgb_frame_rate"], "30")
        self.assertEqual(args["depth_registration"], "false")
        self.assertEqual(args["geometry_input"], "depth_image")
        self.assertEqual(
            args["depth_image_topic"],
            "/camera/depth/image_rect_raw",
        )

    def test_canonical_gemini_launch_exposes_mode_arguments(self):
        path = os.path.join(
            SOURCE_ROOT,
            "depth_camera",
            "ros_astra_launch",
            "launch",
            "gemini.launch",
        )
        args = launch_args(path)
        self.assertEqual(args["rgb_width"], "640")
        self.assertEqual(args["rgb_height"], "480")
        self.assertEqual(args["rgb_frame_rate"], "30")
        self.assertEqual(args["rgb_video_mode"], "mjpeg")

    def test_uvc_driver_preserves_matching_calibration_manager_data(self):
        path = os.path.join(
            SOURCE_ROOT,
            "depth_camera",
            "ros_astra_camera",
            "src",
            "libuvc_camera",
            "camera_driver.cpp",
        )
        with io.open(path, "r", encoding="utf-8") as handle:
            source = handle.read()
        self.assertIn("mode_calibration_matches", source)
        self.assertIn("cinfo_manager_.isCalibrated()", source)

    def test_720p_headless_calibration_contract_is_fail_closed(self):
        path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "calibrate_720p_intrinsics.launch",
        )
        args = launch_args(path)
        self.assertEqual(args["board_columns"], "8")
        self.assertEqual(args["board_rows"], "6")
        self.assertEqual(args["required_views"], "25")
        self.assertTrue(args["output_yaml"].startswith("/tmp/"))
        self.assertTrue(args["output_report"].startswith("/tmp/"))

    def test_accuracy_gate_requires_twenty_attempts(self):
        path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "rgbd_accuracy_gate.launch",
        )
        args = launch_args(path)
        self.assertEqual(args["required_attempts"], "20")
        self.assertEqual(args["capture_timeout"], "120.0")
        self.assertTrue(args["output_path"].startswith("/tmp/"))

    def test_calibration_tools_are_packaged_and_tested(self):
        cmake_path = os.path.join(PACKAGE_ROOT, "CMakeLists.txt")
        with io.open(cmake_path, "r", encoding="utf-8") as handle:
            cmake = handle.read()
        for script in (
            "scripts/calibration_tools.py",
            "scripts/calibrate_rgb_intrinsics.py",
            "scripts/checkerboard_bbox_publisher.py",
            "scripts/rgbd_accuracy_gate.py",
        ):
            self.assertIn(script, cmake)
        self.assertIn(
            "catkin_add_nosetests(test/test_calibration_tools.py)",
            cmake,
        )

    def test_calibration_runtime_dependencies_are_declared(self):
        package_path = os.path.join(PACKAGE_ROOT, "package.xml")
        root = ET.parse(package_path).getroot()
        exec_dependencies = {
            element.text for element in root.findall("exec_depend")
        }
        self.assertIn("python3-opencv", exec_dependencies)
        self.assertIn("python3-yaml", exec_dependencies)

    def test_checkerboard_accuracy_launch_is_single_source(self):
        path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "checkerboard_720p_accuracy.launch",
        )
        root = ET.parse(path).getroot()
        args = launch_args(path)
        self.assertEqual(args["target_class"], "checkerboard")
        self.assertEqual(args["required_attempts"], "20")
        includes = [
            element.attrib["file"] for element in root.findall("include")
        ]
        self.assertTrue(
            any("gemini_720p_rgbd.launch" in item for item in includes)
        )
        self.assertTrue(
            any("rgbd_accuracy_gate.launch" in item for item in includes)
        )


if __name__ == "__main__":
    unittest.main()
