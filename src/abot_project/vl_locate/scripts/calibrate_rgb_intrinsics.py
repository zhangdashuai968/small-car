#!/usr/bin/env python3
import io
import json
import os
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Image

from calibration_tools import (
    build_checkerboard_points,
    calibrate_pinhole,
    calibration_gate,
    is_diverse_view,
    view_feature,
)
from rgbd_geometry import image_array_from_bytes


class HeadlessRgbCalibrator:
    def __init__(self):
        self.image_topic = str(
            rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        )
        self.expected_width = int(rospy.get_param("~expected_width", 1280))
        self.expected_height = int(rospy.get_param("~expected_height", 720))
        self.board_columns = int(rospy.get_param("~board_columns", 8))
        self.board_rows = int(rospy.get_param("~board_rows", 6))
        self.square_size = float(rospy.get_param("~square_size", 0.03))
        self.required_views = int(rospy.get_param("~required_views", 25))
        self.min_feature_distance = float(
            rospy.get_param("~min_feature_distance", 0.06)
        )
        self.min_capture_interval = float(
            rospy.get_param("~min_capture_interval", 0.35)
        )
        self.capture_timeout = float(
            rospy.get_param("~capture_timeout", 180.0)
        )
        self.max_rms = float(rospy.get_param("~max_rms", 0.5))
        self.max_view_p95 = float(
            rospy.get_param("~max_view_p95", 1.0)
        )
        self.min_centroid_span = float(
            rospy.get_param("~min_centroid_span", 0.25)
        )
        self.min_scale_span = float(
            rospy.get_param("~min_scale_span", 0.05)
        )
        self.camera_name = str(rospy.get_param("~camera_name", "camera"))
        self.output_yaml = os.path.abspath(
            str(
                rospy.get_param(
                    "~output_yaml",
                    "/tmp/gemini_rgb_1280x720_calibrated.yaml",
                )
            )
        )
        self.output_report = os.path.abspath(
            str(
                rospy.get_param(
                    "~output_report",
                    "/tmp/gemini_rgb_1280x720_calibration_report.json",
                )
            )
        )

        if self.required_views < 3:
            raise ValueError("~required_views must be at least 3")
        self.board_points = build_checkerboard_points(
            self.board_columns,
            self.board_rows,
            self.square_size,
        )
        self.pattern_size = (self.board_columns, self.board_rows)
        self.image_points = []
        self.features = []
        self.accepted_stamps = []
        self.last_detection_time = 0.0
        self.started_at = time.time()
        self.lock = threading.RLock()
        self.finished = False
        self.processing = False

        self.subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=8 * 1024 * 1024,
        )
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        rospy.loginfo(
            "Headless RGB calibration: topic=%s board=%dx%d square=%.6fm views=%d output=%s",
            self.image_topic,
            self.board_columns,
            self.board_rows,
            self.square_size,
            self.required_views,
            self.output_yaml,
        )

    def image_callback(self, message):
        with self.lock:
            if self.finished or self.processing:
                return
            now_sec = time.time()
            if now_sec - self.last_detection_time < self.min_capture_interval:
                return
            self.last_detection_time = now_sec
            self.processing = True
        try:
            self._process_image(message)
        except (TypeError, ValueError, cv2.error) as error:
            rospy.logwarn_throttle(
                5.0,
                "Calibration frame rejected: %s",
                error,
            )
        finally:
            with self.lock:
                self.processing = False

    def _process_image(self, message):
        if (
            message.width != self.expected_width
            or message.height != self.expected_height
        ):
            raise ValueError(
                "active RGB mode is %dx%d, expected %dx%d"
                % (
                    message.width,
                    message.height,
                    self.expected_width,
                    self.expected_height,
                )
            )
        image = image_array_from_bytes(
            message.data,
            message.width,
            message.height,
            message.step,
            message.encoding,
        )
        conversion = (
            cv2.COLOR_RGB2GRAY
            if message.encoding == "rgb8"
            else cv2.COLOR_BGR2GRAY
        )
        gray = cv2.cvtColor(image, conversion)
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(
            gray,
            self.pattern_size,
            flags,
        )
        if not found:
            rospy.loginfo_throttle(5.0, "Checkerboard not detected")
            return
        criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            40,
            0.001,
        )
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (5, 5),
            (-1, -1),
            criteria,
        )
        feature = view_feature(
            corners,
            (message.width, message.height),
        )
        with self.lock:
            if not is_diverse_view(
                feature,
                self.features,
                self.min_feature_distance,
            ):
                rospy.loginfo_throttle(
                    2.0,
                    "Checkerboard view is too similar to accepted samples",
                )
                return
            self.image_points.append(corners.copy())
            self.features.append(feature)
            self.accepted_stamps.append(message.header.stamp.to_sec())
            count = len(self.image_points)
        rospy.loginfo(
            "Accepted calibration view %d/%d feature=%s",
            count,
            self.required_views,
            [round(value, 4) for value in feature],
        )
        if count >= self.required_views:
            self.finish_calibration()

    def timer_callback(self, _event):
        with self.lock:
            timed_out = (
                not self.finished
                and time.time() - self.started_at >= self.capture_timeout
            )
        if timed_out:
            self.finish_incomplete("capture timeout")

    def finish_calibration(self):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            image_points = list(self.image_points)
            features = list(self.features)
            stamps = list(self.accepted_stamps)
        try:
            result = calibrate_pinhole(
                image_points,
                (self.expected_width, self.expected_height),
                self.board_points,
            )
            gate = calibration_gate(
                result,
                features,
                required_views=self.required_views,
                max_rms=self.max_rms,
                max_view_p95=self.max_view_p95,
                min_centroid_span=self.min_centroid_span,
                min_scale_span=self.min_scale_span,
            )
            report = self._report(result, gate, features, stamps)
            self.write_json(self.output_report, report)
            if gate["passed"]:
                self.write_camera_yaml(result)
                rospy.loginfo(
                    "Calibration PASSED: rms=%.6f p95=%.6f yaml=%s report=%s",
                    gate["rms"],
                    gate["view_error_p95"],
                    self.output_yaml,
                    self.output_report,
                )
            else:
                rospy.logerr(
                    "Calibration FAILED; YAML not written: %s",
                    "; ".join(gate["reasons"]),
                )
        except (TypeError, ValueError, cv2.error) as error:
            self.write_json(
                self.output_report,
                {
                    "passed": False,
                    "reasons": ["calibration exception: %s" % error],
                    "view_count": len(image_points),
                },
            )
            rospy.logerr("Calibration failed: %s", error)
        rospy.signal_shutdown("calibration finished")

    def finish_incomplete(self, reason):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            view_count = len(self.image_points)
            features = list(self.features)
        report = {
            "passed": False,
            "reasons": [reason],
            "view_count": view_count,
            "required_views": self.required_views,
            "features": features,
        }
        self.write_json(self.output_report, report)
        rospy.logerr(
            "Calibration incomplete: %s (%d/%d views); YAML not written",
            reason,
            view_count,
            self.required_views,
        )
        rospy.signal_shutdown("calibration incomplete")

    def _report(self, result, gate, features, stamps):
        camera_matrix = np.asarray(result["camera_matrix"])
        distortion = np.asarray(result["distortion"])
        return {
            "passed": bool(gate["passed"]),
            "reasons": list(gate["reasons"]),
            "camera_name": self.camera_name,
            "image_width": self.expected_width,
            "image_height": self.expected_height,
            "board_inner_corners": [
                self.board_columns,
                self.board_rows,
            ],
            "square_size_m": self.square_size,
            "camera_matrix": camera_matrix.tolist(),
            "distortion": distortion.tolist(),
            "view_errors_px": list(result["view_errors"]),
            "features": features,
            "accepted_stamps": stamps,
            "gate": gate,
        }

    def write_camera_yaml(self, result):
        matrix = np.asarray(result["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(
            result["distortion"],
            dtype=np.float64,
        ).reshape((-1,))
        projection = [
            float(matrix[0, 0]),
            0.0,
            float(matrix[0, 2]),
            0.0,
            0.0,
            float(matrix[1, 1]),
            float(matrix[1, 2]),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        payload = {
            "image_width": self.expected_width,
            "image_height": self.expected_height,
            "camera_name": self.camera_name,
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": matrix.reshape((-1,)).tolist(),
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(distortion.size),
                "data": distortion.tolist(),
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": projection,
            },
        }
        self.ensure_parent(self.output_yaml)
        with io.open(self.output_yaml, "w", encoding="utf-8") as handle:
            yaml.safe_dump(
                payload,
                handle,
                default_flow_style=False,
                allow_unicode=True,
            )

    @staticmethod
    def ensure_parent(path):
        parent = os.path.dirname(path)
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)

    @classmethod
    def write_json(cls, path, payload):
        cls.ensure_parent(path)
        with io.open(path, "w", encoding="utf-8") as handle:
            handle.write(
                json.dumps(
                    payload,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
            )
            handle.write("\n")


def main():
    rospy.init_node("calibrate_rgb_intrinsics")
    HeadlessRgbCalibrator()
    rospy.spin()


if __name__ == "__main__":
    main()
