#!/usr/bin/env python3
import io
import json
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped
from yolov5_ros_msgs.msg import BoundingBoxes

from calibration_tools import accuracy_summary
from vl_locate.msg import DetectionArray


class RgbdAccuracyGate:
    def __init__(self):
        self.source_mode = str(
            rospy.get_param("~source_mode", "yolo")
        ).lower()
        if self.source_mode not in ("yolo", "vlm"):
            raise ValueError("~source_mode must be yolo or vlm")
        self.target_class = str(rospy.get_param("~target_class", ""))
        self.expected_xyz = [
            float(rospy.get_param("~expected_x")),
            float(rospy.get_param("~expected_y")),
            float(rospy.get_param("~expected_z")),
        ]
        self.required_attempts = int(
            rospy.get_param("~required_attempts", 20)
        )
        self.min_valid_rate = float(
            rospy.get_param("~min_valid_rate", 0.90)
        )
        self.max_median_error = float(
            rospy.get_param("~max_median_error", 0.020)
        )
        self.max_p95_error = float(
            rospy.get_param("~max_p95_error", 0.040)
        )
        self.capture_timeout = float(
            rospy.get_param("~capture_timeout", 120.0)
        )
        self.output_frame = str(
            rospy.get_param(
                "~output_frame",
                "camera_rgb_optical_frame",
            )
        )
        self.output_path = os.path.abspath(
            str(
                rospy.get_param(
                    "~output_path",
                    "/tmp/rgbd_accuracy_gate.json",
                )
            )
        )
        self.settle_time = float(rospy.get_param("~settle_time", 1.0))
        self.started_at = time.time()
        self.finish_after = None
        self.attempt_keys = set()
        self.point_keys = set()
        self.points = []
        self.lock = threading.RLock()
        self.finished = False

        rospy.Subscriber(
            rospy.get_param("~output_topic", "/target_point"),
            PointStamped,
            self.point_callback,
            queue_size=20,
        )
        if self.source_mode == "yolo":
            rospy.Subscriber(
                rospy.get_param(
                    "~yolo_detection_topic",
                    "/yolov5/BoundingBoxes",
                ),
                BoundingBoxes,
                self.yolo_callback,
                queue_size=20,
            )
        else:
            rospy.Subscriber(
                rospy.get_param("~vlm_detection_topic", "/Pose"),
                DetectionArray,
                self.vlm_callback,
                queue_size=20,
            )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo(
            "RGB-D accuracy gate ready: source=%s expected=(%.4f, %.4f, %.4f) attempts=%d output=%s",
            self.source_mode,
            self.expected_xyz[0],
            self.expected_xyz[1],
            self.expected_xyz[2],
            self.required_attempts,
            self.output_path,
        )

    def yolo_callback(self, message):
        boxes = list(message.bounding_boxes)
        if self.target_class:
            boxes = [
                box for box in boxes if box.Class == self.target_class
            ]
        if not boxes:
            return
        stamp = message.image_header.stamp
        if stamp.to_nsec() == 0:
            stamp = message.header.stamp
        self.record_attempt(stamp.to_nsec())

    def vlm_callback(self, message):
        if not message.detections:
            return
        self.record_attempt(message.header.stamp.to_nsec())

    def record_attempt(self, stamp_key):
        with self.lock:
            if self.finished or stamp_key in self.attempt_keys:
                return
            self.attempt_keys.add(stamp_key)
            count = len(self.attempt_keys)
            if count >= self.required_attempts and self.finish_after is None:
                self.finish_after = time.time() + self.settle_time
        rospy.loginfo(
            "Accuracy attempt %d/%d",
            count,
            self.required_attempts,
        )

    def point_callback(self, message):
        if message.header.frame_id != self.output_frame:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring target_point frame %s; expected %s",
                message.header.frame_id,
                self.output_frame,
            )
            return
        values = [
            float(message.point.x),
            float(message.point.y),
            float(message.point.z),
        ]
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(5.0, "Ignoring non-finite target_point")
            return
        key = message.header.stamp.to_nsec()
        with self.lock:
            if (
                self.finished
                or not self.attempt_keys
                or key in self.point_keys
            ):
                return
            self.point_keys.add(key)
            self.points.append(values)

    def timer_callback(self, _event):
        with self.lock:
            should_finish = (
                not self.finished
                and (
                    (
                        self.finish_after is not None
                        and time.time() >= self.finish_after
                    )
                    or time.time() - self.started_at >= self.capture_timeout
                )
            )
        if should_finish:
            self.finish()

    def finish(self):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            attempts = len(self.attempt_keys)
            points = list(self.points)
        report = accuracy_summary(
            points,
            self.expected_xyz,
            attempts=attempts,
            required_attempts=self.required_attempts,
            min_valid_rate=self.min_valid_rate,
            max_median_error=self.max_median_error,
            max_p95_error=self.max_p95_error,
        )
        report.update(
            {
                "source_mode": self.source_mode,
                "target_class": self.target_class,
                "output_frame": self.output_frame,
                "raw_points_xyz_m": points,
                "capture_duration_sec": time.time() - self.started_at,
            }
        )
        parent = os.path.dirname(self.output_path)
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        with io.open(self.output_path, "w", encoding="utf-8") as handle:
            handle.write(
                json.dumps(
                    report,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
            )
            handle.write("\n")
        if report["passed"]:
            rospy.loginfo(
                "Accuracy gate PASSED: valid=%.3f median=%.4fm p95=%.4fm report=%s",
                report["valid_rate"],
                report["median_error_m"],
                report["p95_error_m"],
                self.output_path,
            )
        else:
            rospy.logerr(
                "Accuracy gate FAILED: %s report=%s",
                "; ".join(report["reasons"]),
                self.output_path,
            )
        rospy.signal_shutdown("accuracy gate finished")


def main():
    rospy.init_node("rgbd_accuracy_gate")
    RgbdAccuracyGate()
    rospy.spin()


if __name__ == "__main__":
    main()
