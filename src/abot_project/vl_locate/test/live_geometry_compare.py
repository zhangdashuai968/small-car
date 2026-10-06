#!/usr/bin/env python3
"""No-motion live comparison of point-cloud and depth-ROI localization."""
import argparse
import json
import math
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes


class GeometryComparator:
    def __init__(self, sample_count, timeout):
        self.sample_count = int(sample_count)
        self.deadline = time.time() + float(timeout)
        self.lock = threading.RLock()
        self.depth_points = {}
        self.cloud_points = {}
        self.pairs = []
        self.publisher = rospy.Publisher(
            "/yolov5/BoundingBoxes",
            BoundingBoxes,
            queue_size=1,
        )
        self.image_subscriber = rospy.Subscriber(
            "/camera/rgb/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
        )
        self.depth_subscriber = rospy.Subscriber(
            "/target_point",
            PointStamped,
            self.depth_callback,
            queue_size=10,
        )
        self.cloud_subscriber = rospy.Subscriber(
            "/target_point_cloud",
            PointStamped,
            self.cloud_callback,
            queue_size=10,
        )
        self.last_publish = 0.0

    @staticmethod
    def point_array(message):
        return np.asarray(
            [message.point.x, message.point.y, message.point.z],
            dtype=np.float64,
        )

    def image_callback(self, image):
        now = time.time()
        if now - self.last_publish < 0.50:
            return
        self.last_publish = now
        box = BoundingBox(
            probability=0.99,
            xmin=int(image.width * 0.15),
            ymin=int(image.height * 0.15),
            xmax=int(image.width * 0.85),
            ymax=int(image.height * 0.85),
            num=1,
            Class="geometry_compare",
        )
        detections = BoundingBoxes()
        detections.header = image.header
        detections.image_header = image.header
        detections.bounding_boxes = [box]
        self.publisher.publish(detections)

    def depth_callback(self, message):
        self.store(message, self.depth_points, self.cloud_points)

    def cloud_callback(self, message):
        self.store(message, self.cloud_points, self.depth_points)

    def store(self, message, own, other):
        if message.header.frame_id != "camera_rgb_optical_frame":
            return
        point = self.point_array(message)
        if not np.isfinite(point).all():
            return
        stamp = message.header.stamp.to_nsec()
        with self.lock:
            own[stamp] = point
            if stamp in other:
                depth = (
                    point
                    if own is self.depth_points
                    else other[stamp]
                )
                cloud = (
                    point
                    if own is self.cloud_points
                    else other[stamp]
                )
                self.pairs.append((depth, cloud))
                self.depth_points.pop(stamp, None)
                self.cloud_points.pop(stamp, None)
            if len(own) > 32:
                own.pop(sorted(own)[0], None)

    def run(self):
        rate = rospy.Rate(20)
        while (
            not rospy.is_shutdown()
            and len(self.pairs) < self.sample_count
            and time.time() < self.deadline
        ):
            rate.sleep()
        if len(self.pairs) < self.sample_count:
            raise RuntimeError(
                "only %d/%d paired outputs arrived"
                % (len(self.pairs), self.sample_count)
            )
        depth = np.asarray(
            [pair[0] for pair in self.pairs[: self.sample_count]]
        )
        cloud = np.asarray(
            [pair[1] for pair in self.pairs[: self.sample_count]]
        )
        absolute_mm = np.abs(depth - cloud) * 1000.0
        distance_mm = np.linalg.norm(depth - cloud, axis=1) * 1000.0
        result = {
            "sample_count": self.sample_count,
            "finite": bool(
                np.isfinite(depth).all() and np.isfinite(cloud).all()
            ),
            "median_xyz_abs_mm": np.median(absolute_mm, axis=0).tolist(),
            "p95_xyz_abs_mm": np.percentile(
                absolute_mm,
                95,
                axis=0,
            ).tolist(),
            "median_3d_mm": float(np.median(distance_mm)),
            "p95_3d_mm": float(np.percentile(distance_mm, 95)),
            "max_3d_mm": float(np.max(distance_mm)),
        }
        if not all(
            math.isfinite(value)
            for value in (
                result["median_3d_mm"],
                result["p95_3d_mm"],
                result["max_3d_mm"],
            )
        ):
            raise RuntimeError("comparison produced non-finite statistics")
        print(json.dumps(result, sort_keys=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=45.0)
    arguments = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("live_geometry_compare", anonymous=True)
    GeometryComparator(arguments.samples, arguments.timeout).run()


if __name__ == "__main__":
    main()
