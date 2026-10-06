#!/usr/bin/env python3
"""No-motion smoke probe for the RGB-D localization adapter."""
import argparse
import math
import os
import sys
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

from vl_locate.msg import Detection, DetectionArray

SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from rgbd_geometry import estimate_target_point  # noqa: E402
from rgbd_target_localizer import RgbdTargetLocalizer  # noqa: E402


class SmokeProbe:
    def __init__(self, mode, synthetic_depth):
        self.mode = mode
        self.synthetic_depth = synthetic_depth
        self.result = None
        self.result_event = threading.Event()
        self.target_sub = rospy.Subscriber(
            "/target_point", PointStamped, self.target_callback, queue_size=1
        )
        self.yolo_pub = rospy.Publisher(
            "/yolov5/BoundingBoxes", BoundingBoxes, queue_size=1
        )
        self.request_pub = rospy.Publisher(
            "/vl_locate/frame_request", Header, queue_size=1
        )
        self.vlm_pub = rospy.Publisher("/Pose", DetectionArray, queue_size=1)
        self.cloud_pub = rospy.Publisher(
            "/vl_locate/test_points", PointCloud2, queue_size=1
        )
        self.zoom_camera_info = None

    def target_callback(self, message):
        self.result = message
        self.result_event.set()

    def bbox_candidates(self, width, height):
        if self.mode == "yolo":
            return [(
                int(width * 0.15),
                int(height * 0.15),
                int(width * 0.85),
                int(height * 0.85),
            )]
        box_width = int(width * 0.25)
        box_height = int(height * 0.30)
        candidates = []
        for center_y in (0.35, 0.55, 0.72):
            for center_x in (0.25, 0.50, 0.75):
                x_center = int(width * center_x)
                y_center = int(height * center_y)
                candidates.append((
                    max(0, x_center - box_width // 2),
                    max(0, y_center - box_height // 2),
                    min(width, x_center + box_width // 2),
                    min(height, y_center + box_height // 2),
                ))
        return candidates

    def supported_vlm_bbox(self):
        camera_info = rospy.wait_for_message(
            "/new_topic/camera_info", CameraInfo, timeout=5.0
        )
        cloud = rospy.wait_for_message(
            "/camera/depth_registered/points", PointCloud2, timeout=5.0
        )
        points = RgbdTargetLocalizer.point_cloud_xyz(cloud)
        width, height = camera_info.width, camera_info.height
        candidates = [(0, 0, width, height)]
        for width_scale, height_scale in ((0.16, 0.18), (0.25, 0.30), (0.36, 0.40)):
            box_width = int(width * width_scale)
            box_height = int(height * height_scale)
            for center_y in (0.28, 0.45, 0.62, 0.78):
                for center_x in (0.18, 0.38, 0.58, 0.78):
                    x_center = int(width * center_x)
                    y_center = int(height * center_y)
                    candidates.append((
                        max(0, x_center - box_width // 2),
                        max(0, y_center - box_height // 2),
                        min(width, x_center + box_width // 2),
                        min(height, y_center + box_height // 2),
                    ))
        for bbox in candidates:
            point = estimate_target_point(
                points_xyz=points,
                camera_matrix=camera_info.K,
                bbox=bbox,
                min_depth=0.15,
                max_depth=3.0,
                central_roi_scale=0.6,
                min_points=30,
                depth_bin_size=0.02,
                min_cluster_fraction=0.10,
            )
            if point is not None:
                print("supported_vlm_bbox={} expected_z={:.4f}".format(bbox, point[2]))
                return bbox
        finite = np.isfinite(points).all(axis=1)
        depth_valid = (points[:, 2] >= 0.15) & (points[:, 2] <= 3.0)
        valid = points[finite & depth_valid]
        u = camera_info.K[0] * valid[:, 0] / valid[:, 2] + camera_info.K[2]
        v = camera_info.K[4] * valid[:, 1] / valid[:, 2] + camera_info.K[5]
        visible = valid[
            (u >= 0)
            & (u < width)
            & (v >= 0)
            & (v < height)
        ]
        max_bin = 0
        if len(visible):
            _, counts = np.unique(
                np.floor(visible[:, 2] / 0.02).astype(np.int64),
                return_counts=True,
            )
            max_bin = int(np.max(counts))
        print(
            "vlm_projection valid={} visible={} max_full_image_bin={}".format(
                len(valid), len(visible), max_bin
            )
        )
        raise RuntimeError("current zoom image has no bbox with a supported depth cluster")

    def publish_yolo(self, image, bbox):
        x_min, y_min, x_max, y_max = bbox
        box = BoundingBox(
            probability=0.99,
            xmin=x_min,
            ymin=y_min,
            xmax=x_max,
            ymax=y_max,
            num=0,
            Class="smoke_probe",
        )
        message = BoundingBoxes()
        message.header = image.header
        message.image_header = image.header
        message.bounding_boxes = [box]
        self.yolo_pub.publish(message)

    def publish_vlm(self, image, bbox):
        x_min, y_min, x_max, y_max = bbox
        if self.synthetic_depth:
            camera_info = self.zoom_camera_info
            center_u = (x_min + x_max) * 0.5
            center_v = (y_min + y_max) * 0.5
            points = []
            for du in range(-20, 21, 4):
                for dv in range(-20, 21, 4):
                    z = 0.60
                    x = (center_u + du - camera_info.K[2]) * z / camera_info.K[0]
                    y = (center_v + dv - camera_info.K[5]) * z / camera_info.K[4]
                    points.append((x, y, z))
            cloud_header = Header(
                stamp=image.header.stamp,
                frame_id="camera_rgb_optical_frame",
            )
            self.cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(cloud_header, points)
            )
            rospy.sleep(0.10)
        self.request_pub.publish(image.header)
        rospy.sleep(0.20)
        detection = Detection(
            class_name="smoke_probe",
            x_center=(x_min + x_max) // 2,
            y_center=(y_min + y_max) // 2,
            probability=0.99,
            xmin=x_min,
            ymin=y_min,
            xmax=x_max,
            ymax=y_max,
        )
        message = DetectionArray()
        message.header = image.header
        message.detections = [detection]
        self.vlm_pub.publish(message)

    def run(self):
        image_topic = (
            "/camera/rgb/image_raw" if self.mode == "yolo" else "/new_topic"
        )
        rospy.sleep(0.75)
        if self.mode == "vlm" and self.synthetic_depth:
            self.zoom_camera_info = rospy.wait_for_message(
                "/new_topic/camera_info", CameraInfo, timeout=5.0
            )
            candidates = [(160, 120, 480, 360)]
        else:
            candidates = [self.supported_vlm_bbox()] if self.mode == "vlm" else None
        for attempt in range(12):
            image = rospy.wait_for_message(image_topic, Image, timeout=5.0)
            if candidates is None:
                candidates = self.bbox_candidates(image.width, image.height)
            bbox = candidates[attempt % len(candidates)]
            if self.mode == "yolo":
                self.publish_yolo(image, bbox)
            else:
                self.publish_vlm(image, bbox)
            if self.result_event.wait(0.75):
                break

        if self.result is None:
            raise RuntimeError("no /target_point received")
        point = self.result.point
        values = (point.x, point.y, point.z)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("target point contains a non-finite value")
        if self.result.header.frame_id != "camera_rgb_optical_frame":
            raise RuntimeError("unexpected output frame")
        if self.result.header.stamp.to_sec() <= 0.0:
            raise RuntimeError("target point has no source timestamp")
        print(
            "mode={} frame={} stamp={:.9f} xyz=({:.4f},{:.4f},{:.4f})".format(
                self.mode,
                self.result.header.frame_id,
                self.result.header.stamp.to_sec(),
                point.x,
                point.y,
                point.z,
            )
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("yolo", "vlm"), required=True)
    parser.add_argument("--synthetic-depth", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("live_rgbd_smoke_{}".format(args.mode), anonymous=True)
    SmokeProbe(args.mode, args.synthetic_depth).run()


if __name__ == "__main__":
    main()
