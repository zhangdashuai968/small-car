#!/usr/bin/env python3
# _*_ coding:utf-8 _*_
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from yolov5_ros_msgs.msg import BoundingBoxes

from rgbd_geometry import (
    TimedFrameCache,
    depth_array_from_bytes,
    depth_points_in_rgb_bbox,
    estimate_target_point,
    select_detection,
    transform_points,
    validate_camera_matrix,
)
from vl_locate.msg import DetectionArray


class RgbdTargetLocalizer:
    def __init__(self):
        self.source_mode = str(rospy.get_param("~source_mode", "yolo")).lower()
        if self.source_mode not in ("yolo", "vlm"):
            raise ValueError("~source_mode must be 'yolo' or 'vlm'")
        self.geometry_input = str(
            rospy.get_param("~geometry_input", "point_cloud")
        ).lower()
        if self.geometry_input not in ("point_cloud", "depth_image"):
            raise ValueError(
                "~geometry_input must be 'point_cloud' or 'depth_image'"
            )

        self.target_class = str(rospy.get_param("~target_class", ""))
        self.output_frame = str(
            rospy.get_param("~output_frame", "camera_rgb_optical_frame")
        )
        self.point_cloud_frame = str(
            rospy.get_param("~point_cloud_frame", self.output_frame)
        )
        self.depth_frame = str(
            rospy.get_param("~depth_frame", "camera_depth_optical_frame")
        )
        self.depth_to_rgb_rotation = np.asarray(
            rospy.get_param(
                "~depth_to_rgb_rotation",
                [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            ),
            dtype=np.float64,
        ).reshape((3, 3))
        self.depth_to_rgb_translation = np.asarray(
            rospy.get_param("~depth_to_rgb_translation", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        transform_points(
            np.empty((0, 3), dtype=np.float64),
            self.depth_to_rgb_rotation,
            self.depth_to_rgb_translation,
        )
        self.min_depth = float(rospy.get_param("~min_depth", 0.15))
        self.max_depth = float(rospy.get_param("~max_depth", 3.0))
        self.central_roi_scale = float(
            rospy.get_param("~central_roi_scale", 0.6)
        )
        self.min_points = int(rospy.get_param("~min_points", 30))
        self.depth_bin_size = float(rospy.get_param("~depth_bin_size", 0.02))
        self.min_cluster_fraction = float(
            rospy.get_param("~min_cluster_fraction", 0.10)
        )
        self.sync_tolerance = float(rospy.get_param("~sync_tolerance", 0.10))
        self.max_rate = float(rospy.get_param("~max_rate", 5.0))
        self.legacy_bbox_half_size = int(
            rospy.get_param("~legacy_bbox_half_size", 20)
        )
        self.pin_wait_sec = float(rospy.get_param("~pin_wait_sec", 0.5))
        self.roi_margin_px = int(rospy.get_param("~roi_margin_px", 3))
        if self.roi_margin_px < 0:
            raise ValueError("~roi_margin_px must be non-negative")

        history_size = int(rospy.get_param("~history_size", 12))
        pinned_size = int(rospy.get_param("~pinned_size", 4))
        pinned_ttl = float(rospy.get_param("~pinned_ttl", 60.0))
        self.cache = TimedFrameCache(history_size, pinned_size, pinned_ttl)
        self.cache_lock = threading.RLock()
        self.pending_pins = {}
        self.rgb_camera_info = None
        self.zoom_camera_info = None
        self.depth_camera_info = None
        self.last_process_time = 0.0

        self.point_pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/target_point"),
            PointStamped,
            queue_size=10,
        )
        self.cloud_sub = None
        self.depth_sub = None
        self.depth_info_sub = None
        if self.geometry_input == "point_cloud":
            self.cloud_sub = rospy.Subscriber(
                rospy.get_param(
                    "~point_cloud_topic", "/camera/depth_registered/points"
                ),
                PointCloud2,
                self.geometry_callback,
                queue_size=1,
                buff_size=16 * 1024 * 1024,
            )
        else:
            self.depth_sub = rospy.Subscriber(
                rospy.get_param(
                    "~depth_image_topic", "/camera/depth/image_rect_raw"
                ),
                Image,
                self.geometry_callback,
                queue_size=1,
                buff_size=4 * 1024 * 1024,
            )
            self.depth_info_sub = rospy.Subscriber(
                rospy.get_param(
                    "~depth_camera_info_topic", "/camera/depth/camera_info"
                ),
                CameraInfo,
                self.depth_camera_info_callback,
                queue_size=1,
            )
        self.rgb_info_sub = rospy.Subscriber(
            rospy.get_param(
                "~rgb_camera_info_topic", "/camera/rgb/camera_info"
            ),
            CameraInfo,
            self.rgb_camera_info_callback,
            queue_size=1,
        )
        self.zoom_info_sub = rospy.Subscriber(
            rospy.get_param(
                "~vlm_camera_info_topic", "/new_topic/camera_info"
            ),
            CameraInfo,
            self.zoom_camera_info_callback,
            queue_size=1,
        )
        self.frame_request_sub = rospy.Subscriber(
            rospy.get_param(
                "~frame_request_topic", "/vl_locate/frame_request"
            ),
            Header,
            self.frame_request_callback,
            queue_size=4,
        )
        self.vlm_sub = rospy.Subscriber(
            rospy.get_param("~vlm_detection_topic", "/Pose"),
            DetectionArray,
            self.vlm_detection_callback,
            queue_size=4,
        )
        self.yolo_sub = rospy.Subscriber(
            rospy.get_param(
                "~yolo_detection_topic", "/yolov5/BoundingBoxes"
            ),
            BoundingBoxes,
            self.yolo_detection_callback,
            queue_size=4,
        )
        rospy.loginfo(
            "rgbd_target_localizer ready: source=%s geometry=%s target_class=%s input_frame=%s output_frame=%s output=%s",
            self.source_mode,
            self.geometry_input,
            self.target_class or "<highest-confidence>",
            (
                self.point_cloud_frame
                if self.geometry_input == "point_cloud"
                else self.depth_frame
            ),
            self.output_frame,
            self.point_pub.resolved_name,
        )

    def rgb_camera_info_callback(self, message):
        self.rgb_camera_info = message

    def zoom_camera_info_callback(self, message):
        self.zoom_camera_info = message

    def depth_camera_info_callback(self, message):
        self.depth_camera_info = message

    def geometry_callback(self, message):
        stamp_sec = message.header.stamp.to_sec()
        now_sec = rospy.get_time()
        with self.cache_lock:
            self.cache.add(stamp_sec, message)
            for key, (requested_stamp, expires_at) in list(
                self.pending_pins.items()
            ):
                if now_sec > expires_at:
                    self.pending_pins.pop(key, None)
                    rospy.logwarn_throttle(
                        5.0,
                        "VLM frame pin expired before matching geometry arrived",
                    )
                    continue
                if self.cache.pin(
                    key,
                    requested_stamp,
                    self.sync_tolerance,
                    now_sec,
                ):
                    self.pending_pins.pop(key, None)

    def frame_request_callback(self, header):
        key = header.stamp.to_nsec()
        requested_stamp = header.stamp.to_sec()
        now_sec = rospy.get_time()
        with self.cache_lock:
            if self.cache.pin(
                key,
                requested_stamp,
                self.sync_tolerance,
                now_sec,
            ):
                return
            self.pending_pins[key] = (
                requested_stamp,
                now_sec + self.pin_wait_sec,
            )

    def vlm_detection_callback(self, message):
        if self.source_mode != "vlm":
            return
        detections = [
            {
                "class_name": detection.class_name,
                "probability": detection.probability,
                "bbox": self._vlm_bbox(detection),
            }
            for detection in message.detections
        ]
        selected = select_detection(detections, "")
        if selected is None:
            rospy.logwarn_throttle(5.0, "VLM detection message is empty")
            return
        key = message.header.stamp.to_nsec()
        with self.cache_lock:
            geometry_frame = self.cache.get_pinned(key, rospy.get_time())
        if geometry_frame is None:
            rospy.logwarn_throttle(
                5.0,
                "No pinned geometry frame for VLM stamp %d",
                key,
            )
            return
        self.localize(selected, geometry_frame, self.zoom_camera_info)

    def yolo_detection_callback(self, message):
        if self.source_mode != "yolo":
            return
        detections = [
            {
                "class_name": box.Class,
                "probability": box.probability,
                "bbox": (box.xmin, box.ymin, box.xmax, box.ymax),
            }
            for box in message.bounding_boxes
        ]
        selected = select_detection(detections, self.target_class)
        if selected is None:
            rospy.logwarn_throttle(
                5.0,
                "No YOLO detection matches target_class=%s",
                self.target_class,
            )
            return

        stamp = message.image_header.stamp
        if stamp.to_sec() == 0.0:
            stamp = message.header.stamp
        with self.cache_lock:
            geometry_frame = self.cache.nearest(
                stamp.to_sec(),
                self.sync_tolerance,
            )
        if geometry_frame is None:
            rospy.logwarn_throttle(
                5.0,
                "No geometry frame within %.3f s of YOLO stamp",
                self.sync_tolerance,
            )
            return
        self.localize(selected, geometry_frame, self.rgb_camera_info)

    def _vlm_bbox(self, detection):
        if detection.xmax > detection.xmin and detection.ymax > detection.ymin:
            return (
                detection.xmin,
                detection.ymin,
                detection.xmax,
                detection.ymax,
            )
        return (
            detection.x_center - self.legacy_bbox_half_size,
            detection.y_center - self.legacy_bbox_half_size,
            detection.x_center + self.legacy_bbox_half_size,
            detection.y_center + self.legacy_bbox_half_size,
        )

    def localize(self, detection, geometry_frame, camera_info):
        now_sec = rospy.get_time()
        if self.max_rate > 0.0:
            min_interval = 1.0 / self.max_rate
            if now_sec - self.last_process_time < min_interval:
                return
        self.last_process_time = now_sec

        if camera_info is None:
            rospy.logwarn_throttle(5.0, "CameraInfo is not available")
            return

        bbox = self._clip_bbox(
            detection["bbox"], camera_info.width, camera_info.height
        )
        if bbox is None:
            rospy.logwarn_throttle(5.0, "Detection bbox is outside the image")
            return

        try:
            validate_camera_matrix(
                camera_info.K,
                camera_info.width,
                camera_info.height,
            )
            if self.geometry_input == "point_cloud":
                if geometry_frame.header.frame_id not in (
                    self.output_frame,
                    self.point_cloud_frame,
                ):
                    raise ValueError(
                        "point cloud frame mismatch: expected %s or %s, got %s"
                        % (
                            self.output_frame,
                            self.point_cloud_frame,
                            geometry_frame.header.frame_id,
                        )
                    )
                points = self.point_cloud_xyz(geometry_frame)
                if geometry_frame.header.frame_id == self.point_cloud_frame:
                    points = transform_points(
                        points,
                        self.depth_to_rgb_rotation,
                        self.depth_to_rgb_translation,
                    )
            else:
                if self.depth_camera_info is None:
                    raise ValueError("Depth CameraInfo is not available")
                if geometry_frame.header.frame_id != self.depth_frame:
                    raise ValueError(
                        "depth frame mismatch: expected %s, got %s"
                        % (self.depth_frame, geometry_frame.header.frame_id)
                    )
                if (
                    int(self.depth_camera_info.width) != int(geometry_frame.width)
                    or int(self.depth_camera_info.height)
                    != int(geometry_frame.height)
                ):
                    raise ValueError(
                        "depth CameraInfo dimensions do not match depth image"
                    )
                validate_camera_matrix(
                    self.depth_camera_info.K,
                    self.depth_camera_info.width,
                    self.depth_camera_info.height,
                )
                depth_m = depth_array_from_bytes(
                    geometry_frame.data,
                    geometry_frame.width,
                    geometry_frame.height,
                    geometry_frame.step,
                    geometry_frame.encoding,
                    geometry_frame.is_bigendian,
                )
                points = depth_points_in_rgb_bbox(
                    depth_m=depth_m,
                    depth_camera_matrix=self.depth_camera_info.K,
                    rgb_camera_matrix=camera_info.K,
                    depth_to_rgb_rotation=self.depth_to_rgb_rotation,
                    depth_to_rgb_translation=self.depth_to_rgb_translation,
                    rgb_bbox=bbox,
                    min_depth=self.min_depth,
                    max_depth=self.max_depth,
                    margin_px=self.roi_margin_px,
                )
            target = estimate_target_point(
                points_xyz=points,
                camera_matrix=camera_info.K,
                bbox=bbox,
                min_depth=self.min_depth,
                max_depth=self.max_depth,
                central_roi_scale=self.central_roi_scale,
                min_points=self.min_points,
                depth_bin_size=self.depth_bin_size,
                min_cluster_fraction=self.min_cluster_fraction,
            )
        except (KeyError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(
                5.0,
                "%s localization failed: %s",
                self.geometry_input,
                error,
            )
            return
        if target is None:
            rospy.logwarn_throttle(
                5.0,
                "No supported depth cluster for %s bbox=%s",
                detection["class_name"],
                bbox,
            )
            return

        output = PointStamped()
        output.header.stamp = geometry_frame.header.stamp
        output.header.frame_id = self.output_frame
        output.point.x, output.point.y, output.point.z = target
        self.point_pub.publish(output)
        rospy.loginfo(
            "target_point source=%s class=%s xyz=(%.4f, %.4f, %.4f)",
            self.source_mode,
            detection["class_name"],
            output.point.x,
            output.point.y,
            output.point.z,
        )

    @staticmethod
    def _clip_bbox(bbox, width, height):
        if width <= 0 or height <= 0:
            return None
        x_min, y_min, x_max, y_max = [int(round(value)) for value in bbox]
        x_min = max(0, min(width - 1, x_min))
        y_min = max(0, min(height - 1, y_min))
        x_max = max(0, min(width, x_max))
        y_max = max(0, min(height, y_max))
        if x_max <= x_min or y_max <= y_min:
            return None
        return x_min, y_min, x_max, y_max

    @staticmethod
    def point_cloud_xyz(cloud):
        field_offsets = {field.name: field.offset for field in cloud.fields}
        for field_name in ("x", "y", "z"):
            if field_name not in field_offsets:
                raise KeyError("PointCloud2 is missing '{}'".format(field_name))
        byte_order = ">" if cloud.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ("x", "y", "z"),
                "formats": (
                    byte_order + "f4",
                    byte_order + "f4",
                    byte_order + "f4",
                ),
                "offsets": (
                    field_offsets["x"],
                    field_offsets["y"],
                    field_offsets["z"],
                ),
                "itemsize": cloud.point_step,
            }
        )
        count = int(cloud.width) * int(cloud.height)
        structured = np.frombuffer(cloud.data, dtype=dtype, count=count)
        return np.column_stack(
            (structured["x"], structured["y"], structured["z"])
        )


def main():
    rospy.init_node("rgbd_target_localizer")
    RgbdTargetLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
