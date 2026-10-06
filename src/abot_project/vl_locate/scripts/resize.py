#!/usr/bin/env python3
# _*_ coding:utf-8 _*_
import copy

import cv2
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

from rgbd_geometry import (
    compute_zoom_geometry,
    image_array_from_bytes,
    transform_camera_matrix,
    transform_projection_matrix,
)


class ImageZoomer:
    def __init__(self):
        self.input_image_topic = rospy.get_param(
            "~input_image_topic", "/camera/rgb/image_raw"
        )
        self.input_info_topic = rospy.get_param(
            "~input_camera_info_topic", "/camera/rgb/camera_info"
        )
        self.output_image_topic = rospy.get_param("~output_image_topic", "/new_topic")
        self.output_info_topic = rospy.get_param(
            "~output_camera_info_topic", "/new_topic/camera_info"
        )
        self.crop_ratios = (
            float(rospy.get_param("~crop_x1_ratio", 0.43)),
            float(rospy.get_param("~crop_y1_ratio", 0.53)),
            float(rospy.get_param("~crop_x2_ratio", 0.75)),
            float(rospy.get_param("~crop_y2_ratio", 0.78)),
        )
        self.sync_tolerance = float(rospy.get_param("~sync_tolerance", 0.10))

        self.image_pub = rospy.Publisher(
            self.output_image_topic, Image, queue_size=1
        )
        self.info_pub = rospy.Publisher(
            self.output_info_topic, CameraInfo, queue_size=1
        )
        self.image_sub = message_filters.Subscriber(self.input_image_topic, Image)
        self.info_sub = message_filters.Subscriber(self.input_info_topic, CameraInfo)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub],
            queue_size=10,
            slop=self.sync_tolerance,
        )
        self.synchronizer.registerCallback(self.callback)

        rospy.loginfo(
            "image_zoomer: %s + %s -> %s + %s",
            self.input_image_topic,
            self.input_info_topic,
            self.output_image_topic,
            self.output_info_topic,
        )

    def callback(self, image_msg, camera_info_msg):
        try:
            image = image_array_from_bytes(
                data=image_msg.data,
                width=image_msg.width,
                height=image_msg.height,
                step=image_msg.step,
                encoding=image_msg.encoding,
            )
        except ValueError as error:
            rospy.logerr_throttle(5.0, "image_zoomer conversion failed: %s", error)
            return

        height, width = image.shape[:2]
        geometry = compute_zoom_geometry(width, height, self.crop_ratios)
        roi = image[
            geometry["y1"] : geometry["y2"],
            geometry["x1"] : geometry["x2"],
        ]
        interpolation = (
            cv2.INTER_CUBIC if geometry["scale"] > 2.0 else cv2.INTER_LINEAR
        )
        zoomed_roi = cv2.resize(
            roi,
            (geometry["new_width"], geometry["new_height"]),
            interpolation=interpolation,
        )
        zoomed_image = np.zeros((height, width, 3), dtype=np.uint8)
        start_x = geometry["start_x"]
        start_y = geometry["start_y"]
        zoomed_image[
            start_y : start_y + geometry["new_height"],
            start_x : start_x + geometry["new_width"],
        ] = zoomed_roi

        output_image = Image()
        output_image.header = copy.deepcopy(image_msg.header)
        output_image.height = height
        output_image.width = width
        output_image.encoding = image_msg.encoding
        output_image.is_bigendian = image_msg.is_bigendian
        output_image.step = width * 3
        output_image.data = zoomed_image.tobytes()

        output_info = copy.deepcopy(camera_info_msg)
        output_info.header = copy.deepcopy(image_msg.header)
        output_info.width = width
        output_info.height = height
        output_info.K = transform_camera_matrix(camera_info_msg.K, geometry)
        output_info.P = transform_projection_matrix(camera_info_msg.P, geometry)
        output_info.roi.x_offset = 0
        output_info.roi.y_offset = 0
        output_info.roi.width = 0
        output_info.roi.height = 0
        output_info.roi.do_rectify = False

        self.image_pub.publish(output_image)
        self.info_pub.publish(output_info)


def main():
    rospy.init_node("image_zoomer")
    ImageZoomer()
    rospy.spin()


if __name__ == "__main__":
    main()
