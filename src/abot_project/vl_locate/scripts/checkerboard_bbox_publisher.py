#!/usr/bin/env python3
import threading

import rospy
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

from calibration_tools import detect_checkerboard_bbox
from rgbd_geometry import image_array_from_bytes


class CheckerboardBboxPublisher:
    def __init__(self):
        self.image_topic = str(
            rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        )
        self.output_topic = str(
            rospy.get_param(
                "~output_topic",
                "/yolov5/BoundingBoxes",
            )
        )
        self.expected_width = int(rospy.get_param("~expected_width", 1280))
        self.expected_height = int(rospy.get_param("~expected_height", 720))
        self.board_columns = int(rospy.get_param("~board_columns", 8))
        self.board_rows = int(rospy.get_param("~board_rows", 6))
        self.class_name = str(
            rospy.get_param("~class_name", "checkerboard")
        )
        self.outer_square_margin = float(
            rospy.get_param("~outer_square_margin", 1.0)
        )
        self.max_rate = float(rospy.get_param("~max_rate", 5.0))
        if self.max_rate <= 0.0:
            raise ValueError("~max_rate must be positive")
        self.min_interval = 1.0 / self.max_rate
        self.last_processed = 0.0
        self.processing = False
        self.lock = threading.RLock()
        self.publisher = rospy.Publisher(
            self.output_topic,
            BoundingBoxes,
            queue_size=5,
        )
        self.subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=8 * 1024 * 1024,
        )
        rospy.loginfo(
            "Checkerboard bbox publisher: topic=%s output=%s board=%dx%d rate=%.2f",
            self.image_topic,
            self.output_topic,
            self.board_columns,
            self.board_rows,
            self.max_rate,
        )

    def image_callback(self, message):
        with self.lock:
            now = rospy.get_time()
            if self.processing or now - self.last_processed < self.min_interval:
                return
            self.processing = True
            self.last_processed = now
        try:
            self.process_image(message)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(
                5.0,
                "Checkerboard frame rejected: %s",
                error,
            )
        finally:
            with self.lock:
                self.processing = False

    def process_image(self, message):
        if (
            int(message.width) != self.expected_width
            or int(message.height) != self.expected_height
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
        bbox = detect_checkerboard_bbox(
            image,
            columns=self.board_columns,
            rows=self.board_rows,
            outer_square_margin=self.outer_square_margin,
        )
        if bbox is None:
            rospy.loginfo_throttle(5.0, "Checkerboard not detected")
            return
        box = BoundingBox()
        box.probability = 1.0
        box.xmin = int(bbox[0])
        box.ymin = int(bbox[1])
        box.xmax = int(bbox[2])
        box.ymax = int(bbox[3])
        box.num = 0
        box.Class = self.class_name

        detections = BoundingBoxes()
        detections.header = message.header
        detections.image_header = message.header
        detections.bounding_boxes = [box]
        self.publisher.publish(detections)
        rospy.loginfo_throttle(
            2.0,
            "Checkerboard bbox [%d,%d,%d,%d]",
            box.xmin,
            box.ymin,
            box.xmax,
            box.ymax,
        )


def main():
    rospy.init_node("checkerboard_bbox_publisher")
    CheckerboardBboxPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
