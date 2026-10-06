#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
from collections import deque

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

from yolov5_runtime import (
    TensorRtRunner,
    backend_kind,
    image_array_from_ros_bytes,
    letterbox_image,
    load_class_names,
    numpy_non_max_suppression,
    parse_bool,
    resolve_device,
    scale_boxes_to_original,
)


class YoloDetector:
    def __init__(self):
        self.yolov5_path = str(rospy.get_param("/yolov5_path", ""))
        if not os.path.isdir(self.yolov5_path):
            raise RuntimeError(
                "yolov5 source directory does not exist: %s"
                % self.yolov5_path
            )
        if self.yolov5_path not in sys.path:
            sys.path.insert(0, self.yolov5_path)

        self.weight_path = os.path.abspath(
            str(rospy.get_param("~weight_path", ""))
        )
        if not os.path.isfile(self.weight_path):
            raise RuntimeError(
                "weight_path does not exist; automatic downloads are disabled: %s"
                % self.weight_path
            )
        self.backend = backend_kind(self.weight_path)

        explicit_device = (
            rospy.get_param("~device")
            if rospy.has_param("~device")
            else None
        )
        legacy_use_cpu = rospy.get_param("~use_cpu", False)
        self.torch = None
        if self.backend == "pytorch":
            import torch

            self.torch = torch
            requested_device = resolve_device(
                explicit_device,
                legacy_use_cpu,
                torch.cuda.is_available(),
            )
        else:
            requested_device = resolve_device(
                explicit_device,
                legacy_use_cpu,
                cuda_available=True,
            )
        if self.backend == "tensorrt" and requested_device == "cpu":
            raise RuntimeError("TensorRT engine requires a CUDA device")
        self.device = requested_device

        self.input_width = int(rospy.get_param("~input_width", 640))
        self.input_height = int(rospy.get_param("~input_height", 384))
        if (
            self.input_width <= 0
            or self.input_height <= 0
            or self.input_width % 32 != 0
            or self.input_height % 32 != 0
        ):
            raise ValueError(
                "input_width and input_height must be positive multiples of 32"
            )
        self.conf = float(rospy.get_param("~conf", 0.3))
        self.iou = float(rospy.get_param("~iou", 0.45))
        self.max_det = int(rospy.get_param("~max_det", 100))
        if not (0.0 <= self.conf <= 1.0):
            raise ValueError("~conf must be in [0, 1]")
        if not (0.0 <= self.iou <= 1.0):
            raise ValueError("~iou must be in [0, 1]")
        if self.max_det <= 0:
            raise ValueError("~max_det must be positive")
        self.publish_debug_image = parse_bool(
            rospy.get_param("~publish_debug_image", False)
        )

        data_path = str(
            rospy.get_param(
                "~data_path",
                os.path.join(self.yolov5_path, "data", "coco128.yaml"),
            )
        )
        if not os.path.isfile(data_path):
            raise RuntimeError("class data YAML does not exist: %s" % data_path)
        requested_half = parse_bool(rospy.get_param("~half", True))
        if self.backend == "tensorrt":
            self.model = TensorRtRunner(self.weight_path)
            expected_shape = (
                1,
                3,
                self.input_height,
                self.input_width,
            )
            if self.model.input_shape != expected_shape:
                raise RuntimeError(
                    "TensorRT engine input shape %s does not match requested %s"
                    % (self.model.input_shape, expected_shape)
                )
            self.half = self.model.input_dtype == np.float16
            self.names = load_class_names(data_path)
            if requested_half and self.model.input_dtype != np.float16:
                rospy.logwarn(
                    "TensorRT input binding is FP32; engine internals may still use FP16"
                )
        else:
            from models.common import DetectMultiBackend
            from utils.general import non_max_suppression
            from utils.torch_utils import select_device

            self.non_max_suppression = non_max_suppression
            self.device = select_device(requested_device)
            self.model = DetectMultiBackend(
                self.weight_path,
                device=self.device,
                data=data_path,
            )
            self.half = requested_half and self.device.type != "cpu"
            self.model.model.half() if self.half else self.model.model.float()
            self.names = self.model.names
        self.image_topic = str(
            rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        )
        self.pub_topic = str(
            rospy.get_param("~pub_topic", "/yolov5/BoundingBoxes")
        )
        self.box_pub = rospy.Publisher(
            self.pub_topic,
            BoundingBoxes,
            queue_size=1,
        )
        self.image_pub = None
        if self.publish_debug_image:
            self.image_pub = rospy.Publisher(
                "/yolov5/detection_image",
                Image,
                queue_size=1,
            )

        if self.backend == "pytorch" and self.device.type != "cpu":
            self.torch.backends.cudnn.benchmark = True
        warmup_shape = (
            1,
            3,
            self.input_height,
            self.input_width,
        )
        if self.backend == "tensorrt":
            warmup = np.zeros(
                warmup_shape,
                dtype=self.model.input_dtype,
            )
            for _ in range(3):
                self.model.infer(warmup)
        else:
            for _ in range(3):
                self.model.warmup(warmup_shape, half=self.half)

        self.latencies_ms = deque(maxlen=60)
        self.frame_count = 0
        self.color_cache = {}
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=64 * 1024 * 1024,
        )
        precision = (
            "FP16-engine/%s-input" % self.model.input_dtype
            if self.backend == "tensorrt"
            else ("FP16" if self.half else "FP32")
        )
        rospy.loginfo(
            "yolov5 ready: backend=%s precision=%s device=%s input=1x3x%dx%d model=%s output=%s",
            self.backend,
            precision,
            self.device,
            self.input_height,
            self.input_width,
            self.weight_path,
            self.pub_topic,
        )

    def image_callback(self, message):
        started = time.perf_counter()
        try:
            bgr = image_array_from_ros_bytes(
                message.data,
                message.width,
                message.height,
                message.step,
                message.encoding,
            )
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            input_image, geometry = letterbox_image(
                rgb,
                self.input_width,
                self.input_height,
            )
            input_array = np.ascontiguousarray(
                input_image.transpose((2, 0, 1))[None]
            )
            if self.backend == "tensorrt":
                input_array = input_array.astype(
                    self.model.input_dtype,
                    copy=False,
                )
                input_array /= np.asarray(
                    255.0,
                    dtype=self.model.input_dtype,
                )
                prediction = self.model.infer(input_array)
                detections = numpy_non_max_suppression(
                    prediction,
                    conf_threshold=self.conf,
                    iou_threshold=self.iou,
                    max_det=self.max_det,
                )
            else:
                tensor = self.torch.from_numpy(input_array).to(self.device)
                tensor = tensor.half() if self.half else tensor.float()
                tensor /= 255.0
                with self.torch.no_grad():
                    prediction = self.model(tensor)
                    detections = self.non_max_suppression(
                        prediction,
                        conf_thres=self.conf,
                        iou_thres=self.iou,
                        max_det=self.max_det,
                    )[0]
            if self.backend == "pytorch" and self.device.type != "cpu":
                self.torch.cuda.synchronize()
            output = self._bounding_boxes_message(
                message,
                detections,
                geometry,
            )
            self.box_pub.publish(output)
            if (
                self.image_pub is not None
                and self.image_pub.get_num_connections() > 0
            ):
                self._publish_debug_image(message, bgr, output)
        except (RuntimeError, TypeError, ValueError) as error:
            rospy.logerr_throttle(5.0, "YOLO inference failed: %s", error)
            return

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.latencies_ms.append(elapsed_ms)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            values = np.asarray(self.latencies_ms, dtype=np.float64)
            rospy.loginfo(
                "yolov5 runtime: frames=%d mean=%.2fms p95=%.2fms detections=%d",
                self.frame_count,
                float(np.mean(values)),
                float(np.percentile(values, 95)),
                len(output.bounding_boxes),
            )

    def _bounding_boxes_message(self, image_message, detections, geometry):
        output = BoundingBoxes()
        output.header = image_message.header
        output.image_header = image_message.header
        if detections is None or len(detections) == 0:
            return output

        if self.backend == "pytorch":
            detection_array = detections.detach().cpu().numpy()
        else:
            detection_array = np.asarray(detections, dtype=np.float32)
        scaled = scale_boxes_to_original(
            detection_array[:, :4],
            geometry,
            source_width=image_message.width,
            source_height=image_message.height,
        )
        count = int(len(detection_array))
        for index, detection in enumerate(detection_array):
            box = BoundingBox()
            box.probability = float(detection[4])
            box.xmin = int(round(float(scaled[index, 0])))
            box.ymin = int(round(float(scaled[index, 1])))
            box.xmax = int(round(float(scaled[index, 2])))
            box.ymax = int(round(float(scaled[index, 3])))
            box.num = count
            class_index = int(round(float(detection[5])))
            if isinstance(self.names, dict):
                class_name = self.names.get(class_index, str(class_index))
            elif 0 <= class_index < len(self.names):
                class_name = self.names[class_index]
            else:
                class_name = str(class_index)
            box.Class = str(class_name)
            output.bounding_boxes.append(box)
        return output

    def _publish_debug_image(self, source_message, bgr, boxes):
        debug = bgr.copy()
        for box in boxes.bounding_boxes:
            color = self.color_cache.get(box.Class)
            if color is None:
                seed = abs(hash(box.Class)) % (2 ** 32)
                generator = np.random.RandomState(seed)
                color = tuple(int(value) for value in generator.randint(32, 224, 3))
                self.color_cache[box.Class] = color
            cv2.rectangle(
                debug,
                (box.xmin, box.ymin),
                (box.xmax, box.ymax),
                color,
                2,
            )
            cv2.putText(
                debug,
                "%s %.2f" % (box.Class, box.probability),
                (box.xmin, max(16, box.ymin - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        output = Image()
        output.header = source_message.header
        output.height = source_message.height
        output.width = source_message.width
        output.encoding = "bgr8"
        output.is_bigendian = 0
        output.step = int(source_message.width) * 3
        output.data = debug.tobytes()
        self.image_pub.publish(output)


def main():
    rospy.init_node("abot_yolov5")
    try:
        YoloDetector()
    except Exception as error:
        rospy.logfatal("YOLO startup failed: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
