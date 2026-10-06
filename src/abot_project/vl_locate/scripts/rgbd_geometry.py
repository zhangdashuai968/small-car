#!/usr/bin/env python3
from collections import OrderedDict, deque
import math

import numpy as np


def image_array_from_bytes(data, width, height, step, encoding):
    if encoding not in ("rgb8", "bgr8"):
        raise ValueError("unsupported image encoding: %s" % encoding)
    row_bytes = int(width) * 3
    if int(step) < row_bytes:
        raise ValueError("image step is smaller than packed RGB row")
    raw = np.frombuffer(data, dtype=np.uint8)
    required = int(height) * int(step)
    if raw.size < required:
        raise ValueError("image data is shorter than height * step")
    rows = raw[:required].reshape((int(height), int(step)))
    return rows[:, :row_bytes].reshape((int(height), int(width), 3)).copy()


def depth_array_from_bytes(
    data,
    width,
    height,
    step,
    encoding,
    is_bigendian,
):
    width = int(width)
    height = int(height)
    step = int(step)
    if width <= 0 or height <= 0:
        raise ValueError("depth image dimensions must be positive")
    if encoding == "16UC1":
        item_size = 2
        dtype_code = "u2"
        scale = 0.001
    elif encoding == "32FC1":
        item_size = 4
        dtype_code = "f4"
        scale = 1.0
    else:
        raise ValueError("unsupported depth encoding: %s" % encoding)
    row_bytes = width * item_size
    if step < row_bytes:
        raise ValueError("depth image step is smaller than packed row")
    required = height * step
    if len(data) < required:
        raise ValueError("depth image data is shorter than height * step")
    byte_order = ">" if bool(is_bigendian) else "<"
    values = np.ndarray(
        shape=(height, width),
        dtype=np.dtype(byte_order + dtype_code),
        buffer=data,
        strides=(step, item_size),
    )
    return values.astype(np.float32, copy=True) * scale


def compute_zoom_geometry(width, height, crop_ratios):
    x1_ratio, y1_ratio, x2_ratio, y2_ratio = crop_ratios
    x1 = max(0, min(width - 1, int(width * x1_ratio)))
    y1 = max(0, min(height - 1, int(height * y1_ratio)))
    x2 = max(x1 + 1, min(width, int(width * x2_ratio)))
    y2 = max(y1 + 1, min(height, int(height * y2_ratio)))

    roi_width = x2 - x1
    roi_height = y2 - y1
    scale = min(float(width) / roi_width, float(height) / roi_height)
    new_width = max(1, min(width, int(roi_width * scale)))
    new_height = max(1, min(height, int(roi_height * scale)))

    return {
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "scale": scale,
        "new_width": new_width,
        "new_height": new_height,
        "start_x": (width - new_width) // 2,
        "start_y": (height - new_height) // 2,
    }


def transform_camera_matrix(camera_matrix, geometry):
    matrix = list(camera_matrix)
    scale = geometry["scale"]
    matrix[0] *= scale
    matrix[2] = (matrix[2] - geometry["x1"]) * scale + geometry["start_x"]
    matrix[4] *= scale
    matrix[5] = (matrix[5] - geometry["y1"]) * scale + geometry["start_y"]
    return matrix


def transform_projection_matrix(projection_matrix, geometry):
    matrix = list(projection_matrix)
    scale = geometry["scale"]
    matrix[0] *= scale
    matrix[2] = (matrix[2] - geometry["x1"]) * scale + geometry["start_x"]
    matrix[3] *= scale
    matrix[5] *= scale
    matrix[6] = (matrix[6] - geometry["y1"]) * scale + geometry["start_y"]
    matrix[7] *= scale
    return matrix


def select_detection(detections, target_class):
    candidates = list(detections)
    if target_class:
        candidates = [
            detection
            for detection in candidates
            if detection.get("class_name", "") == target_class
        ]
    if not candidates:
        return None
    return max(candidates, key=lambda detection: float(detection.get("probability", 0.0)))


def validate_camera_matrix(camera_matrix, width, height):
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape((-1,))
    if matrix.size != 9:
        raise ValueError("camera matrix must contain 9 values")
    if not np.isfinite(matrix).all():
        raise ValueError("camera matrix contains non-finite values")
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        raise ValueError("camera image dimensions must be positive")
    fx, fy = float(matrix[0]), float(matrix[4])
    cx, cy = float(matrix[2]), float(matrix[5])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if not (0.0 <= cx < width and 0.0 <= cy < height):
        raise ValueError(
            "camera principal point is outside image bounds: "
            "cx=%.6f cy=%.6f width=%d height=%d"
            % (cx, cy, width, height)
        )
    if abs(float(matrix[8]) - 1.0) > 1e-6:
        raise ValueError("camera matrix K[8] must equal 1")
    return matrix


def transform_points(points_xyz, rotation, translation):
    points = np.asarray(points_xyz, dtype=np.float64).reshape((-1, 3))
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64).reshape((-1,))
    if rotation.shape != (3, 3):
        raise ValueError("rotation must have shape 3x3")
    if translation.size != 3:
        raise ValueError("translation must contain 3 values")
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError("rigid transform contains non-finite values")
    if not np.allclose(
        np.matmul(rotation.T, rotation),
        np.eye(3),
        rtol=0.0,
        atol=1e-6,
    ):
        raise ValueError("rotation must be orthonormal")
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-6:
        raise ValueError("rotation determinant must equal +1")
    return np.matmul(points, rotation.T) + translation


def compute_depth_candidate_roi(
    rgb_bbox,
    rgb_camera_matrix,
    depth_camera_matrix,
    depth_to_rgb_rotation,
    depth_to_rgb_translation,
    min_depth,
    max_depth,
    depth_width,
    depth_height,
    margin_px,
):
    rgb_k = validate_camera_matrix(
        rgb_camera_matrix,
        max(1, int(math.ceil(max(float(rgb_bbox[2]), rgb_camera_matrix[2] * 2.0)))),
        max(1, int(math.ceil(max(float(rgb_bbox[3]), rgb_camera_matrix[5] * 2.0)))),
    )
    depth_k = validate_camera_matrix(
        depth_camera_matrix,
        depth_width,
        depth_height,
    )
    rotation = np.asarray(depth_to_rgb_rotation, dtype=np.float64).reshape((3, 3))
    translation = np.asarray(
        depth_to_rgb_translation,
        dtype=np.float64,
    ).reshape((3,))
    transform_points(np.empty((0, 3)), rotation, translation)
    min_depth = float(min_depth)
    max_depth = float(max_depth)
    if not (0.0 < min_depth < max_depth):
        raise ValueError("depth range must satisfy 0 < min_depth < max_depth")

    x_min, y_min, x_max, y_max = [float(value) for value in rgb_bbox]
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("rgb bbox must have positive area")
    rgb_fx, rgb_fy = float(rgb_k[0]), float(rgb_k[4])
    rgb_cx, rgb_cy = float(rgb_k[2]), float(rgb_k[5])
    samples_rgb = []
    for depth in (min_depth, max_depth):
        for u in (x_min, x_max):
            for v in (y_min, y_max):
                samples_rgb.append(
                    [
                        (u - rgb_cx) * depth / rgb_fx,
                        (v - rgb_cy) * depth / rgb_fy,
                        depth,
                    ]
                )
    samples_rgb = np.asarray(samples_rgb, dtype=np.float64)
    samples_depth = np.matmul(samples_rgb - translation, rotation)
    if not np.isfinite(samples_depth).all():
        raise ValueError("depth ROI projection contains non-finite values")
    if np.any(samples_depth[:, 2] <= 0.0):
        raise ValueError("depth ROI projection crosses the depth camera plane")

    depth_u = (
        float(depth_k[0]) * samples_depth[:, 0] / samples_depth[:, 2]
        + float(depth_k[2])
    )
    depth_v = (
        float(depth_k[4]) * samples_depth[:, 1] / samples_depth[:, 2]
        + float(depth_k[5])
    )
    margin = max(0, int(margin_px))
    roi_x_min = max(0, int(math.floor(float(np.min(depth_u)))) - margin)
    roi_y_min = max(0, int(math.floor(float(np.min(depth_v)))) - margin)
    roi_x_max = min(
        int(depth_width),
        int(math.ceil(float(np.max(depth_u)))) + margin,
    )
    roi_y_max = min(
        int(depth_height),
        int(math.ceil(float(np.max(depth_v)))) + margin,
    )
    if roi_x_max <= roi_x_min or roi_y_max <= roi_y_min:
        raise ValueError("projected depth ROI is outside the depth image")
    return roi_x_min, roi_y_min, roi_x_max, roi_y_max


def depth_points_in_rgb_bbox(
    depth_m,
    depth_camera_matrix,
    rgb_camera_matrix,
    depth_to_rgb_rotation,
    depth_to_rgb_translation,
    rgb_bbox,
    min_depth,
    max_depth,
    margin_px,
):
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth image must have shape HxW")
    depth_height, depth_width = depth.shape
    depth_k = validate_camera_matrix(
        depth_camera_matrix,
        depth_width,
        depth_height,
    )
    roi = compute_depth_candidate_roi(
        rgb_bbox=rgb_bbox,
        rgb_camera_matrix=rgb_camera_matrix,
        depth_camera_matrix=depth_k,
        depth_to_rgb_rotation=depth_to_rgb_rotation,
        depth_to_rgb_translation=depth_to_rgb_translation,
        min_depth=min_depth,
        max_depth=max_depth,
        depth_width=depth_width,
        depth_height=depth_height,
        margin_px=margin_px,
    )
    x_min, y_min, x_max, y_max = roi
    roi_depth = depth[y_min:y_max, x_min:x_max]
    valid = (
        np.isfinite(roi_depth)
        & (roi_depth >= float(min_depth))
        & (roi_depth <= float(max_depth))
    )
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    z = roi_depth[rows, columns].astype(np.float64)
    u = columns.astype(np.float64) + float(x_min)
    v = rows.astype(np.float64) + float(y_min)
    points_depth = np.column_stack(
        (
            (u - float(depth_k[2])) * z / float(depth_k[0]),
            (v - float(depth_k[5])) * z / float(depth_k[4]),
            z,
        )
    )
    points_rgb = transform_points(
        points_depth,
        depth_to_rgb_rotation,
        depth_to_rgb_translation,
    )
    finite = np.isfinite(points_rgb).all(axis=1) & (points_rgb[:, 2] > 0.0)
    points_rgb = points_rgb[finite]
    if len(points_rgb) == 0:
        return points_rgb
    return _points_in_bbox(
        points_rgb,
        np.asarray(rgb_camera_matrix, dtype=np.float64).reshape((9,)),
        rgb_bbox,
    )


def _points_in_bbox(points_xyz, camera_matrix, bbox):
    fx, fy = float(camera_matrix[0]), float(camera_matrix[4])
    cx, cy = float(camera_matrix[2]), float(camera_matrix[5])
    x_min, y_min, x_max, y_max = bbox

    z = points_xyz[:, 2]
    u = fx * points_xyz[:, 0] / z + cx
    v = fy * points_xyz[:, 1] / z + cy
    mask = (
        (u >= x_min)
        & (u <= x_max)
        & (v >= y_min)
        & (v <= y_max)
    )
    return points_xyz[mask]


def _scaled_bbox(bbox, scale):
    x_min, y_min, x_max, y_max = [float(value) for value in bbox]
    center_x = (x_min + x_max) * 0.5
    center_y = (y_min + y_max) * 0.5
    half_width = max(0.5, (x_max - x_min) * 0.5 * scale)
    half_height = max(0.5, (y_max - y_min) * 0.5 * scale)
    return (
        center_x - half_width,
        center_y - half_height,
        center_x + half_width,
        center_y + half_height,
    )


def estimate_target_point(
    points_xyz,
    camera_matrix,
    bbox,
    min_depth,
    max_depth,
    central_roi_scale,
    min_points,
    depth_bin_size,
    min_cluster_fraction,
):
    points = np.asarray(points_xyz, dtype=np.float32).reshape((-1, 3))
    finite = np.isfinite(points).all(axis=1)
    depth_valid = (points[:, 2] >= min_depth) & (points[:, 2] <= max_depth)
    points = points[finite & depth_valid]
    if len(points) < min_points:
        return None

    central_points = _points_in_bbox(
        points,
        camera_matrix,
        _scaled_bbox(bbox, central_roi_scale),
    )
    selected_points = central_points
    if len(selected_points) < min_points:
        selected_points = _points_in_bbox(points, camera_matrix, bbox)
    if len(selected_points) < min_points:
        return None

    bin_ids = np.floor(selected_points[:, 2] / depth_bin_size).astype(np.int64)
    unique_ids, counts = np.unique(bin_ids, return_counts=True)
    required_support = max(
        int(min_points),
        int(math.ceil(len(selected_points) * min_cluster_fraction)),
    )
    supported_ids = unique_ids[counts >= required_support]
    if len(supported_ids) == 0:
        return None

    nearest_bin = int(np.min(supported_ids))
    cluster = selected_points[bin_ids == nearest_bin]
    if len(cluster) < required_support:
        return None
    return tuple(float(value) for value in np.median(cluster, axis=0))


class TimedFrameCache:
    def __init__(self, history_size, pinned_size, ttl_sec):
        self.history = deque(maxlen=int(history_size))
        self.pinned = OrderedDict()
        self.pinned_size = int(pinned_size)
        self.ttl_sec = float(ttl_sec)

    def add(self, stamp_sec, frame):
        self.history.append((float(stamp_sec), frame))

    def nearest(self, stamp_sec, tolerance_sec):
        if not self.history:
            return None
        requested = float(stamp_sec)
        delta, frame = min(
            ((abs(stamp - requested), frame) for stamp, frame in self.history),
            key=lambda item: item[0],
        )
        if delta > float(tolerance_sec):
            return None
        return frame

    def pin(self, key, stamp_sec, tolerance_sec, now_sec):
        self._prune(now_sec)
        frame = self.nearest(stamp_sec, tolerance_sec)
        if frame is None:
            return False
        self.pinned.pop(key, None)
        self.pinned[key] = (float(now_sec), frame)
        while len(self.pinned) > self.pinned_size:
            self.pinned.popitem(last=False)
        return True

    def get_pinned(self, key, now_sec):
        self._prune(now_sec)
        value = self.pinned.get(key)
        return None if value is None else value[1]

    def _prune(self, now_sec):
        expired = [
            key
            for key, (stored_at, _) in self.pinned.items()
            if float(now_sec) - stored_at > self.ttl_sec
        ]
        for key in expired:
            self.pinned.pop(key, None)
