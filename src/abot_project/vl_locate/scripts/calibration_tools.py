#!/usr/bin/env python3
import math

import cv2
import numpy as np


def build_checkerboard_points(columns, rows, square_size):
    columns = int(columns)
    rows = int(rows)
    square_size = float(square_size)
    if columns < 2 or rows < 2:
        raise ValueError("checkerboard must contain at least 2x2 inner corners")
    if not np.isfinite(square_size) or square_size <= 0.0:
        raise ValueError("square_size must be a positive finite value")
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = (
        np.mgrid[0:columns, 0:rows]
        .T.reshape((-1, 2))
        .astype(np.float32)
    )
    points[:, :2] *= square_size
    return points


def checkerboard_bbox_from_corners(
    corners,
    image_size,
    columns,
    rows,
    outer_square_margin=1.0,
):
    width, height = [int(value) for value in image_size]
    columns = int(columns)
    rows = int(rows)
    margin = float(outer_square_margin)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if columns < 2 or rows < 2:
        raise ValueError("checkerboard must contain at least 2x2 corners")
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("outer_square_margin must be non-negative")
    points = np.asarray(corners, dtype=np.float32).reshape((-1, 2))
    if points.shape[0] != columns * rows:
        raise ValueError("corner count does not match checkerboard shape")
    if not np.isfinite(points).all():
        raise ValueError("checkerboard corners contain non-finite values")
    grid = points.reshape((rows, columns, 2))
    horizontal = np.linalg.norm(
        grid[:, 1:, :] - grid[:, :-1, :],
        axis=2,
    )
    vertical = np.linalg.norm(
        grid[1:, :, :] - grid[:-1, :, :],
        axis=2,
    )
    spacing = np.concatenate(
        (horizontal.reshape((-1,)), vertical.reshape((-1,)))
    )
    spacing = spacing[np.isfinite(spacing) & (spacing > 0.0)]
    if not spacing.size:
        raise ValueError("checkerboard corner spacing is invalid")
    padding = float(np.median(spacing)) * margin
    x_min = max(0, int(math.floor(float(points[:, 0].min()) - padding)))
    y_min = max(0, int(math.floor(float(points[:, 1].min()) - padding)))
    x_max = min(
        width - 1,
        int(math.ceil(float(points[:, 0].max()) + padding)),
    )
    y_max = min(
        height - 1,
        int(math.ceil(float(points[:, 1].max()) + padding)),
    )
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("checkerboard bbox is empty")
    return (x_min, y_min, x_max, y_max)


def detect_checkerboard_bbox(
    image,
    columns,
    rows,
    outer_square_margin=1.0,
):
    array = np.asarray(image)
    if array.ndim == 2:
        gray = array
    elif array.ndim == 3 and array.shape[2] == 3:
        gray = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError("image must be grayscale or three-channel color")
    if gray.dtype != np.uint8:
        raise ValueError("checkerboard image must use uint8 pixels")
    pattern_size = (int(columns), int(rows))
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(
        gray,
        pattern_size,
        flags=flags,
    )
    if not found:
        return None
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
    return checkerboard_bbox_from_corners(
        corners,
        image_size=(array.shape[1], array.shape[0]),
        columns=columns,
        rows=rows,
        outer_square_margin=outer_square_margin,
    )


def view_feature(corners, image_size):
    width, height = [int(value) for value in image_size]
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    points = np.asarray(corners, dtype=np.float32).reshape((-1, 2))
    if points.shape[0] < 4 or not np.isfinite(points).all():
        raise ValueError("view corners must contain at least four finite points")
    centroid = points.mean(axis=0)
    hull = cv2.convexHull(points)
    area_fraction = float(cv2.contourArea(hull)) / float(width * height)
    rectangle = cv2.minAreaRect(points)
    angle = float(rectangle[2]) / 180.0
    return [
        float(centroid[0]) / width,
        float(centroid[1]) / height,
        math.sqrt(max(0.0, area_fraction)),
        angle,
    ]


def is_diverse_view(feature, accepted_features, min_distance):
    feature = np.asarray(feature, dtype=np.float64).reshape((-1,))
    if feature.size != 4 or not np.isfinite(feature).all():
        raise ValueError("view feature must contain four finite values")
    min_distance = float(min_distance)
    if min_distance < 0.0:
        raise ValueError("min_distance must be non-negative")
    accepted = list(accepted_features)
    if not accepted:
        return True
    distances = [
        float(np.linalg.norm(feature - np.asarray(item, dtype=np.float64)))
        for item in accepted
    ]
    return min(distances) >= min_distance


def calibrate_pinhole(image_points, image_size, object_points, flags=0):
    width, height = [int(value) for value in image_size]
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    observations = [
        np.asarray(points, dtype=np.float32).reshape((-1, 1, 2))
        for points in image_points
    ]
    if len(observations) < 3:
        raise ValueError("at least three calibration views are required")
    board = np.asarray(object_points, dtype=np.float32).reshape((-1, 3))
    if board.shape[0] < 4:
        raise ValueError("object_points must contain at least four points")
    if any(points.shape[0] != board.shape[0] for points in observations):
        raise ValueError("every view must contain all checkerboard corners")
    object_views = [board.copy() for _ in observations]

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_views,
        observations,
        (width, height),
        None,
        None,
        flags=int(flags),
    )
    view_errors = []
    for object_view, observed, rvec, tvec in zip(
        object_views,
        observations,
        rvecs,
        tvecs,
    ):
        projected, _ = cv2.projectPoints(
            object_view,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        error = cv2.norm(observed, projected, cv2.NORM_L2)
        error /= math.sqrt(float(observed.shape[0]))
        view_errors.append(float(error))
    return {
        "rms": float(rms),
        "camera_matrix": camera_matrix,
        "distortion": distortion.reshape((-1,)),
        "view_errors": view_errors,
        "rvecs": [np.asarray(value).reshape((-1,)) for value in rvecs],
        "tvecs": [np.asarray(value).reshape((-1,)) for value in tvecs],
        "image_width": width,
        "image_height": height,
    }


def calibration_gate(
    result,
    features,
    required_views=20,
    max_rms=0.5,
    max_view_p95=1.0,
    min_centroid_span=0.25,
    min_scale_span=0.05,
):
    features = np.asarray(features, dtype=np.float64).reshape((-1, 4))
    view_errors = np.asarray(result.get("view_errors", []), dtype=np.float64)
    reasons = []
    view_count = int(features.shape[0])
    if view_count < int(required_views):
        reasons.append(
            "view count %d is below required %d"
            % (view_count, int(required_views))
        )

    rms = float(result.get("rms", float("inf")))
    if not np.isfinite(rms) or rms > float(max_rms):
        reasons.append(
            "RMS %.6f exceeds %.6f" % (rms, float(max_rms))
        )

    view_p95 = (
        float(np.percentile(view_errors, 95))
        if view_errors.size
        else float("inf")
    )
    if not np.isfinite(view_p95) or view_p95 > float(max_view_p95):
        reasons.append(
            "view P95 %.6f exceeds %.6f"
            % (view_p95, float(max_view_p95))
        )

    if features.size:
        centroid_span_x = float(np.ptp(features[:, 0]))
        centroid_span_y = float(np.ptp(features[:, 1]))
        scale_span = float(np.ptp(features[:, 2]))
    else:
        centroid_span_x = 0.0
        centroid_span_y = 0.0
        scale_span = 0.0
    if min(centroid_span_x, centroid_span_y) < float(min_centroid_span):
        reasons.append(
            "centroid coverage span %.6f/%.6f is below %.6f"
            % (
                centroid_span_x,
                centroid_span_y,
                float(min_centroid_span),
            )
        )
    if scale_span < float(min_scale_span):
        reasons.append(
            "scale coverage span %.6f is below %.6f"
            % (scale_span, float(min_scale_span))
        )

    return {
        "passed": not reasons,
        "reasons": reasons,
        "view_count": view_count,
        "rms": rms,
        "view_error_p95": view_p95,
        "centroid_span_x": centroid_span_x,
        "centroid_span_y": centroid_span_y,
        "scale_span": scale_span,
    }


def accuracy_summary(
    points_xyz,
    expected_xyz,
    attempts,
    required_attempts=20,
    min_valid_rate=0.90,
    max_median_error=0.020,
    max_p95_error=0.040,
):
    expected = np.asarray(expected_xyz, dtype=np.float64).reshape((-1,))
    if expected.size != 3 or not np.isfinite(expected).all():
        raise ValueError("expected_xyz must contain three finite values")
    points = np.asarray(points_xyz, dtype=np.float64).reshape((-1, 3))
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    attempts = int(attempts)
    if attempts < 0:
        raise ValueError("attempts must be non-negative")
    valid_count = min(int(points.shape[0]), attempts) if attempts else 0
    points = points[:valid_count]
    valid_rate = float(valid_count) / attempts if attempts else 0.0
    reasons = []
    if attempts < int(required_attempts):
        reasons.append(
            "attempt count %d is below required %d"
            % (attempts, int(required_attempts))
        )
    if valid_rate < float(min_valid_rate):
        reasons.append(
            "valid rate %.6f is below %.6f"
            % (valid_rate, float(min_valid_rate))
        )

    if valid_count:
        residuals = points - expected
        errors = np.linalg.norm(residuals, axis=1)
        median_error = float(np.median(errors))
        p95_error = float(np.percentile(errors, 95))
        median_point = np.median(points, axis=0)
        jitter = np.linalg.norm(points - median_point, axis=1)
        axis_abs = np.abs(residuals)
        axis_median = np.median(axis_abs, axis=0)
        axis_p95 = np.percentile(axis_abs, 95, axis=0)
        jitter_median = float(np.median(jitter))
        jitter_p95 = float(np.percentile(jitter, 95))
    else:
        median_error = float("inf")
        p95_error = float("inf")
        median_point = np.asarray([float("nan")] * 3)
        axis_median = np.asarray([float("inf")] * 3)
        axis_p95 = np.asarray([float("inf")] * 3)
        jitter_median = float("inf")
        jitter_p95 = float("inf")

    if median_error > float(max_median_error):
        reasons.append(
            "median error %.6f exceeds %.6f"
            % (median_error, float(max_median_error))
        )
    if p95_error > float(max_p95_error):
        reasons.append(
            "P95 error %.6f exceeds %.6f"
            % (p95_error, float(max_p95_error))
        )

    def finite_or_none(value):
        return float(value) if np.isfinite(value) else None

    def finite_vector_or_none(values):
        return [finite_or_none(value) for value in values]

    return {
        "passed": not reasons,
        "reasons": reasons,
        "attempts": attempts,
        "valid_count": valid_count,
        "valid_rate": valid_rate,
        "expected_xyz_m": expected.tolist(),
        "median_xyz_m": finite_vector_or_none(median_point),
        "median_error_m": finite_or_none(median_error),
        "p95_error_m": finite_or_none(p95_error),
        "axis_median_abs_error_m": finite_vector_or_none(axis_median),
        "axis_p95_abs_error_m": finite_vector_or_none(axis_p95),
        "jitter_median_m": finite_or_none(jitter_median),
        "jitter_p95_m": finite_or_none(jitter_p95),
        "thresholds": {
            "required_attempts": int(required_attempts),
            "min_valid_rate": float(min_valid_rate),
            "max_median_error_m": float(max_median_error),
            "max_p95_error_m": float(max_p95_error),
        },
    }
