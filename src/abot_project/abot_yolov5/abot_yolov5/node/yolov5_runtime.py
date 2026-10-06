#!/usr/bin/env python3
import ctypes
import ctypes.util
import os

import cv2
import numpy as np


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off", ""):
        return False
    raise ValueError("invalid boolean value: %s" % value)


def resolve_device(device, legacy_use_cpu, cuda_available):
    if device is None or str(device).strip() == "":
        requested = "cpu" if parse_bool(legacy_use_cpu) else "0"
    else:
        requested = str(device).strip().lower().replace("cuda:", "")
    if requested != "cpu" and not bool(cuda_available):
        raise RuntimeError("CUDA device requested but torch.cuda is unavailable")
    if requested not in ("cpu", "0"):
        raise ValueError("device must be 'cpu' or '0'")
    return requested


def backend_kind(model_path):
    suffix = os.path.splitext(str(model_path))[1].lower()
    if suffix == ".pt":
        return "pytorch"
    if suffix == ".engine":
        return "tensorrt"
    raise ValueError("weight_path must end in .pt or .engine")


def compute_letterbox_geometry(
    source_width,
    source_height,
    target_width,
    target_height,
):
    source_width = int(source_width)
    source_height = int(source_height)
    target_width = int(target_width)
    target_height = int(target_height)
    if min(source_width, source_height, target_width, target_height) <= 0:
        raise ValueError("image dimensions must be positive")
    scale = min(
        float(target_width) / float(source_width),
        float(target_height) / float(source_height),
    )
    new_width = int(round(source_width * scale))
    new_height = int(round(source_height * scale))
    pad_width = target_width - new_width
    pad_height = target_height - new_height
    pad_x = int(round(pad_width / 2.0 - 0.1))
    pad_y = int(round(pad_height / 2.0 - 0.1))
    return {
        "scale": scale,
        "new_width": new_width,
        "new_height": new_height,
        "pad_x": pad_x,
        "pad_y": pad_y,
        "pad_right": pad_width - pad_x,
        "pad_bottom": pad_height - pad_y,
        "target_width": target_width,
        "target_height": target_height,
    }


def letterbox_image(image, target_width, target_height):
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("input image must have shape HxWx3")
    geometry = compute_letterbox_geometry(
        source_width=image.shape[1],
        source_height=image.shape[0],
        target_width=target_width,
        target_height=target_height,
    )
    resized = cv2.resize(
        image,
        (geometry["new_width"], geometry["new_height"]),
        interpolation=cv2.INTER_LINEAR,
    )
    output = cv2.copyMakeBorder(
        resized,
        geometry["pad_y"],
        geometry["pad_bottom"],
        geometry["pad_x"],
        geometry["pad_right"],
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    if output.shape[:2] != (int(target_height), int(target_width)):
        raise RuntimeError("letterbox output has an unexpected shape")
    return output, geometry


def scale_boxes_to_original(
    boxes,
    geometry,
    source_width,
    source_height,
):
    result = np.asarray(boxes, dtype=np.float64).reshape((-1, 4)).copy()
    if len(result) == 0:
        return result
    scale = float(geometry["scale"])
    if scale <= 0.0:
        raise ValueError("letterbox scale must be positive")
    result[:, [0, 2]] = (
        result[:, [0, 2]] - float(geometry["pad_x"])
    ) / scale
    result[:, [1, 3]] = (
        result[:, [1, 3]] - float(geometry["pad_y"])
    ) / scale
    result[:, [0, 2]] = np.clip(result[:, [0, 2]], 0.0, float(source_width))
    result[:, [1, 3]] = np.clip(result[:, [1, 3]], 0.0, float(source_height))
    return result


def image_array_from_ros_bytes(data, width, height, step, encoding):
    if encoding not in ("bgr8", "rgb8"):
        raise ValueError("unsupported RGB encoding: %s" % encoding)
    width = int(width)
    height = int(height)
    step = int(step)
    row_bytes = width * 3
    if step < row_bytes:
        raise ValueError("RGB image step is smaller than packed row")
    raw = np.frombuffer(data, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError("RGB image data is shorter than height * step")
    image = raw[:required].reshape((height, step))[:, :row_bytes]
    image = image.reshape((height, width, 3)).copy()
    if encoding == "rgb8":
        image = image[:, :, ::-1].copy()
    return image


def _box_iou_one_to_many(box, boxes):
    if len(boxes) == 0:
        return np.empty((0,), dtype=np.float32)
    top_left = np.maximum(box[:2], boxes[:, :2])
    bottom_right = np.minimum(box[2:4], boxes[:, 2:4])
    intersection_size = np.maximum(bottom_right - top_left, 0.0)
    intersection = intersection_size[:, 0] * intersection_size[:, 1]
    box_area = max(0.0, float(box[2] - box[0])) * max(
        0.0,
        float(box[3] - box[1]),
    )
    boxes_area = np.maximum(boxes[:, 2] - boxes[:, 0], 0.0) * np.maximum(
        boxes[:, 3] - boxes[:, 1],
        0.0,
    )
    union = box_area + boxes_area - intersection
    return np.divide(
        intersection,
        union,
        out=np.zeros_like(intersection, dtype=np.float32),
        where=union > 0.0,
    )


def numpy_non_max_suppression(
    prediction,
    conf_threshold,
    iou_threshold,
    max_det,
):
    rows = np.asarray(prediction, dtype=np.float32)
    if rows.ndim == 3:
        if rows.shape[0] != 1:
            raise ValueError("only batch size 1 is supported")
        rows = rows[0]
    if rows.ndim != 2 or rows.shape[1] < 6:
        raise ValueError("prediction must have shape Nx(5 + class_count)")
    class_scores = rows[:, 5:]
    class_ids = np.argmax(class_scores, axis=1)
    confidences = rows[:, 4] * class_scores[
        np.arange(rows.shape[0]),
        class_ids,
    ]
    valid = np.isfinite(rows[:, :5]).all(axis=1)
    valid &= np.isfinite(confidences)
    valid &= confidences >= float(conf_threshold)
    rows = rows[valid]
    class_ids = class_ids[valid]
    confidences = confidences[valid]
    if rows.shape[0] == 0:
        return np.empty((0, 6), dtype=np.float32)

    boxes = np.empty((rows.shape[0], 4), dtype=np.float32)
    boxes[:, 0] = rows[:, 0] - rows[:, 2] / 2.0
    boxes[:, 1] = rows[:, 1] - rows[:, 3] / 2.0
    boxes[:, 2] = rows[:, 0] + rows[:, 2] / 2.0
    boxes[:, 3] = rows[:, 1] + rows[:, 3] / 2.0
    order = np.argsort(-confidences)
    keep = []
    while order.size > 0 and len(keep) < int(max_det):
        selected = int(order[0])
        keep.append(selected)
        remaining = order[1:]
        if remaining.size == 0:
            break
        same_class = class_ids[remaining] == class_ids[selected]
        overlap = _box_iou_one_to_many(boxes[selected], boxes[remaining])
        order = remaining[
            np.logical_not(
                np.logical_and(same_class, overlap > float(iou_threshold))
            )
        ]
    keep = np.asarray(keep, dtype=np.int64)
    return np.column_stack(
        (
            boxes[keep],
            confidences[keep],
            class_ids[keep].astype(np.float32),
        )
    ).astype(np.float32, copy=False)


def load_class_names(data_path):
    import yaml

    with open(data_path, "r") as handle:
        data = yaml.safe_load(handle)
    names = data.get("names")
    if isinstance(names, list):
        return names
    if isinstance(names, dict):
        return {
            int(index): str(name)
            for index, name in names.items()
        }
    raise ValueError("class data YAML must define names as a list or mapping")


class TensorRtRunner:
    CUDA_MEMCPY_HOST_TO_DEVICE = 1
    CUDA_MEMCPY_DEVICE_TO_HOST = 2

    def __init__(self, engine_path):
        import tensorrt as trt

        self._trt = trt
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        with open(engine_path, "rb") as handle:
            self._engine = self._runtime.deserialize_cuda_engine(handle.read())
        if self._engine is None:
            raise RuntimeError("failed to deserialize TensorRT engine")
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("failed to create TensorRT execution context")

        input_indices = [
            index
            for index in range(self._engine.num_bindings)
            if self._engine.binding_is_input(index)
        ]
        output_indices = [
            index
            for index in range(self._engine.num_bindings)
            if not self._engine.binding_is_input(index)
        ]
        if len(input_indices) != 1 or len(output_indices) != 1:
            raise RuntimeError(
                "TensorRT engine must have exactly one input and one output"
            )
        self.input_index = input_indices[0]
        self.output_index = output_indices[0]
        self.input_shape = tuple(
            int(value)
            for value in self._engine.get_binding_shape(self.input_index)
        )
        self.output_shape = tuple(
            int(value)
            for value in self._engine.get_binding_shape(self.output_index)
        )
        if any(value <= 0 for value in self.input_shape + self.output_shape):
            raise RuntimeError("TensorRT engine must use static binding shapes")
        self.input_dtype = np.dtype(
            trt.nptype(self._engine.get_binding_dtype(self.input_index))
        )
        self.output_dtype = np.dtype(
            trt.nptype(self._engine.get_binding_dtype(self.output_index))
        )
        self._cudart = self._load_cudart()
        self._device_input = self._cuda_malloc(
            int(np.prod(self.input_shape)) * self.input_dtype.itemsize
        )
        self._device_output = self._cuda_malloc(
            int(np.prod(self.output_shape)) * self.output_dtype.itemsize
        )
        self._bindings = [0] * self._engine.num_bindings
        self._bindings[self.input_index] = self._device_input.value
        self._bindings[self.output_index] = self._device_output.value
        self._output = np.empty(self.output_shape, dtype=self.output_dtype)

    def _load_cudart(self):
        candidates = [
            ctypes.util.find_library("cudart"),
            "libcudart.so",
            "libcudart.so.10.2",
        ]
        library = None
        for candidate in candidates:
            if not candidate:
                continue
            try:
                library = ctypes.CDLL(candidate)
                break
            except OSError:
                continue
        if library is None:
            raise RuntimeError("CUDA Runtime library could not be loaded")
        library.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        library.cudaMalloc.restype = ctypes.c_int
        library.cudaFree.argtypes = [ctypes.c_void_p]
        library.cudaFree.restype = ctypes.c_int
        library.cudaMemcpy.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
        ]
        library.cudaMemcpy.restype = ctypes.c_int
        library.cudaGetErrorString.argtypes = [ctypes.c_int]
        library.cudaGetErrorString.restype = ctypes.c_char_p
        return library

    def _check_cuda(self, status, operation):
        if int(status) == 0:
            return
        detail = self._cudart.cudaGetErrorString(int(status))
        message = detail.decode("utf-8") if detail else "unknown CUDA error"
        raise RuntimeError("%s failed: %s" % (operation, message))

    def _cuda_malloc(self, byte_count):
        pointer = ctypes.c_void_p()
        self._check_cuda(
            self._cudart.cudaMalloc(
                ctypes.byref(pointer),
                ctypes.c_size_t(byte_count),
            ),
            "cudaMalloc",
        )
        return pointer

    def infer(self, input_array):
        host_input = np.ascontiguousarray(input_array, dtype=self.input_dtype)
        if host_input.shape != self.input_shape:
            raise ValueError(
                "TensorRT input shape %s does not match engine shape %s"
                % (host_input.shape, self.input_shape)
            )
        self._check_cuda(
            self._cudart.cudaMemcpy(
                self._device_input,
                ctypes.c_void_p(host_input.ctypes.data),
                ctypes.c_size_t(host_input.nbytes),
                self.CUDA_MEMCPY_HOST_TO_DEVICE,
            ),
            "cudaMemcpy H2D",
        )
        if not self._context.execute_v2(self._bindings):
            raise RuntimeError("TensorRT execute_v2 returned false")
        self._check_cuda(
            self._cudart.cudaMemcpy(
                ctypes.c_void_p(self._output.ctypes.data),
                self._device_output,
                ctypes.c_size_t(self._output.nbytes),
                self.CUDA_MEMCPY_DEVICE_TO_HOST,
            ),
            "cudaMemcpy D2H",
        )
        return self._output

    def close(self):
        cudart = getattr(self, "_cudart", None)
        if cudart is None:
            return
        for attribute in ("_device_input", "_device_output"):
            pointer = getattr(self, attribute, None)
            if pointer and pointer.value:
                cudart.cudaFree(pointer)
                pointer.value = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
