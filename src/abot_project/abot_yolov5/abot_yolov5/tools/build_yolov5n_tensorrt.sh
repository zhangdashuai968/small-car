#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
YOLO_ROOT="${PACKAGE_ROOT}/yolov5"
WEIGHT_PATH="${1:-${PACKAGE_ROOT}/weights/yolov5n.pt}"
OUTPUT_DIR="${2:-${HOME}/.cache/abot/tensorrt}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
ENGINE_PATH="${OUTPUT_DIR}/yolov5n_1x3x384x640_fp16_trt8.engine"
CACHED_WEIGHT="${OUTPUT_DIR}/yolov5n.pt"

test -f "${WEIGHT_PATH}"
test -x "${TRTEXEC}"
"${PYTHON_BIN}" -c "import onnx, torch; assert torch.cuda.is_available()"
mkdir -p "${OUTPUT_DIR}"
cp -f "${WEIGHT_PATH}" "${CACHED_WEIGHT}"

cd "${YOLO_ROOT}"
"${PYTHON_BIN}" export.py \
  --weights "${CACHED_WEIGHT}" \
  --imgsz 384 640 \
  --batch-size 1 \
  --device 0 \
  --opset 13 \
  --include onnx

ONNX_PATH="${CACHED_WEIGHT%.pt}.onnx"
test -f "${ONNX_PATH}"
"${PYTHON_BIN}" "${PACKAGE_ROOT}/tools/prune_yolov5_onnx_outputs.py" \
  "${ONNX_PATH}"
"${TRTEXEC}" \
  --onnx="${ONNX_PATH}" \
  --saveEngine="${ENGINE_PATH}" \
  --fp16 \
  --workspace=512

sha256sum "${CACHED_WEIGHT}" "${ONNX_PATH}" "${ENGINE_PATH}"
