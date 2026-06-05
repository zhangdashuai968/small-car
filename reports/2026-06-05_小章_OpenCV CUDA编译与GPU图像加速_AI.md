---
date: 2026-06-05
executor: 小章
experiment: E06
duration_h: 7
success: true
git_commit_before: "77ed4ca"
changed_files:
  - src/abot_project/vl_locate/launch/vl_locate.launch (image_zoomer 改用 resize_gpu.sh wrapper)
  - src/abot_project/vl_locate/scripts/resize_core.py (新增: GPU加速版resize)
  - src/abot_project/vl_locate/scripts/resize_gpu.sh (新增: bash wrapper绕开conda)
  - tools/cmake_cuda.py (新增: OpenCV 4.5.5 CUDA编译脚本)
  - src/abot_project/ZachLab_grasp/node/xarm_driver.py (修复: SerialRecvieCallback 空转)
car_state:
  bringup: OK
  navigation: N/A
  slam: N/A
  arm: OK
  battery_V: unknown
---

# AI 会话报告 — OpenCV CUDA 本地编译 + GPU 图像加速

## 目标

在 Jetson Nano 上本地编译 OpenCV 4.5.5 with CUDA，并将 resize.py 从 CPU 改为 GPU 加速，降低 vl_locate 链路 CPU 占用。

## 编译过程

### 环境
- JetPack 4.6 (L4T R32.6.1)
- CUDA 10.2, cuDNN 8.2.1
- OpenCV 4.5.5 + opencv_contrib 4.5.5
- GPU arch: sm_53 (Maxwell)

### 踩坑记录

| # | 错误 | 原因 | 修复 |
|---|------|------|------|
| 1 | `cudev module required` | 漏了 `OPENCV_EXTRA_MODULES_PATH` | 添加 contrib 路径 |
| 2 | 下载慢 (wechat_qrcode/xfeatures2d 模型) | GitHub raw 被墙 | 禁用不需要的 contrib 模块 |
| 3 | `IlmImf undefined reference` (63%) | 内置 OpenEXR 链接顺序 bug | `-D WITH_OPENEXR=OFF` |
| 4 | `opencv_perf_gapi` 链接失败 (100%) | ADE 库链接问题 | `-D BUILD_opencv_gapi=OFF` |
| 5 | Python2 绑定编译失败 | 重新 cmake 后生成头文件未更新 | 删掉 python_bindings_generator 目录重新 cmake |
| 6 | Python3 加载 4.1.1 系统旧版 | 系统路径优先级高于 /usr/local | `sudo rm /usr/lib/python3.6/dist-packages/cv2` |
| 7 | Python3 加载 4.11.0 conda版（无CUDA） | conda env 39 的 opencv-python 挡路 | 挪到 `/usr/local/lib/python3.6/dist-packages/` |

### 编译结果
- Python2: cv2 4.5.5 + CUDA ✅
- Python3: cv2 4.5.5 + CUDA ✅
- GPU resize 测试通过 ✅

## resize.py GPU 改造

### 踩坑

| # | 错误 | 原因 | 修复 |
|---|------|------|------|
| 1 | `OpenCV(4.11.0) no CUDA support` | conda 39 激活后 `python` 被劫持 | resize_gpu.sh wrapper 写死 `/usr/bin/python2` |
| 2 | `cv2.cuda.resize(src, dsize, dst)` → dst 为空 | Python 绑定 API 不同 | 改用返回值模式 `dst = cv2.cuda.resize(src, dsize)` |
| 3 | `TypeError: int() ... not 'NoneType'` | ROI numpy slice 非连续内存，GPU upload 失败 | 添加 `.copy()` 确保连续内存 |
| 4 | shebang `#!/usr/bin/python2` 无效 | ROS Melodic 无视 shebang，用 PATH 里的 `python` | 改用 bash wrapper `resize_gpu.sh` → launch 文件 `type="resize_gpu.sh"` |

### 最终方案

```
vl_locate.launch
  ├── gemini (camera_node)         ← 15Hz MJPEG
  ├── resize_gpu.sh → resize_core.py  ← /usr/bin/python2 + GPU cv2.cuda.resize
  └── server.py                    ← conda Python 3.9 + OpenAI SDK
```

Wrapper (`resize_gpu.sh`):
```bash
#!/bin/bash
exec /usr/bin/python2 /home/abot/catkin_ws/src/abot_project/vl_locate/scripts/resize_core.py "$@"
```

GPU 核心代码:
```python
gpu_src = cv2.cuda_GpuMat()
gpu_src.upload(roi)  # .copy() 确保连续内存
gpu_dst = cv2.cuda.resize(gpu_src, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
zoomed_roi = gpu_dst.download()
```

## 性能对比

### vl_locate 链路 CPU

| 进程 | 原始 (30Hz CPU) | 优化后 (15Hz GPU) | 降幅 |
|------|----------------|-------------------|------|
| camera_node | 24.7% | 13.0% | -47% |
| resize.py | 21% (CPU) | **8.1% (GPU)** | **-61%** |
| server.py | 19.3% | 3.5% | 空闲 |
| **vl_locate 合计** | **~65%** | **~25%** | **-62%** |

### 全栈负载 (bringup + vl_locate + grasp)

| 阶段 | load | 说明 |
|------|------|------|
| 原始 (gg.py 98%) | 3.12 | 机械臂驱动空转 |
| 修 gg.py + 去 relay | 1.37 | 基础修复 |
| **GPU resize + 15Hz** | **0.56** | 最终状态 |

### 全车进程分布 (load 0.56)

```
bringup:  22.3% (arduino 16.5 + ekf 2.4 + others)
vl_locate: 24.5% (camera 13.0 + resize 8.0 + server 3.5)
grasp:     3.9% (gg 2.7 + launch 1.2)
系统:     13.7% (Xorg 5.8 + NoMachine 5.9 + GPU 1.0)
```

## 文件变更

### 新增文件
- `tools/cmake_cuda.py` — OpenCV 4.5.5 CUDA 编译脚本（一键）
- `tools/resize_gpu.sh` — bash wrapper 绕开 conda Python 劫持
- `src/abot_project/vl_locate/scripts/resize_core.py` — GPU 加速版 resize
- `src/abot_project/vl_locate/scripts/resize.py.bak2` — 原始 CPU 版备份

### 修改文件
- `vl_locate.launch` — `type="resize_gpu.sh"` 替代 `type="resize.py"`
- `xarm_driver.py` — `SerialRecvieCallback` 空转修复 (`time.sleep(0.02)`)
- `resize.py` (Nano 上) → 已重命名为 `resize_core.py`

## 启动命令（当前）

```bash
# 终端1: 底盘
roslaunch abot bringup.launch

# 终端2: 定位
roslaunch abot localize.launch

# 终端3: VLM 链路 (需要 conda 39 给 server.py)
conda activate 39
roslaunch vl_locate vl_locate.launch

# 终端4: 机械臂
roslaunch ZachLab_grasp grasp.launch
```

## 清理

```bash
# 编译残留 (省 ~3GB)
rm -rf ~/opencv-4.5.5 ~/opencv_contrib-4.5.5 ~/opencv-4.5.5.zip ~/opencv_contrib-4.5.5.zip
```
