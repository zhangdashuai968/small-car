#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ABOT M1 视觉抓取全链路数学模拟
模拟 resize_core -> VLM bbox -> normalize_bbox -> 2点标定 -> arm坐标
注入故障场景, 对比 arm 预期坐标 vs 实际坐标

关键前提 (来自上轮调试):
  标定是用 zoomed 图像做的, cali_1_im/cali_2_im 是 zoomed 图像上的像素坐标
  所以 pixel_to_arm 的输入必须是 zoomed 空间坐标, 不需要 unzoom
"""

import numpy as np

# ======================== 全局常量 ========================
IMG_W, IMG_H = 640, 480

# ---- resize_core.py: ROI裁剪 + GPU 放大 ----
ROI_X1 = int(IMG_W * 0.43)   # 275
ROI_Y1 = int(IMG_H * 0.53)   # 254
ROI_X2 = int(IMG_W * 0.75)   # 480
ROI_Y2 = int(IMG_H * 0.78)   # 374
ROI_W  = ROI_X2 - ROI_X1     # 205
ROI_H  = ROI_Y2 - ROI_Y1     # 120
SCALE  = min(IMG_W / ROI_W, IMG_H / ROI_H)  # 3.122
NEW_W  = int(ROI_W * SCALE)  # 640
NEW_H  = int(ROI_H * SCALE)  # 374
START_X = (IMG_W - NEW_W) // 2  # 0
START_Y = (IMG_H - NEW_H) // 2  # 53

# ---- gg.py 标定参数 (在 zoomed 图像上标定) ----
cali_1_im = [48, 384]   # zoomed图像左下角像素
cali_1_mc = [-75, 110]  # 对应arm坐标
cali_2_im = [590, 111]  # zoomed图像右上角像素
cali_2_mc = [120, 213]  # 对应arm坐标

grasp_offset_x = 10
grasp_offset_y = 0


# ======================== 核心变换函数 ========================

def orig_to_zoomed(orig_x, orig_y):
    """原始相机图像坐标 -> zoomed图像坐标 (resize_core正向)"""
    roi_x = orig_x - ROI_X1
    roi_y = orig_y - ROI_Y1
    zoomed_x = roi_x * SCALE + START_X
    zoomed_y = roi_y * SCALE + START_Y
    return zoomed_x, zoomed_y


def normalize_bbox(bbox_2d, img_w=IMG_W, img_h=IMG_H):
    """
    复刻 server.py normalize_bbox
    仅当坐标超出图像尺寸时触发 0-1000 归一化
    返回: (归一化后bbox, 是否触发)
    """
    x_min, y_min, x_max, y_max = map(float, bbox_2d)
    triggered = False
    if max(x_min, x_max) > img_w or max(y_min, y_max) > img_h:
        triggered = True
        x_min = x_min / 1000.0 * img_w
        x_max = x_max / 1000.0 * img_w
        y_min = y_min / 1000.0 * img_h
        y_max = y_max / 1000.0 * img_h

    x_min = max(0, min(img_w - 1, int(round(x_min))))
    x_max = max(0, min(img_w - 1, int(round(x_max))))
    y_min = max(0, min(img_h - 1, int(round(y_min))))
    y_max = max(0, min(img_h - 1, int(round(y_max))))
    return [x_min, y_min, x_max, y_max], triggered


def pixel_to_arm(px_x, px_y):
    """
    gg.py camera_point_to_grasp_pose
    输入: zoomed 图像上的像素坐标 (标定在此空间)
    输出: arm (x, y) mm
    """
    X_cali_im = [cali_1_im[0], cali_2_im[0]]
    X_cali_mc = [cali_1_mc[0], cali_2_mc[0]]
    Y_cali_im = [cali_2_im[1], cali_1_im[1]]   # 大->小
    Y_cali_mc = [cali_2_mc[1], cali_1_mc[1]]

    X_arm = int(np.interp(px_x, X_cali_im, X_cali_mc))
    Y_arm = int(np.interp(px_y, Y_cali_im, Y_cali_mc))

    arm_x = Y_arm + grasp_offset_x
    arm_y = -X_arm + grasp_offset_y
    return arm_x, arm_y


def bbox_center(bbox):
    """bbox [x1,y1,x2,y2] -> center (cx, cy)"""
    return int((bbox[0] + bbox[2]) / 2), int((bbox[1] + bbox[3]) / 2)


def bbox_center_coco(cx, cy, w, h):
    """COCO [cx,cy,w,h] -> center —— 本身就是中心, 直接取前两个"""
    return int(cx), int(cy)


# ======================== 单场景测试 ========================

def test(scenario, object_orig, vlm_bbox_raw):
    """
    scenario: 场景描述
    object_orig: 物体在原始相机图像中的真实中心 (x, y)
    vlm_bbox_raw: VLM返回的原始bbox

    计算两条路径的arm坐标:
      Truth:  orig -> zoomed -> pixel_to_arm (标定在zoomed空间)
      VLM:    vlm_bbox -> normalize -> center -> pixel_to_arm
    """
    obj_zx, obj_zy = orig_to_zoomed(object_orig[0], object_orig[1])
    truth_arm = pixel_to_arm(obj_zx, obj_zy)

    bbox_norm, triggered = normalize_bbox(vlm_bbox_raw)
    vlm_cx, vlm_cy = bbox_center(bbox_norm)
    vlm_arm = pixel_to_arm(vlm_cx, vlm_cy)

    dx = vlm_arm[0] - truth_arm[0]
    dy = vlm_arm[1] - truth_arm[1]
    dist = np.sqrt(dx**2 + dy**2)

    # 判断: arm坐标系 x=前后 y=左右(取反), 偏差>30mm抓不到
    if dist > 40:
        grade = "[CRIT]"
    elif dist > 20:
        grade = "[WARN]"
    else:
        grade = "[OK]"

    print(f"\n{'─'*55}")
    print(f"{grade} {scenario}")
    print(f"{'─'*55}")
    print(f"  物体原始像素: {object_orig}  ->  zoomed: ({obj_zx:.0f},{obj_zy:.0f})")
    print(f"  Truth arm:    x={truth_arm[0]:4d}, y={truth_arm[1]:4d}")
    print(f"  VLM bbox_raw: {vlm_bbox_raw}")
    print(f"  normalize触发: {triggered}")
    print(f"  VLM center:   ({vlm_cx},{vlm_cy})  (zoomed空间)")
    print(f"  VLM arm:      x={vlm_arm[0]:4d}, y={vlm_arm[1]:4d}")
    print(f"  误差: dx={dx:4d}mm, dy={dy:4d}mm, |d|={dist:.0f}mm")
    print(f"  解读: dx=前后(dx>0偏前=靠近车头), dy=左右(dy>0偏左, dy<0偏右)")

    return dist


# ======================== 主测试 ========================

if __name__ == '__main__':
    print("=" * 55)
    print("ABOT 视觉抓取全链路模拟 (标定=zoomed空间)")
    print("=" * 55)

    # 物体真实位置 (原始相机图像中)
    obj = (370, 310)

    # ---------- 场景1: 理想情况 ----------
    zx, zy = orig_to_zoomed(obj[0], obj[1])
    perfect_bbox = [zx-30, zy-30, zx+30, zy+30]
    test("S1 理想: VLM返回正确zoomed像素bbox",
         obj, perfect_bbox)

    # ---------- 场景2: normalize_bbox 盲区 ----------
    # VLM返回0-1000坐标, 但值全在640/480内 -> 不触发归一化
    # 物体在zoomed中心 (297,228) -> 0-1000映射: x=297/640*1000=464, y=228/480*1000=475
    s2_cx = zx / IMG_W * 1000  # 464
    s2_cy = zy / IMG_H * 1000  # 475
    s2_bbox_1000 = [s2_cx-50, s2_cy-50, s2_cx+50, s2_cy+50]
    test("S2 CRIT: VLM 0-1000坐标但值<640 -> normalize盲区",
         obj, s2_bbox_1000)

    # ---------- 场景3: VLM返回COCO格式 ----------
    coco_bbox = [zx, zy, 60, 60]  # cx, cy, w, h 被当成 x1,y1,x2,y2
    test("S3 CRIT: VLM返回[cx,cy,w,h] COCO格式 被误当[x1,y1,x2,y2]",
         obj, coco_bbox)

    # ---------- 场景4: VLM检测框整体右偏 ----------
    right_bias_bbox = [zx+20, zy-30, zx+80, zy+30]  # 中心右偏 20px
    test("S4: VLM检测框右偏20px (模型幻觉)",
         obj, right_bias_bbox)

    # ---------- 场景5: qwen3.7-plus 0-1000坐标 完全不同位置 ----------
    qwen37_bbox_1000 = [520, 200, 580, 260]  # 0-1000坐标
    test("S5 CRIT: qwen3.7-plus返回0-1000坐标 无触发归一化",
         obj, qwen37_bbox_1000)

    # ---------- 场景6: 标定参数漂移 ----------
    # 直接模拟 arm 坐标整体偏移的后果
    zx6, zy6 = orig_to_zoomed(obj[0], obj[1])
    bbox6 = [zx6-30, zy6-30, zx6+30, zy6+30]
    bbox_norm6, _ = normalize_bbox(bbox6)
    cx6, cy6 = bbox_center(bbox_norm6)
    # 用偏移后的标定点手动算
    bad_cali_1_mc = [-65, 115]
    bad_cali_2_mc = [130, 218]
    X_arm6 = int(np.interp(cx6, [cali_1_im[0], cali_2_im[0]], [bad_cali_1_mc[0], bad_cali_2_mc[0]]))
    Y_arm6 = int(np.interp(cy6, [cali_2_im[1], cali_1_im[1]], [bad_cali_2_mc[1], bad_cali_1_mc[1]]))
    bad_arm_x = Y_arm6 + grasp_offset_x
    bad_arm_y = -X_arm6 + grasp_offset_y
    truth6 = pixel_to_arm(zx6, zy6)
    dx6 = bad_arm_x - truth6[0]
    dy6 = bad_arm_y - truth6[1]
    dist6 = np.sqrt(dx6**2 + dy6**2)
    print(f"\n{'─'*55}")
    print(f"[WARN] S6: 标定漂移 +10mm X, +5mm Y (相机松动)")
    print(f"{'─'*55}")
    print(f"  Truth arm:    x={truth6[0]:4d}, y={truth6[1]:4d}")
    print(f"  Bad cali arm: x={bad_arm_x:4d}, y={bad_arm_y:4d}")
    print(f"  误差: dx={dx6:4d}mm, dy={dy6:4d}mm, |d|={dist6:.0f}mm")

    # ========== 诊断 ==========
    print(f"\n{'='*55}")
    print("DIAGNOSIS")
    print(f"{'='*55}")
    print("""
  "只抓右边" = arm y 始终偏负 (dy < 0, 即 arm 偏右)

  最可能根因排名:
  1. S5/S2: normalize_bbox 盲区
     qwen3.7-plus 返回 0-1000 坐标, 值在 640/480 内, 不触发归一化
     -> bbox 整体放大 1.56x, 物体越靠图像右侧, 偏差越大

  2. S3: 模型输出格式切换
     qwen-vl-max -> qwen3.7-plus 后 bbox 语义可能变为 [cx,cy,w,h]

  3. S4: 模型在 zoomed 图像上的系统性检测偏差
     zoomed 图像只有中间一条(上下黑边), 模型可能误解位置

  实车验证方法:
     rostopic echo /Pose  # 看 x_center, y_center
     scp vlm_now.jpg 到本地看画的框是否对准物体
     如果框没对准 -> VLM问题(归一化/格式)
     如果框对准了但抓不准 -> 标定问题
""")
