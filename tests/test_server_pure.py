# -*- coding: utf-8 -*-
"""vl_locate/scripts/server.py（VLM 检测服务，生产代码）中的纯方法。

- VLM.normalize_bbox: VLM 返回 bbox 的 0-1000 归一化 + 像素 clamp。
  server.py 模块级 import rospy，不能整载；提取后 rospy 以 stub 注入。
- VLM.extract_json_result: 从 VLM 任意格式文本中容错提取 JSON。

调用方式：提取后是普通函数，self 传 None（两方法均不引用 self 属性）。
"""
import json

import pytest

from conftest import extract_func

REL = "src/abot_project/vl_locate/scripts/server.py"
normalize_bbox = extract_func(REL, "normalize_bbox")
extract_json_result = extract_func(REL, "extract_json_result")


def bbox(b, w=640, h=480):
    return normalize_bbox(None, b, w, h)  # None = 原方法的 self


class TestNormalizeBbox:
    def test_plain_pixel_bbox_untouched(self):
        assert bbox([10, 20, 100, 200]) == [10, 20, 100, 200]

    def test_float_rounded_to_int(self):
        assert bbox([10.4, 20.6, 100.4, 200.6]) == [10, 21, 100, 201]

    def test_x_over_width_triggers_0_1000(self):
        # 行为锁：触发后四个坐标整体按 0-1000 重映射（不是只缩放超界轴）
        # x: 512/576; y: 100→48, 200→96
        assert bbox([800, 100, 900, 200]) == [512, 48, 576, 96]

    def test_y_over_height_triggers_0_1000(self):
        # 同上全轴重映射: y: 384/432; x: 100→64
        assert bbox([0, 800, 100, 900]) == [0, 384, 64, 432]

    def test_both_axes_negative_and_clamp(self):
        # 触发归一化: x∈{-32,448} y∈{-24,240}，负值 clamp 到 0
        assert bbox([-50, -50, 700, 500]) == [0, 0, 448, 240]

    def test_exactly_img_size_does_not_trigger(self):
        # 触发条件是严格大于；== img_w 只走 clamp 到 img_w-1
        assert bbox([640, 0, 640, 100]) == [639, 0, 639, 100]

    def test_min_max_order_preserved(self):
        # 行为锁：不排序。x_min > x_max 的畸形输入原样保留（仅 clamp/取整）
        assert bbox([100, 20, 10, 200]) == [100, 20, 10, 200]


class TestExtractJsonResult:
    def test_plain_dict(self):
        assert extract_json_result(None, '{"label": "绿方块", "bbox_2d": [1,2,3,4]}') == {
            "label": "绿方块", "bbox_2d": [1, 2, 3, 4]
        }

    def test_markdown_fenced(self):
        assert extract_json_result(None, '```json\n{"a": 1}\n```') == {"a": 1}

    def test_surrounding_noise(self):
        assert extract_json_result(None, '检测到目标：{"a": 1} 请查收') == {"a": 1}

    def test_plain_list(self):
        assert extract_json_result(None, '[{"a": 1}, {"b": 2}]') == [
            {"a": 1}, {"b": 2}
        ]

    def test_first_candidate_wins_even_list_before_dict(self):
        # 行为锁：按出现顺序返回第一个 dict 或非空 list（先到先得，
        # 并非 dict 全局优先）
        assert extract_json_result(None, '[1, 2] {"a": 1}') == [1, 2]

    def test_bare_scalar_raises(self):
        # 行为锁：只认 [ / { 起解，裸标量从不进 candidates —— 直接 raise
        with pytest.raises(json.JSONDecodeError):
            extract_json_result(None, "123")

    def test_no_json_raises(self):
        with pytest.raises(json.JSONDecodeError):
            extract_json_result(None, "没有任何 JSON 内容")
