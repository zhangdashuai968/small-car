# -*- coding: utf-8 -*-
"""scripts/pipeline_sim.py —— VLM 抓取坐标链路数学模拟（zoomed 空间标定）。

两件事：
1. 锁 sim 自身数学：ROI→zoomed 变换、bbox 归一化、像素→arm 两点标定插值；
2. 互锁：sim 复刻的 normalize_bbox 必须与 server.py 生产原版逐值一致——
   将来任何一边改动而另一边没跟，这里先红。
"""
import pytest

from conftest import extract_func

_server_normalize_bbox = extract_func(
    "src/abot_project/vl_locate/scripts/server.py", "normalize_bbox"
)


class TestSimConstants:
    """锁链路常量：SCALE 由 ROI 尺寸推导，START_Y 是黑边居中偏移。"""

    def test_scale(self, pipeline_sim):
        assert pipeline_sim.SCALE == pytest.approx(640 / 205)  # min(640/205, 480/120)

    def test_zoomed_canvas(self, pipeline_sim):
        assert (pipeline_sim.NEW_W, pipeline_sim.NEW_H) == (640, 374)
        assert (pipeline_sim.START_X, pipeline_sim.START_Y) == (0, 53)


class TestOrigToZoomed:
    def test_roi_top_left(self, pipeline_sim):
        assert pipeline_sim.orig_to_zoomed(275, 254) == pytest.approx((0.0, 53.0))

    def test_roi_bottom_right(self, pipeline_sim):
        zx, zy = pipeline_sim.orig_to_zoomed(480, 374)
        assert zx == pytest.approx(640.0)
        assert zy == pytest.approx(120 * pipeline_sim.SCALE + 53)

    def test_sim_main_scenario_point(self, pipeline_sim):
        # pipeline_sim 自身 __main__ 场景注释锚点 (370,310) -> (297,228)
        zx, zy = pipeline_sim.orig_to_zoomed(370, 310)
        assert zx == pytest.approx(296.6, abs=0.1)
        assert zy == pytest.approx(227.8, abs=0.1)


class TestSimNormalizeBbox:
    def test_not_triggered(self, pipeline_sim):
        out, triggered = pipeline_sim.normalize_bbox([10, 20, 100, 200])
        assert (out, triggered) == ([10, 20, 100, 200], False)

    def test_triggered_0_1000(self, pipeline_sim):
        # 与 server 版同语义：触发后四轴整体重映射（y: 100→48, 200→96）
        out, triggered = pipeline_sim.normalize_bbox([800, 100, 900, 200])
        assert (out, triggered) == ([512, 48, 576, 96], True)

    @pytest.mark.parametrize(
        "raw",
        [
            [10, 20, 100, 200],
            [800, 100, 900, 200],
            [0, 800, 100, 900],
            [-50, -50, 700, 500],
            [640, 0, 640, 100],
            [10.4, 20.6, 100.4, 200.6],
        ],
    )
    def test_interlock_with_server_version(self, pipeline_sim, raw):
        # 互锁：sim 复刻版 vs server.py 生产原版，同一输入必须同一输出
        sim_out, _ = pipeline_sim.normalize_bbox(raw)
        server_out = _server_normalize_bbox(None, raw, 640, 480)
        assert sim_out == server_out


class TestPixelToArm:
    def test_calibration_endpoint_1(self, pipeline_sim):
        # 标定点回代：cali_1_im(48,384) ↔ arm(120,75)
        assert pipeline_sim.pixel_to_arm(48, 384) == (120, 75)

    def test_calibration_endpoint_2(self, pipeline_sim):
        # 标定点回代：cali_2_im(590,111) ↔ arm(223,-120)
        assert pipeline_sim.pixel_to_arm(590, 111) == (223, -120)


class TestBboxCenter:
    def test_center(self, pipeline_sim):
        assert pipeline_sim.bbox_center([10, 20, 110, 220]) == (60, 120)

    def test_int_truncation_on_half(self, pipeline_sim):
        # 行为锁：int() 截断，.5 向零取整（cx 60.5->60, cy 120.5->120）
        assert pipeline_sim.bbox_center([10, 20, 111, 221]) == (60, 120)

    def test_coco_center_is_identity(self, pipeline_sim):
        assert pipeline_sim.bbox_center_coco(7, 8, 9, 10) == (7, 8)
