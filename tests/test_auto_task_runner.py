# -*- coding: utf-8 -*-
"""auto_task_runner.py 纯函数：_norm（yaw 归一化 atan2 版）、_clamp、load_waypoints。

锁当前行为：包括反常输入（如 _clamp 的 lo>hi）的现状语义——将来有人"顺手改"
这些小工具时，这里会先红，逼他确认是不是真要改语义。
"""
import math

import pytest
import yaml

from conftest import extract_func

_norm = extract_func("scripts/auto_task_runner.py", "_norm")
_clamp = extract_func("scripts/auto_task_runner.py", "_clamp")
_load_waypoints = extract_func(
    "scripts/auto_task_runner.py", "load_waypoints", inject={"yaml": yaml}
)


class TestNorm:
    def test_zero(self):
        assert _norm(0.0) == 0.0

    def test_in_range_untouched(self):
        for a in (0.5, -0.5, 1.0, -1.0, 2.9, -2.9):
            assert _norm(a) == pytest.approx(a)

    def test_pi_boundaries(self):
        assert _norm(math.pi) == pytest.approx(math.pi)
        assert _norm(-math.pi) == pytest.approx(-math.pi)

    def test_wrap(self):
        assert _norm(2 * math.pi + 0.5) == pytest.approx(0.5)
        assert _norm(-2 * math.pi - 0.1) == pytest.approx(-0.1)
        assert _norm(math.pi + 0.1) == pytest.approx(0.1 - math.pi)
        assert _norm(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)

    def test_big_angle_o1(self):
        # atan2 版无循环，超大角度一次到位（这正是它优于 while 循环版之处）
        assert _norm(10 * math.pi + 0.3) == pytest.approx(0.3, abs=1e-12)

    def test_property_range_and_period(self):
        for i in range(-40, 41):
            a = i * math.pi / 8
            r = _norm(a)
            assert -math.pi - 1e-12 <= r <= math.pi + 1e-12
            # 周期性按"角度差归一化后为 0"断言：±π 端点处结果符号
            # 取决于 sin 的浮点残差（-5π→-π 而 9π→+π），不能直接比相等
            assert _norm(r - _norm(a + 2 * math.pi * 7)) == pytest.approx(0.0, abs=1e-9)


class TestClamp:
    def test_interior(self):
        assert _clamp(5, 0, 10) == 5

    def test_boundaries(self):
        assert _clamp(0, 0, 10) == 0
        assert _clamp(10, 0, 10) == 10

    def test_out_of_range(self):
        assert _clamp(-1, 0, 10) == 0
        assert _clamp(11, 0, 10) == 10

    def test_float(self):
        assert _clamp(3.7, -1.0, 1.0) == 1.0

    def test_inverted_bounds_current_behavior(self):
        # 行为锁：lo>hi 时当前实现返回 lo（max 先吃掉 min 结果）。
        # 若某天这里需要改成 raise，先改测试——这就是回归网存在的意义。
        assert _clamp(5, 10, 0) == 10


class TestLoadWaypoints:
    def _write(self, tmp_path, text):
        f = tmp_path / "wps.yaml"
        f.write_text(text, encoding="utf-8")
        return str(f)

    def test_roundtrip(self, tmp_path):
        f = self._write(
            tmp_path,
            "waypoints:\n"
            "  - name: p1\n    x: 1.0\n    y: 2.0\n    yaw: 0.5\n"
            "  - name: p2\n    x: -3.0\n    y: 4.0\n    yaw: -1.5\n",
        )
        wps = _load_waypoints(f)
        assert wps == [
            {"name": "p1", "x": 1.0, "y": 2.0, "yaw": 0.5},
            {"name": "p2", "x": -3.0, "y": 4.0, "yaw": -1.5},
        ]

    def test_missing_waypoints_key_raises(self, tmp_path):
        # 行为锁：当前缺键直接 KeyError，无兜底
        f = self._write(tmp_path, "other: 1\n")
        with pytest.raises(KeyError):
            _load_waypoints(f)
