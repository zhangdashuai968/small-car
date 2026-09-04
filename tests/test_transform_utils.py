# -*- coding: utf-8 -*-
"""transform_utils.normalize_angle —— yaw 归一化（while 循环版）。

仓库存在两份同名双胞胎文件（abot_nav 包版 / abot_ai_speech 节点版），
参数化同时锁两份行为，并用 AST 断言两份源码一致——防将来漂移分叉。
"""
import ast
import math
import pathlib

import pytest

from conftest import ROOT, extract_func

TWIN_PATHS = [
    "src/abot_project/abot_nav/src/abot_nav/transform_utils.py",
    "src/abot_project/abot_ai_speech/nodes/transform_utils.py",
]


def _make(rel):
    return extract_func(rel, "normalize_angle", inject={"pi": math.pi})


@pytest.fixture(scope="module", params=TWIN_PATHS, ids=["nav_pkg", "speech_nodes"])
def norm(request):
    return _make(request.param)


def test_zero(norm):
    assert norm(0.0) == 0.0


def test_in_range_untouched(norm):
    for a in (0.5, -0.5, 1.0, -1.0, 2.9, -2.9):
        assert norm(a) == pytest.approx(a)


def test_pi_boundaries(norm):
    # 边界本身不动：π 不 > π、-π 不 < -π，原样返回
    assert norm(math.pi) == math.pi
    assert norm(-math.pi) == -math.pi


def test_wrap(norm):
    assert norm(3 * math.pi) == pytest.approx(math.pi)
    assert norm(-3 * math.pi) == pytest.approx(-math.pi)
    assert norm(math.pi + 0.1) == pytest.approx(0.1 - math.pi)
    assert norm(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert norm(2 * math.pi) == pytest.approx(0.0, abs=1e-12)


def test_property_range(norm):
    # 多圈采样，结果必须落在 [-π, π]
    for i in range(-40, 41):
        a = i * math.pi / 8
        assert -math.pi <= norm(a) <= math.pi


def test_equivalent_to_atan2_version(norm):
    # 与 auto_task_runner._norm（atan2 版）跨实现一致性：
    # 同为"归一化到 [-π,π]"，两种实现不许给出不同答案
    _norm = extract_func("scripts/auto_task_runner.py", "_norm")
    for i in range(-40, 41):
        a = i * math.pi / 8
        assert norm(a) == pytest.approx(_norm(a))


def test_twins_are_identical_source():
    # 双胞胎锁：两份文件的 normalize_angle 节点必须逐字一致。
    # 将来只改其中一份时这里会红——要么同步两份，要么明确分叉并拆掉本测试。
    dumps = []
    for rel in TWIN_PATHS:
        src = (ROOT / rel).read_text(encoding="utf-8", errors="replace")
        node = next(
            n for n in ast.parse(src).body
            if isinstance(n, ast.FunctionDef) and n.name == "normalize_angle"
        )
        dumps.append(ast.dump(node, include_attributes=False))
    assert dumps[0] == dumps[1]


def test_quat_to_angle_is_pykdl_locked_out():
    # 记录性断言：同文件的 quat_to_angle 依赖 PyKDL（本地无 ROS），
    # 有意不纳入离线测试。若它被改写成纯实现，可移入测试。
    src = (ROOT / TWIN_PATHS[0]).read_text(encoding="utf-8", errors="replace")
    assert "PyKDL" in src
