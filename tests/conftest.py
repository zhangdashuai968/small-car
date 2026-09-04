# -*- coding: utf-8 -*-
"""small-car 纯逻辑函数测试基建。

被测源码模块大多 import rospy/tf/PyKDL（本地 Windows 无 ROS），不能整模块加载；
本文件用 AST 按名提取函数（含类方法）源码，exec 进干净命名空间——零改源码。
类方法提取后是普通函数，self 形参传 None 即可（所选方法均不引用 self 属性）。

跑法（仓库根）：python -m pytest tests/ -v
"""
import ast
import importlib.util
import json
import math
import pathlib
import types

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent


def _stub_rospy():
    """仅提供被提取函数用到的日志接口，让 normalize_bbox 等可离线运行。"""
    m = types.ModuleType("rospy")
    m.loginfo = m.logwarn = m.logerr = lambda *a, **k: None
    return m


def extract_func(rel_path, name, inject=None):
    """从 rel_path 按名提取函数/类方法源码，独立执行后返回该函数对象。

    默认注入 math / json / rospy(stub)；额外依赖经 inject 补充。
    假设 name 在文件内唯一。
    """
    path = ROOT / rel_path
    src = path.read_text(encoding="utf-8", errors="replace")
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            ns = {"__name__": f"extracted.{name}"}
            ns.update({"math": math, "json": json, "rospy": _stub_rospy()})
            if inject:
                ns.update(inject)
            exec(compile(ast.get_source_segment(src, node), str(path), "exec"), ns)
            return ns[name]
    raise LookupError(f"{name!r} not found in {rel_path}")


@pytest.fixture(scope="session")
def pipeline_sim():
    """pipeline_sim.py 仅依赖 numpy，可整模块加载（其 __main__ 场景块不会执行）。"""
    path = ROOT / "scripts" / "pipeline_sim.py"
    spec = importlib.util.spec_from_file_location("pipeline_sim_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
