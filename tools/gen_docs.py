# -*- coding: utf-8 -*-
"""合并 small-car 团队必读 md -> 单文件 HTML（深色主题，带目录跳转）。

用法（在仓库任意位置）：
    pip install markdown
    python tools/gen_docs.py
输出：仓库根目录 团队必读文档汇总.html
"""
import os
import html
import datetime
import markdown

# 仓库根 = 本文件(tools/gen_docs.py)的上上级目录, 路径相对化, 任何 clone 都能跑
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "团队必读文档汇总.html")

# (锚点, 显示标题, 相对路径)
DOCS = [
    ("quickstart",  "快速启动命令速查（建图 / 存图 / 定位）",   "启动命令速查.md"),
    ("rules",       "比赛规则（最终目标 · 硬约束）",            "比赛规则.md"),
    ("overview",    "项目总览 README",                         "README.md"),
    ("guide",       "小车实验指南（技术教育详解）",              "src/小车实验指南技术教育详解(2).md"),
    ("trouble",     "故障排查 TROUBLESHOOTING",                 "TROUBLESHOOTING.md"),
    ("log0526",     "调试日志 2026-05-26 · SLAM建图与局部规划器解耦", "logs/2026-05-26_SLAM建图与局部规划器解耦.md"),
    ("tuning",      "调参极限分析报告",                          "reports/调参极限分析报告.md"),
    ("automap",     "自动建图指南 auto_map_guide",               "src/abot_project/abot/docs/auto_map_guide.md"),
    ("autoreadme",  "abot_project 自动化说明 README_AUTO",       "src/abot_project/README_AUTO.md"),
    ("abotreadme",  "abot 包 README",                           "src/abot_project/abot/README.md"),
]

MD_EXT = ["tables", "fenced_code", "sane_lists", "attr_list", "toc"]


def render_md(path):
    with open(path, encoding="utf-8") as f:
        text = f.read()
    md = markdown.Markdown(extensions=MD_EXT, output_format="html5")
    return md.convert(text)


CSS = """
:root{--bg:#0f1115;--card:#181b22;--card2:#1f232c;--ink:#e7eaf0;--mut:#9aa3b2;
--line:#2b313c;--accent:#5b9dff;--ok:#39c08a;--warn:#e0a838;--danger:#e0556b;--code:#0b0d11;}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
font-family:"Segoe UI","Microsoft YaHei",system-ui,sans-serif;line-height:1.65;font-size:15px}
.wrap{max-width:1000px;margin:0 auto;padding:32px 22px 100px}
header{border-bottom:1px solid var(--line);padding-bottom:20px;margin-bottom:8px}
h1.page{font-size:27px;margin:0 0 8px}
.sub{color:var(--mut);font-size:14px}
.meta{display:flex;flex-wrap:wrap;gap:8px;margin-top:14px}
.tag{background:var(--card2);border:1px solid var(--line);border-radius:6px;padding:3px 10px;font-size:12.5px;color:var(--mut)}
.tag b{color:var(--accent);font-weight:600}
.toc{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:16px 20px;margin:20px 0 8px}
.toc .th{font-weight:700;color:#cdd5e2;margin-bottom:10px}
.toc ol{margin:0;padding-left:22px}
.toc li{margin:5px 0}
.toc a{color:var(--accent);text-decoration:none}
.toc a:hover{text-decoration:underline}
section.doc{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:8px 26px 26px;margin:26px 0}
h1.doctitle{font-size:22px;margin:22px 0 4px;padding-bottom:10px;border-bottom:2px solid var(--accent)}
.src{color:var(--mut);font-size:12.5px;margin-bottom:8px}
.src code{background:#262b35;color:#ffd9a0;padding:1px 6px;border-radius:4px}
.back{float:right;font-size:12.5px;color:var(--mut);text-decoration:none}
.back:hover{color:var(--accent)}
.md h1{font-size:20px;margin:26px 0 10px;color:#fff}
.md h2{font-size:18px;margin:24px 0 10px;padding-left:11px;border-left:4px solid var(--accent);color:#dbe3f0}
.md h3{font-size:15.5px;margin:18px 0 7px;color:#cdd5e2}
.md h4{font-size:14.5px;margin:14px 0 6px;color:#bcc6d6}
.md p{margin:9px 0}
.md a{color:var(--accent)}
.md ul,.md ol{margin:9px 0;padding-left:26px}
.md li{margin:4px 0}
.md code{font-family:"Cascadia Code",Consolas,monospace;background:#262b35;color:#ffd9a0;padding:1.5px 6px;border-radius:4px;font-size:13px}
.md pre{background:var(--code);border:1px solid var(--line);border-radius:9px;padding:15px;overflow:auto;font-size:13px;line-height:1.55}
.md pre code{background:none;color:#d6deea;padding:0}
.md table{width:100%;border-collapse:collapse;margin:13px 0;font-size:13.5px;display:block;overflow-x:auto}
.md th,.md td{border:1px solid var(--line);padding:8px 11px;text-align:left;vertical-align:top}
.md th{background:var(--card2);color:#cdd5e2;font-weight:600}
.md blockquote{margin:12px 0;padding:8px 16px;border-left:4px solid var(--mut);background:var(--card2);color:var(--mut);border-radius:0 8px 8px 0}
.md hr{border:none;border-top:1px solid var(--line);margin:20px 0}
.md img{max-width:100%}
.missing{color:var(--danger)}
"""


def build():
    nav_items = []
    sections = []
    for i, (slug, title, rel) in enumerate(DOCS, 1):
        p = os.path.join(REPO, rel)
        if os.path.exists(p):
            body = render_md(p)
        else:
            body = '<p class="missing">（文件缺失：%s）</p>' % html.escape(rel)
        nav_items.append('<li><a href="#%s">%s</a></li>' % (slug, html.escape(title)))
        sections.append(
            '<section id="%s" class="doc">'
            '<a class="back" href="#top">回到目录 ↑</a>'
            '<h1 class="doctitle">%d. %s</h1>'
            '<div class="src">来源: <code>%s</code></div>'
            '<div class="md">%s</div>'
            '</section>' % (slug, i, html.escape(title), html.escape(rel), body)
        )

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    head = (
        '<!DOCTYPE html><html lang="zh-CN"><head><meta charset="UTF-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1.0">'
        '<title>abot small-car 团队必读文档汇总</title><style>' + CSS + '</style></head><body>'
    )
    header = (
        '<div class="wrap" id="top"><header>'
        '<h1 class="page">abot small-car · 团队必读文档汇总</h1>'
        '<div class="sub">单文件离线 HTML，浏览器直接打开即可阅读，无需克隆仓库。</div>'
        '<div class="meta">'
        '<span class="tag">仓库 <b>zhangdashuai968/small-car</b></span>'
        '<span class="tag">文档 <b>%d 份</b></span>'
        '<span class="tag">生成 <b>%s</b></span>'
        '<span class="tag">平台 <b>Melodic / Tegra X1</b></span>'
        '</header>' % (len(DOCS), now)
    )
    toc = '<div class="toc"><div class="th">目录</div><ol>' + "".join(nav_items) + '</ol></div>'
    tail = "".join(sections) + '</div></body></html>'

    out = head + header + toc + tail
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(out)
    print("written:", OUT)
    print("size:", os.path.getsize(OUT), "bytes")


if __name__ == "__main__":
    build()
