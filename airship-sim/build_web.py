"""生成自包含单文件 web 版 index.html。

把 airship_sim/ 下的 Python 物理模块以 JSON 内联进 HTML,
浏览器端无需任何运行时文件请求(fetch),因此可在任意静态主机
(GitHub Pages 无论是否 Jekyll、Vercel、本地 file://)直接运行。

用法:
    python airship-sim/build_web.py
输出:
    airship-sim/index.html         (自包含,部署用)
    airship-sim/docs/index.html    (模板,fetch 版,开发用,不改动)

设计:渲染层零物理逻辑;此脚本只做"把源码搬进 HTML",不改任何物理。
"""
from __future__ import annotations

import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE / "airship_sim"
TEMPLATE = HERE / "docs" / "index.html"
OUT = HERE / "index.html"

# 加载顺序即模块依赖顺序(与模板里的 mods 数组一致)
MODULES = [
    "__init__", "config", "math3d", "added_mass", "aero",
    "dynamics", "sensors", "controller", "atmosphere", "simulation",
]

# fetch 版加载片段(模板中的原文),整段替换为内联读取
FETCH_SNIPPET = """    const resp = await fetch(`./airship_sim/${mod}.py`);
    if (!resp.ok) throw new Error(`Failed to fetch ${mod}.py: ${resp.status}`);
    const code = await resp.text();
    py.FS.writeFile(`/airship_sim/${mod}.py`, code);"""

INLINE_SNIPPET = """    py.FS.writeFile(`/airship_sim/${mod}.py`, AIRSHIP_MODULES[mod]);"""


def build() -> None:
    modules = {name: (PKG / f"{name}.py").read_text(encoding="utf-8")
               for name in MODULES}

    html = TEMPLATE.read_text(encoding="utf-8")

    if FETCH_SNIPPET not in html:
        raise SystemExit("模板里没找到 fetch 加载片段,模板结构可能已变,请检查 build_web.py")
    html = html.replace(FETCH_SNIPPET, INLINE_SNIPPET)

    # 在 mods 数组前声明 AIRSHIP_MODULES(从内联 JSON 读取)
    marker = "  const mods = ['__init__', 'config', 'math3d', 'added_mass', 'aero',"
    if marker not in html:
        raise SystemExit("模板里没找到 mods 数组声明")
    html = html.replace(
        marker,
        "  const AIRSHIP_MODULES = JSON.parse("
        "document.getElementById('airship-py-modules').textContent);\n" + marker,
    )

    # JSON 块:</script> 用 <\\/script> 规避 HTML 解析器提前闭合(script 内 JSON 的标准做法)
    payload = json.dumps(modules, ensure_ascii=False).replace("</", "<\\/")
    json_block = (
        '<script type="application/json" id="airship-py-modules">'
        + payload + "</script>\n"
    )
    # 插在 importmap 之前
    anchor = '<script type="importmap">'
    if anchor not in html:
        raise SystemExit("模板里没找到 importmap 锚点")
    html = html.replace(anchor, json_block + anchor, 1)

    OUT.write_text(html, encoding="utf-8")
    total = sum(len(v.encode("utf-8")) for v in modules.values())
    print(f"生成 {OUT}  ({OUT.stat().st_size} bytes, 内联 Python {total} bytes, "
          f"{len(MODULES)} 模块)")


if __name__ == "__main__":
    build()
