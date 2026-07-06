"""生成自包含单文件游戏页 game/index.html。

把引擎模块 + game_core.py 以 JSON 内联进模板(game/docs/index.html 的
__MODULES_JSON__ 占位符),浏览器端零运行时文件请求。

用法: python airship-sim/game/build_game.py
"""
from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE.parent / "airship_sim"
TEMPLATE = HERE / "docs" / "index.html"
OUT = HERE / "index.html"

ENGINE_MODULES = [
    "__init__", "config", "math3d", "added_mass", "aero", "dynamics",
    "sensors", "controller", "atmosphere", "wind_field", "ground", "energy",
    "presets_crewed", "simulation", "environments",
]


def build() -> None:
    modules = {name: (PKG / f"{name}.py").read_text(encoding="utf-8")
               for name in ENGINE_MODULES}
    modules["game_core"] = (HERE / "game_core.py").read_text(encoding="utf-8")

    html = TEMPLATE.read_text(encoding="utf-8")
    marker = "__MODULES_JSON__"
    if marker not in html:
        raise SystemExit("模板缺少 __MODULES_JSON__ 占位符")
    payload = json.dumps(modules, ensure_ascii=False).replace("</", "<\\/")
    html = html.replace(marker, payload, 1)
    OUT.write_text(html, encoding="utf-8")
    total = sum(len(v.encode("utf-8")) for v in modules.values())
    print(f"生成 {OUT}  ({OUT.stat().st_size} bytes, 内联 Python {total} bytes, "
          f"{len(modules)} 模块)")


if __name__ == "__main__":
    build()
