#!/usr/bin/env python3
"""记录 R1 各段关键姿态（真实端点 = r1_complete_cycle_plan.json + 场景新测）"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# 旧 plan 真实端点（endpoints_deg）
old = json.load(open(ROOT / "robot_control" / "plans" / "r1_complete_cycle_plan.json"))
EP = {k: [round(float(x), 4) for x in v] for k, v in old["endpoints_deg"].items()}

# 新测姿态（比旧端点更准，替换对应位置）
NEW_POSES = {
    "box_grasp":          [-70.0, -3.0, -106.3, 20.2, 90.2, -15.0],
    "box_place_final":    [0.9, -35.9, -55.5, 1.9, 89.2, 55.9],
    "term_transition":    [-14.6, 2.5, -50.6, -22.2, 91.0, 40.4],
    "term_grasp":         [-61.7, -31.3, -72.7, 15.0, 90.1, -6.7],
    "term_install_end":   [-0.3, -44.7, -33.1, -11.8, 89.2, 54.7],
}

KEY_POSES = {
    "home":               [0.0] * 6,
    "box_pick_app":       EP["box_pick_app"],
    "box_pick_tcp":       EP["box_pick_tcp"],
    "box_grasp":          NEW_POSES["box_grasp"],
    "box_place_app":      EP["box_place_app"],
    "box_place_final":    NEW_POSES["box_place_final"],
    "term_transition":    NEW_POSES["term_transition"],
    "terminal_pick_app":  EP["terminal_pick_app"],
    "terminal_pick_tcp":  EP["terminal_pick_tcp"],
    "term_grasp":         NEW_POSES["term_grasp"],
    "terminal_place_app": EP["terminal_place_app"],
    "terminal_place_tcp": EP["terminal_place_tcp"],
    "term_install_end":   NEW_POSES["term_install_end"],
}

# 完整段列表: (段名, 起点, 终点) —— 与 moveit 规划脚本共用
SEGMENTS = [
    ("r1_initial_to_box_pick_app", "home", "box_pick_app"),
    ("r1_box_descend", "box_pick_app", "box_pick_tcp"),
    ("r1_box_grasp", "box_pick_tcp", "box_grasp"),
    ("r1_box_lift_and_transfer", "box_grasp", "box_place_app"),
    ("r1_box_place_descend", "box_place_app", "box_place_final"),
    ("r1_box_to_term_transition", "box_place_final", "term_transition"),
    ("r1_terminal_approach", "term_transition", "terminal_pick_app"),
    ("r1_terminal_descend", "terminal_pick_app", "term_grasp"),
    ("r1_terminal_lift_and_transfer", "term_grasp", "terminal_place_app"),
    ("r1_terminal_place_descend", "terminal_place_app", "term_install_end"),
    ("r1_return_home", "term_install_end", "box_pick_app"),
]

OUT = {
    "robot": "R1",
    "captured_at": "2026-08-09",
    "note": "box_grasp/box_place_final/term_transition/term_grasp/term_install_end 为场景新测, 其余为 r1_complete_cycle_plan.json 端点",
    "key_poses": KEY_POSES,
    "segments": [{"name": n, "start": KEY_POSES[s], "goal": KEY_POSES[g]}
                 for n, s, g in SEGMENTS],
}

path = ROOT / "data" / "captured_paths" / "r1_key_poses.json"
path.write_text(json.dumps(OUT, ensure_ascii=False, indent=1))
print(f"已保存 {path}")
print(f"关键姿态 {len(KEY_POSES)} 个, 段 {len(SEGMENTS)} 个")
