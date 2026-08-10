#!/usr/bin/env python3
"""段边界速度连续过渡: 用 Hermite 样条替换每段开头 2 点

对每对相邻段 (A, B):
  过渡从 A[-1] 出发, 入口速度 = A 尾部方向, 出口速度 = B 方向,
  样条终点 = B[1], 然后 B 从 B[2] 继续。
桥接对(反走抬升衔接)跳过: r1 box_grasp->lift / terminal_descend->mid_transfer。
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "data" / "captured_paths"

# demo 中反走抬升桥接的段对 (数据不连续但 demo 用反走衔接)
BRIDGES = {
    "r1": {("r1_box_grasp", "r1_box_lift_and_transfer"),
           ("r1_terminal_descend", "r1_terminal_mid_transfer")},
    "r2": set(),
    "r3": {("r3_product_place_descend", "r3_place_to_module_pick_app")},
}


def hermite(p0, v0, p1, v1, n):
    pts = []
    for i in range(1, n + 1):
        t = i / (n + 1)
        h00 = 2 * t ** 3 - 3 * t ** 2 + 1
        h10 = t ** 3 - 2 * t ** 2 + t
        h01 = -2 * t ** 3 + 3 * t ** 2
        h11 = t ** 3 - t ** 2
        pts.append([h00 * p0[m] + h10 * v0[m] + h01 * p1[m] + h11 * v1[m]
                    for m in range(6)])
    return pts


def blend_pair(a_pts, b_pts, n=6):
    p0 = a_pts[-1]
    p1 = b_pts[1]
    v0 = [a_pts[-1][m] - a_pts[-2][m] for m in range(6)]
    v1 = [b_pts[2][m] - b_pts[1][m] for m in range(6)]
    trans = [p0] + hermite(p0, v0, p1, v1, n) + [p1]
    return trans + b_pts[2:]


def main():
    total = 0
    for robot in ("r1", "r2", "r3"):
        d = json.load(open(OUT_DIR / f"{robot}_key_poses.json"))
        segs = d["segments"]
        for i in range(len(segs) - 1):
            an, bn = segs[i]["name"], segs[i + 1]["name"]
            if (an, bn) in BRIDGES.get(robot, set()):
                print(f"{robot}: 跳过桥接对 {an} -> {bn}")
                continue
            af = OUT_DIR / f"{an}.json"
            bf = OUT_DIR / f"{bn}.json"
            if not af.exists() or not bf.exists():
                continue
            a = json.load(open(af))
            b = json.load(open(bf))
            a_pts = a["trajectories"][0]["points_deg"]
            b_pts = b["trajectories"][0]["points_deg"]
            gap0 = max(abs(x - y) for x, y in zip(a_pts[-1], b_pts[0]))
            if gap0 > 0.01:
                print(f"{robot}: {an}->{bn} 起点终点不一致({gap0:.3f}°) 跳过")
                continue
            new_b = blend_pair(a_pts, b_pts)
            # 保证终点不变
            new_b[-1] = list(b_pts[-1])
            b["trajectories"][0]["points_deg"] = new_b
            b["trajectories"][0]["n_points"] = len(new_b)
            bf.write_text(json.dumps(b, ensure_ascii=False, indent=1))
            total += 1
            print(f"{robot}: {bn} 开头已过渡 ({len(a_pts)}->A尾, {len(b_pts)}->{len(new_b)}点)")
    print(f"完成 {total} 处过渡")


if __name__ == "__main__":
    main()
