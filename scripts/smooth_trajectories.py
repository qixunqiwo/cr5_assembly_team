#!/usr/bin/env python3
"""轨迹平滑: 3点滑动平均消除 moveit 锯齿, 保护段首尾点"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "data" / "captured_paths"


def smooth(pts):
    if len(pts) < 4:
        return pts
    out = [list(pts[0])]
    for i in range(1, len(pts) - 1):
        out.append([(pts[i - 1][m] + pts[i][m] + pts[i + 1][m]) / 3.0
                    for m in range(6)])
    out.append(list(pts[-1]))
    return out


def main():
    total = 0
    for robot in ("r1", "r2", "r3"):
        d = json.load(open(OUT_DIR / f"{robot}_key_poses.json"))
        for seg in d["segments"]:
            f = OUT_DIR / f"{seg['name']}.json"
            if not f.exists():
                continue
            j = json.load(open(f))
            pts = j["trajectories"][0]["points_deg"]
            new = smooth(pts)
            if new != pts:
                j["trajectories"][0]["points_deg"] = new
                j["trajectories"][0]["n_points"] = len(new)
                f.write_text(json.dumps(j, ensure_ascii=False, indent=1))
                total += 1
                print(f"{seg['name']}: {len(pts)} -> {len(new)} 点")
    print(f"完成 {total} 段")


if __name__ == "__main__":
    main()
