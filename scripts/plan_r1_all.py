#!/usr/bin/env python3
"""R1 全部段自动规划: capture + moveit(起点->终点), 终点强制精确, 段间连续"""
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "data" / "captured_paths"


def plan_one(name: str, start: list, goal: list) -> bool:
    out_file = f"{name}.json"
    cap = subprocess.Popen(
        ["python3", "scripts/capture_rviz_path.py", out_file],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(3)
    try:
        r = subprocess.run(
            ["python3", "scripts/moveit_plan_goal.py"]
            + [str(x) for x in start] + [str(x) for x in goal],
            capture_output=True, text=True, timeout=90,
        )
    except subprocess.TimeoutExpired:
        cap.terminate()
        print(f"  {name}: 规划超时 FAIL")
        return False
    for _ in range(90):
        if cap.poll() is not None:
            break
        time.sleep(0.5)
    if cap.poll() is None:
        cap.terminate()
        print(f"  {name}: 捕获超时 FAIL")
        return False
    out = OUT_DIR / out_file
    if not out.exists():
        print(f"  {name}: 无轨迹文件 FAIL")
        return False
    data = json.load(open(out))
    pts = data["trajectories"][0]["points_deg"]
    pts[-1] = list(goal)
    data["trajectories"][0]["n_points"] = len(pts)
    out.write_text(json.dumps(data, ensure_ascii=False, indent=1))
    print(f"  {name}: OK {len(pts)}点  start={[round(x,1) for x in pts[0]]}")
    return True


def main() -> int:
    d = json.load(open(OUT_DIR / "r1_key_poses.json"))
    segments = d["segments"]
    print(f"共 {len(segments)} 段")
    ok = 0
    for seg in segments:
        ok += plan_one(seg["name"], seg["start"], seg["goal"])
    print(f"\n完成 {ok}/{len(segments)}")
    return 0 if ok == len(segments) else 1


if __name__ == "__main__":
    raise SystemExit(main())
