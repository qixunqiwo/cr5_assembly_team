#!/usr/bin/env python3
"""R1 单段规划: 启动捕获子进程 + moveit 规划(起点->终点) + 验证保存

用法: python3 scripts/plan_and_capture.py <段名>
段名见 data/captured_paths/r1_key_poses.json 的 segments。
"""
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "data" / "captured_paths"


def main() -> int:
    if len(sys.argv) != 2:
        print("用法: plan_and_capture.py <段名>")
        return 1
    name = sys.argv[1]
    d = json.load(open(OUT_DIR / "r1_key_poses.json"))
    seg = next((s for s in d["segments"] if s["name"] == name), None)
    if seg is None:
        print(f"未找到段 {name}")
        return 1
    start, goal = seg["start"], seg["goal"]
    out_file = f"{name}.json"

    print(f"段: {name}")
    print(f"  起点: {start}")
    print(f"  终点: {goal}")

    cap = subprocess.Popen(
        ["python3", "scripts/capture_rviz_path.py", out_file],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(3)
    try:
        r = subprocess.run(
            ["python3", "scripts/moveit_plan_goal.py"]
            + [str(x) for x in start] + [str(x) for x in goal],
            capture_output=True, text=True, timeout=60,
        )
        print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "规划无输出")
    finally:
        for _ in range(60):
            if cap.poll() is not None:
                break
            time.sleep(0.5)
    if cap.poll() is None:
        cap.terminate()
        print("捕获超时未收到轨迹")
        return 1
    out = OUT_DIR / out_file
    if not out.exists():
        print("未保存轨迹文件")
        return 1
    data = json.load(open(out))
    pts = data["trajectories"][0]["points_deg"]
    print(f"捕获完成: {len(pts)} 点")
    print(f"  start: {[round(x,1) for x in pts[0]]}")
    print(f"  end  : {[round(x,1) for x in pts[-1]]}")
    print(f"  目标 : {[round(x,1) for x in goal]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
