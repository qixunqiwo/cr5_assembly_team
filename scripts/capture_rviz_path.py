#!/usr/bin/env python3
"""捕获 moveit 规划路径（RViz 点 Plan 后触发）

用法：
  1. 先启动这个脚本（保持运行）
  2. 在 RViz 里设置目标，点 "Plan"
  3. 脚本捕获到路径后保存到 data/captured_path.json 并退出

路径格式：
  {
    "captured_at": ...,
    "joint_names": [...],
    "points_deg": [ [j1..j6], ... ]   # 每个点6个关节角(度)
  }
"""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from moveit_msgs.msg import DisplayTrajectory

ROOT = Path(__file__).resolve().parents[1]
OUT = (
    ROOT / "data" / "captured_paths" / sys.argv[1]
    if len(sys.argv) > 1
    else ROOT / "data" / "captured_path.json"
)


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("r5_capture_path")

    captured = []

    def on_msg(msg: DisplayTrajectory) -> None:
        nonlocal captured
        for traj in msg.trajectory:
            jt = traj.joint_trajectory
            joint_names = list(jt.joint_names)
            points_deg = []
            for point in jt.points:
                if point.positions:
                    points_deg.append([round(math.degrees(v), 4) for v in point.positions])
            if points_deg:
                captured.append(
                    {
                        "joint_names": joint_names,
                        "points_deg": points_deg,
                        "n_points": len(points_deg),
                    }
                )

    sub = node.create_subscription(
        DisplayTrajectory, "/display_planned_path", on_msg, 10
    )
    print("=" * 60)
    print("路径捕获脚本运行中...")
    print(f"输出: {OUT}")
    print("请在 RViz 里设置目标并点 'Plan'")
    print("捕获到路径后自动保存, 按 Ctrl+C 手动退出")
    print("=" * 60)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if captured:
                break
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if not captured:
        print("\n未捕获到任何路径")
        rclpy.shutdown()
        return 1

    result = {
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "trajectories": captured,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1))
    print(f"\n✅ 已捕获 {len(captured)} 段轨迹")
    for c in captured:
        print(f"   joints={c['joint_names']}  点数={c['n_points']}")
        print(f"   start={c['points_deg'][0]}")
        print(f"   end  ={c['points_deg'][-1]}")
    print(f"\n保存到: {OUT}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
