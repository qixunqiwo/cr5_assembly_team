#!/usr/bin/env python3
"""重新规划机器人全部段 (moveit 原始轨迹, 不做 blend/平滑)

只强制 pts[0]=start, pts[-1]=goal。覆盖被 blend/平滑改过的文件。
用法: python3 scripts/replan_robot.py <r1|r2|r3>
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import RobotState, JointConstraint, Constraints
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "data" / "captured_paths"


def plan(cli, node, start, goal):
    req = GetMotionPlan.Request()
    req.motion_plan_request.group_name = "cr5_group"
    req.motion_plan_request.planner_id = "OMPL/RRTConnectkConfigDefault"
    req.motion_plan_request.allowed_planning_time = 8.0
    req.motion_plan_request.num_planning_attempts = 5
    req.motion_plan_request.start_state = RobotState()
    req.motion_plan_request.start_state.joint_state = JointState()
    req.motion_plan_request.start_state.joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    req.motion_plan_request.start_state.joint_state.position = [math.radians(x) for x in start]
    req.motion_plan_request.goal_constraints.append(Constraints(joint_constraints=[
        JointConstraint(joint_name=f"joint{i + 1}", position=math.radians(v),
                        tolerance_above=0.02, tolerance_below=0.02, weight=1.0)
        for i, v in enumerate(goal)]))
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=40.0)
    resp = fut.result()
    if not resp or not resp.motion_plan_response.trajectory.joint_trajectory.points:
        return None
    return [[round(math.degrees(v), 4) for v in p.positions]
            for p in resp.motion_plan_response.trajectory.joint_trajectory.points]


def main() -> int:
    if len(sys.argv) != 2 or sys.argv[1] not in ("r1", "r2", "r3", "r4"):
        print("用法: replan_robot.py <r1|r2|r3>")
        return 1
    robot = sys.argv[1]
    d = json.load(open(OUT_DIR / f"{robot}_key_poses.json"))

    rclpy.init()
    node = rclpy.create_node(f"{robot}_replan")
    cli = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    for _ in range(15):
        if cli.wait_for_service(timeout_sec=1.0):
            break
        time.sleep(0.5)
    print("moveit ready:", cli.service_is_ready())

    ok = 0
    for seg in d["segments"]:
        name, start, goal = seg["name"], seg["start"], seg["goal"]
        pts = plan(cli, node, start, goal)
        if pts is None:
            print(f"  {name}: FAIL")
            continue
        pts[0], pts[-1] = list(start), list(goal)
        out = {"captured_at": f"moveit-{robot}-clean", "trajectories": [{
            "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            "points_deg": pts, "n_points": len(pts)}]}
        (OUT_DIR / f"{name}.json").write_text(
            json.dumps(out, ensure_ascii=False, indent=1))
        print(f"  {name}: OK {len(pts)}点")
        ok += 1
    node.destroy_node()
    rclpy.shutdown()
    print(f"完成 {ok}/{len(d['segments'])}")
    return 0 if ok == len(d["segments"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
