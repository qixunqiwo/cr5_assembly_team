#!/usr/bin/env python3
"""用 moveit 设置 goal 关节角并规划（用于 RViz 显示 + 捕获路径）

用法: python3 scripts/moveit_plan_goal.py  <j1 j2 j3 j4 j5 j6(度)>
"""
import math
import sys
import time

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState

if len(sys.argv) == 7:
    START_DEG = [0.0] * 6
    GOAL_DEG = [float(x) for x in sys.argv[1:7]]
elif len(sys.argv) == 13:
    START_DEG = [float(x) for x in sys.argv[1:7]]
    GOAL_DEG = [float(x) for x in sys.argv[7:13]]
else:
    print("用法: 终点(j1..j6) 或 起点(j1..j6)+终点(j1..j6)")
    sys.exit(1)

rclpy.init()
node = rclpy.create_node("moveit_set_goal")
cli = node.create_client(GetMotionPlan, "/plan_kinematic_path")
for _ in range(15):
    if cli.wait_for_service(timeout_sec=1.0):
        break
    time.sleep(0.5)
print("plan_kinematic_path ready:", cli.service_is_ready())
if not cli.service_is_ready():
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

req = GetMotionPlan.Request()
req.motion_plan_request.group_name = "cr5_group"
req.motion_plan_request.planner_id = "OMPL/RRTConnectkConfigDefault"
req.motion_plan_request.allowed_planning_time = 8.0
req.motion_plan_request.num_planning_attempts = 5

# start state = 指定起点（默认零位）
req.motion_plan_request.start_state = RobotState()
req.motion_plan_request.start_state.joint_state = JointState()
req.motion_plan_request.start_state.joint_state.name = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
]
req.motion_plan_request.start_state.joint_state.position = START_DEG

# goal = PICK
req.motion_plan_request.goal_constraints.append(
    __import__("moveit_msgs.msg", fromlist=["Constraints"]).Constraints(
        joint_constraints=[
            __import__("moveit_msgs.msg", fromlist=["JointConstraint"]).JointConstraint(
                joint_name=f"joint{i + 1}",
                position=math.radians(v),
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for i, v in enumerate(GOAL_DEG)
        ]
    )
)

print(f"规划中: home -> {GOAL_DEG}")
future = cli.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
resp = future.result()
if resp is None:
    print("规划失败（无响应）")
else:
    print("error_code:", resp.motion_plan_response.error_code.val)
    print("轨迹点数:", len(resp.motion_plan_response.trajectory.joint_trajectory.points) if resp.motion_plan_response.trajectory.joint_trajectory.points else 0)
    if resp.motion_plan_response.trajectory.joint_trajectory.points:
        p0 = resp.motion_plan_response.trajectory.joint_trajectory.points[0]
        pn = resp.motion_plan_response.trajectory.joint_trajectory.points[-1]
        print("start:", [round(math.degrees(v), 2) for v in p0.positions])
        print("end:  ", [round(math.degrees(v), 2) for v in pn.positions])

node.destroy_node()
rclpy.shutdown()
