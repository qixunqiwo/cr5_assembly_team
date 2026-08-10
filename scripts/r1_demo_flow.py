#!/usr/bin/env python3
"""R1 完整流程回放（moveit 预录路径 v2，含新测姿态）

流程: 抓箱->放箱->抓端子->安装端子
路径: data/captured_paths/r1_*.json (11 段, 新测终点已含接触姿态)
"""
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim_bridge.coppelia_client import SimBridge

MAX_STEP = 5.0
STEP_DELAY = 0.0

# (标签, 前进段列表, 抬升段名[可None]) —— 抬升段在 attach 后反走
FLOW = [
    ("抓箱", ["r1_initial_to_box_pick_app.json", "r1_box_descend.json", "r1_box_grasp.json"],
     "r1_box_grasp.json"),
    ("搬箱", ["r1_box_lift_and_transfer.json"], None),
    ("放箱", ["r1_box_place_descend.json"], None),
    ("去端子", ["r1_box_to_term_transition.json", "r1_terminal_approach.json"], None),
    ("抓端子", ["r1_terminal_descend.json"], "r1_terminal_descend.json"),
    ("搬端子", ["r1_terminal_mid_transfer.json", "r1_terminal_mid_to_place_app.json"], None),
    ("安装端子", ["r1_terminal_place_descend.json"], None),
    ("抬升离开", ["r1_terminal_place_descend.json"], None),
    ("回抓箱上方", ["r1_return_home.json"], None),
]


def load(f):
    d = json.load(open(ROOT / "data" / "captured_paths" / f))
    return d["trajectories"][0]["points_deg"]


def main():
    b = SimBridge(request_timeout=20.0)
    for _ in range(10):
        if b.connect():
            break
        time.sleep(2)
    if not b._connected:
        print("connect failed:", b.last_error)
        return 1
    sim = b._client.require("sim")
    print(f"scene: {b.scene_path()}")

    if sim.getSimulationState() != 0:
        b.stop_simulation()
        time.sleep(0.5)
    r1 = b.get_object_handle("R1")
    r1j = b.get_robot_joint_handles("R1")
    box = b.get_object_handle("BOX_BLANK")
    terminal = b.get_object_handle("TERMINAL_BLOCK_SUPPLY")
    try:
        sim.removeObjects([b.get_object_handle("ASSEMBLY_PRODUCT")])
    except Exception:
        pass
    for j in r1j:
        sim.setJointPosition(j, 0.0)
    # 物料复位到供应位
    sim.setObjectPosition(box, -1, [-1.86, 0.22, 0.156])
    sim.setObjectPosition(terminal, -1, [-1.82, -0.02, 0.1665])
    for part in (box, terminal):
        for s in sim.getObjectsInTree(part, sim.object_shape_type, 0):
            sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 1)

    # 步进模式 + 关节最大速度提升（同 R5 演示）
    orig_maxvel = []
    for j in r1j:
        orig_maxvel.append(sim.getObjectFloatParam(j, sim.jointfloatparam_maxvel))
        sim.setObjectFloatParam(j, sim.jointfloatparam_maxvel, math.radians(500.0))
    b.set_stepping(True)

    def replay(pts):
        for i in range(len(pts) - 1):
            a, c = pts[i], pts[i + 1]
            gap = max(abs(x - y) for x, y in zip(a, c))
            n = max(1, int(gap / MAX_STEP) + 1)
            for k in range(1, n + 1):
                f = k / n
                tgt = [a[m] + (c[m] - a[m]) * f for m in range(6)]
                for j, v in zip(r1j, [math.radians(x) for x in tgt]):
                    sim.setJointPosition(j, v)
                b.step()
                time.sleep(0.0002)
        for j, v in zip(r1j, [math.radians(x) for x in pts[-1]]):
            sim.setJointPosition(j, v)
        b.step()

    b.start_simulation()
    time.sleep(0.3)
    attached_box = False
    attached_term = False
    try:
        for label, fwd, lift in FLOW:
            print(f"--- {label} ---")
            if label == "抬升离开":
                replay(load(fwd[0])[::-1])
                print("  [垂直抬升]")
                continue
            for f in fwd:
                replay(load(f))
            if label == "抓箱" and not attached_box:
                b.set_gripper_gap("R1", 0.150)
                
                b.attach_object("BOX_BLANK", "R1")
                attached_box = True
                replay(load(lift)[::-1])
                print("  [箱体 attach + 抬升]")
            elif label == "放箱" and attached_box:
                b.set_gripper_gap("R1", 0.158)
                
                b.detach_object(box)
                attached_box = False
                print("  [箱体 detach]")
            elif label == "抓端子" and not attached_term:
                b.set_gripper_gap("R1", 0.046)
                
                b.attach_object("TERMINAL_BLOCK_SUPPLY", "R1")
                attached_term = True
                replay(load(lift)[::-1])
                print("  [端子 attach + 抬升]")
            elif label == "安装端子" and attached_term:
                b.detach_object(terminal)
                attached_term = False
                print("  [端子 detach, 保持夹紧]")
            elif label == "抬升离开":
                replay(load(fwd[0])[::-1])
                print("  [垂直抬升]")
            elif label == "回抓箱上方" and not attached_term and not attached_box:
                b.set_gripper_gap("R1", 0.158)
                print("  [夹爪张开复位]")
    finally:
        if attached_box:
            try:
                b.detach_object(box)
            except Exception:
                pass
        if attached_term:
            try:
                b.detach_object(terminal)
            except Exception:
                pass
        try:
            sim.setObjectPosition(box, -1, [-1.86, 0.22, 0.156])
            sim.setObjectPosition(terminal, -1, [-1.82, -0.02, 0.1665])
        except Exception:
            pass
        try:
            for j, mv in zip(r1j, orig_maxvel):
                sim.setObjectFloatParam(j, sim.jointfloatparam_maxvel, mv)
        except Exception:
            pass
        try:
            for j in r1j:
                sim.setJointPosition(j, 0.0)
            time.sleep(0.3)
            b.stop_simulation()
            time.sleep(0.3)
        except Exception:
            pass
        try:
            b.disconnect()
        except Exception:
            pass
    print("完成, 物料已复位")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
