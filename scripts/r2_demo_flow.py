#!/usr/bin/env python3
"""R2 完整流程回放（moveit 预录路径 v1）

流程: 抓PCB -> attach -> 等待点 -> 放PCB(装配位) -> detach -> 回家
路径: data/captured_paths/r2_*.json
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

FLOW = [
    ("抓PCB", ["r2_initial_to_pick_app.json", "r2_pick_descend.json"], "r2_pick_to_safe_wait.json"),
    ("等待点", ["r2_safe_wait_to_place_app.json"], None),
    ("放PCB", ["r2_place_descend.json"], None),
    ("回等待点", ["r2_place_to_safe_wait.json"], None),
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
    r2 = b.get_object_handle("R2")
    r2j = b.get_robot_joint_handles("R2")
    pcb = b.get_object_handle("PCB_SUPPLY")
    box = b.get_object_handle("BOX_BLANK")
    try:
        sim.removeObjects([b.get_object_handle("ASSEMBLY_PRODUCT")])
    except Exception:
        pass
    for j in r2j:
        sim.setJointPosition(j, 0.0)
    sim.setObjectPosition(pcb, -1, [-1.22, -0.42, 0.1584])
    # 箱体放到装配台, 便于观察 PCB 落位
    sim.setObjectPosition(box, -1, [-1.078563, 0.120898, 0.21946])
    for s in sim.getObjectsInTree(pcb, sim.object_shape_type, 0):
        sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 1)

    orig_maxvel = []
    for j in r2j:
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
                for j, v in zip(r2j, [math.radians(x) for x in tgt]):
                    sim.setJointPosition(j, v)
                b.step()
                time.sleep(0.0002)
        for j, v in zip(r2j, [math.radians(x) for x in pts[-1]]):
            sim.setJointPosition(j, v)
        b.step()

    b.start_simulation()
    time.sleep(0.3)
    attached = False
    try:
        for label, fwd, lift in FLOW:
            print(f"--- {label} ---")
            for f in fwd:
                replay(load(f))
            if label == "抓PCB" and not attached:
                b.attach_object("PCB_SUPPLY", "R2")
                attached = True
                replay(load(lift))
                print("  [PCB attach, 去等待点]")
            elif label == "放PCB" and attached:
                b.detach_object(pcb)
                attached = False
                print("  [PCB detach]")
    finally:
        if attached:
            try:
                b.detach_object(pcb)
            except Exception:
                pass
        try:
            for j, mv in zip(r2j, orig_maxvel):
                sim.setObjectFloatParam(j, sim.jointfloatparam_maxvel, mv)
        except Exception:
            pass
        try:
            sim.setObjectPosition(pcb, -1, [-1.22, -0.42, 0.1584])
            sim.setObjectPosition(box, -1, [-1.86, 0.22, 0.156])
        except Exception:
            pass
        try:
            for j in r2j:
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
