#!/usr/bin/env python3
"""R3 完整流程回放（moveit 预录路径 v1）

流程: 抓模块->放模块(装配)->抓产品->转运产品到检测区->回家
路径: data/captured_paths/r3_*.json
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
    ("抓模块", ["r3_initial_to_module_pick_app.json", "r3_module_pick_descend.json"], None),
    ("搬模块", ["r3_module_lift_transfer.json"], None),
    ("放模块", ["r3_module_place_descend.json"], None),
    ("抬升", ["r3_module_place_descend.json"], None),
    ("去产品", ["r3_module_to_product_pick_app.json"], None),
    ("抓产品", ["r3_product_pick_descend.json"], None),
    ("转运产品", ["r3_product_transfer.json"], None),
    ("放产品", ["r3_product_place_descend.json"], None),
    ("抬升离开", ["r3_product_place_descend.json"], None),
    ("回抓模块上方", ["r3_place_to_module_pick_app.json"], None),
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
    r3 = b.get_object_handle("R3")
    r3j = b.get_robot_joint_handles("R3")
    module = b.get_object_handle("CONTROL_MODULE_SUPPLY")
    product = b.get_object_handle("INSPECTION_PRODUCT")
    try:
        sim.removeObjects([b.get_object_handle("ASSEMBLY_PRODUCT")])
    except Exception:
        pass
    for j in r3j:
        sim.setJointPosition(j, 0.0)
    sim.setObjectPosition(module, -1, [-0.78, -0.20, 0.1665])
    sim.setObjectPosition(product, -1, [-1.08, 0.12, 0.2160])
    for part in (module, product):
        for s in sim.getObjectsInTree(part, sim.object_shape_type, 0):
            sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 1)

    orig_maxvel = []
    for j in r3j:
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
                for j, v in zip(r3j, [math.radians(x) for x in tgt]):
                    sim.setJointPosition(j, v)
                b.step()
                time.sleep(0.0002)
        for j, v in zip(r3j, [math.radians(x) for x in pts[-1]]):
            sim.setJointPosition(j, v)
        b.step()

    b.start_simulation()
    time.sleep(0.3)
    attached_module = False
    attached_product = False
    try:
        for label, fwd, _ in FLOW:
            print(f"--- {label} ---")
            if label in ("抬升离开", "抬升"):
                replay(load(fwd[0])[::-1])
                print("  [垂直抬升]")
                continue
            for f in fwd:
                replay(load(f))
            if label == "抓模块" and not attached_module:
                b.set_gripper_gap("R3", 0.080)
                time.sleep(0.1)
                b.attach_object("CONTROL_MODULE_SUPPLY", "R3")
                attached_module = True
                print("  [模块 attach]")
            elif label == "放模块" and attached_module:
                b.set_gripper_gap("R3", 0.170)
                b.detach_object(module)
                attached_module = False
                print("  [模块 detach, 松开夹爪]")
                start = list(sim.getObjectPosition(module, -1))
                target = [-1.053, 0.111, 0.267]
                for k in range(1, 9):
                    f = k / 8
                    sim.setObjectPosition(module, -1, [start[m] + (target[m] - start[m]) * f for m in range(3)])
                    b.step()
                tp = sim.getObjectPosition(module, -1)
                print(f"  [模块下降到位 z={tp[2]:.4f} (标准 0.267)]")
            elif label == "抓产品" and not attached_product:
                b.set_gripper_gap("R3", 0.170)
                print("  [夹爪张开]")
                b.attach_object("INSPECTION_PRODUCT", "R3")
                attached_product = True
                print("  [产品 attach]")
            elif label == "放产品" and attached_product:
                b.set_gripper_gap("R3", 0.170)
                time.sleep(0.1)
                b.detach_object(product)
                attached_product = False
                print("  [产品 detach]")
    finally:
        if attached_module:
            try:
                b.detach_object(module)
            except Exception:
                pass
        if attached_product:
            try:
                b.detach_object(product)
            except Exception:
                pass
        try:
            for j, mv in zip(r3j, orig_maxvel):
                sim.setObjectFloatParam(j, sim.jointfloatparam_maxvel, mv)
        except Exception:
            pass
        try:
            sim.setObjectPosition(module, -1, [-0.78, -0.20, 0.1665])
            sim.setObjectPosition(product, -1, [3.0, 3.0, 0.5])
        except Exception:
            pass
        try:
            for j in r3j:
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
