#!/usr/bin/env python3
"""R4 锁付流程回放（moveit 预录路径 v1）

流程: home -> 等待点 -> 锁付app -> tcp -> press(按压) -> 退回app -> 回等待点
路径: data/captured_paths/r4_*.json
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
    ("去等待点", ["r4_home_to_wait.json"], None),
    ("去锁付", ["r4_wait_to_app.json"], None),
    ("下降", ["r4_app_to_tcp.json"], None),
    ("按压锁付", ["r4_tcp_to_press.json"], "press"),
    ("退回", ["r4_press_to_app.json"], None),
    ("回等待点", ["r4_app_to_wait.json"], None),
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
    r4 = b.get_object_handle("R4")
    r4j = b.get_robot_joint_handles("R4")
    for j in r4j:
        sim.setJointPosition(j, 0.0)
    orig_maxvel = []
    for j in r4j:
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
                for j, v in zip(r4j, [math.radians(x) for x in tgt]):
                    sim.setJointPosition(j, v)
                b.step()
                time.sleep(0.0002)
        for j, v in zip(r4j, [math.radians(x) for x in pts[-1]]):
            sim.setJointPosition(j, v)
        b.step()

    b.start_simulation()
    time.sleep(0.3)
    try:
        for label, fwd, hold in FLOW:
            print(f"--- {label} ---")
            for f in fwd:
                replay(load(f))
            if hold == "press":
                for _ in range(10):
                    b.step()
                print("  [锁付按压 0.5s]")
    finally:
        try:
            for j, mv in zip(r4j, orig_maxvel):
                sim.setObjectFloatParam(j, sim.jointfloatparam_maxvel, mv)
        except Exception:
            pass
        try:
            for j in r4j:
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
    print("完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
