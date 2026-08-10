#!/usr/bin/env python3
"""键盘微调 R3 第6关节

按键:
  1 = j6 +5°    2 = j6 -5°
  3 = j6 +1°    4 = j6 -1°
  q = 退出
每次调整后打印当前 j6 角度和 pad 连线方向(判断是否垂直 wall)。
"""
import sys
import time
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim_bridge.coppelia_client import SimBridge


def main() -> int:
    b = SimBridge(request_timeout=20.0)
    for _ in range(10):
        if b.connect():
            break
        time.sleep(2)
    if not b._connected:
        print("connect failed:", b.last_error)
        return 1
    sim = b._client.require("sim")
    r3j = b.get_robot_joint_handles("R3")
    r3 = b.get_object_handle("R3")

    if sim.getSimulationState() != 0:
        b.stop_simulation()
        time.sleep(0.5)
    b.set_stepping(True)
    b.start_simulation()
    time.sleep(0.3)

    def pad_dir():
        pad = {}
        for o in sim.getObjectsInTree(r3, sim.handle_all, 0):
            a = sim.getObjectAlias(o)
            if a in ("R3T_left_inner_rubber_pad", "R3T_right_inner_rubber_pad"):
                pad[a] = sim.getObjectPosition(o, -1)
        v = [pad["R3T_right_inner_rubber_pad"][i] - pad["R3T_left_inner_rubber_pad"][i]
             for i in range(3)]
        return v

    current = [sim.getJointPosition(j) for j in r3j]
    j6 = current[5]

    def apply():
        nonlocal current
        target = list(current)
        target[5] = j6
        for j, v in zip(r3j, target):
            sim.setJointPosition(j, v)
        for _ in range(8):
            b.step()
        time.sleep(0.1)
        current = [sim.getJointPosition(j) for j in r3j]
        j6_deg = math.degrees(current[5])
        v = pad_dir()
        ang = math.degrees(math.atan2(v[0], abs(v[1]))) if abs(v[1]) > 1e-9 else 0.0
        print(f"  j6={j6_deg:8.2f}°   pad连线 x={v[0]:+.4f} y={v[1]:+.4f} (偏角 {ang:+.1f}°)")

    print("=" * 60)
    print("R3 j6 微调:  1=+5°  2=-5°  3=+1°  4=-1°  q=退出")
    print("=" * 60)
    apply()
    try:
        while True:
            k = input("> ").strip().lower()
            if k == "q":
                break
            if k == "1":
                j6 += math.radians(5)
            elif k == "2":
                j6 -= math.radians(5)
            elif k == "3":
                j6 += math.radians(1)
            elif k == "4":
                j6 -= math.radians(1)
            else:
                print("  按键: 1/2/3/4/q")
                continue
            apply()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
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
    print("退出, R3 已复位")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
