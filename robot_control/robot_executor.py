"""Team-facing real robot executor for the five-CR5A simulation cell."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import RobotState, RobotStatus, Task, TaskResult, TaskStatus
from robot_control.r1_motion import (
    PLAN_PATH,
    R1_ACTIONS,
    R1_BOX_PLACED,
    R1_COMPLETE_CYCLE,
    R1_TERMINAL_PLACED,
    R1MotionController,
    load_r1_plan,
)
from robot_control.r2_motion import (
    R2_ACTIONS,
    R2_PCB_PLACED,
    R2MotionController,
)
from robot_control.r3_motion import (
    R3_ACTIONS,
    R3_MODULE_PLACED,
    R3_PRODUCT_TRANSFER_CLEARANCE,
    R3_PRODUCT_TO_INSPECTION,
    R3MotionController,
)
from robot_control.r4_motion import (
    R4_ACTIONS,
    R4_SCREW_DONE,
    R4MotionController,
)
from robot_control.r5_motion import (
    R5_ACTIONS,
    R5_SORT_DEFECT_DONE,
    R5_SORT_GOOD_DONE,
    R5MotionController,
)
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import ROBOT_IDS, SENSORS, normalize_robot_id


COORDINATED_CYCLE = "COORDINATED_CYCLE"
SUPPORTED_ACTIONS = R1_ACTIONS | R2_ACTIONS | R3_ACTIONS | R4_ACTIONS | R5_ACTIONS | {COORDINATED_CYCLE}
ACTION_ROBOTS = {
    COORDINATED_CYCLE: "R1",
    **{action: "R1" for action in R1_ACTIONS},
    **{action: "R2" for action in R2_ACTIONS},
    **{action: "R3" for action in R3_ACTIONS},
    **{action: "R4" for action in R4_ACTIONS},
    **{action: "R5" for action in R5_ACTIONS},
}


class RobotExecutor(IRobotExecutor):
    """Execute validated tasks without changing the shared interface contract.

    The current five-arm visual process actions are implemented for R1-R5.
    Unsupported robot/task combinations return ``failed``; they are never
    reported as successful placeholders.
    """

    def __init__(
        self,
        sim_bridge: Optional[SimBridge] = None,
        plan_path: Path = PLAN_PATH,
        speed_deg_s: float = 50.0,
        hold_seconds: float = 1.0,
        motion_controller_factory: Callable[..., R1MotionController] = (
            R1MotionController
        ),
        r2_motion_controller_factory: Callable[..., R2MotionController] = (
            R2MotionController
        ),
        r3_motion_controller_factory: Callable[..., R3MotionController] = (
            R3MotionController
        ),
        r4_motion_controller_factory: Callable[..., R4MotionController] = (
            R4MotionController
        ),
        r5_motion_controller_factory: Callable[..., R5MotionController] = (
            R5MotionController
        ),
    ):
        self._bridge = sim_bridge or SimBridge()
        self._plan_path = Path(plan_path)
        self._speed_deg_s = float(speed_deg_s)
        self._hold_seconds = min(1.0, max(0.0, float(hold_seconds)))
        self._motion_controller_factory = motion_controller_factory
        self._r2_motion_controller_factory = r2_motion_controller_factory
        self._r3_motion_controller_factory = r3_motion_controller_factory
        self._r4_motion_controller_factory = r4_motion_controller_factory
        self._r5_motion_controller_factory = r5_motion_controller_factory
        self._state_lock = threading.RLock()
        self._execution_lock = threading.Lock()
        self._assembly_lock = threading.Lock()
        self._inspection_lock = threading.Lock()
        self._controllers: Dict[str, Any] = {}
        self._ready = False
        self._last_error = ""
        self._fast_demo = False
        self._robots: Dict[str, RobotState] = {
            robot_id: RobotState(robot_id=robot_id)
            for robot_id in ROBOT_IDS
        }

    @property
    def last_error(self) -> str:
        return self._last_error

    def set_fast_demo(self, enabled: bool) -> None:
        """开启/关闭快速演示模式（所有机械臂大步进直设，速度快且顺）。"""
        self._fast_demo = bool(enabled)
        for controller in self._controllers.values():
            setter = getattr(controller, "set_fast_mode", None)
            if setter is not None:
                setter(self._fast_demo)
            r5_setter = getattr(controller, "set_flow_demo_speed", None)
            if r5_setter is not None:
                r5_setter(500.0 if self._fast_demo else None)

    @staticmethod
    def _resolve_action(task: Task) -> Optional[str]:
        for candidate in (
            task.scene_command,
            task.target_point,
            task.process,
            task.task_id,
        ):
            normalized = str(candidate).strip().upper()
            if normalized in SUPPORTED_ACTIONS:
                return normalized
        return None

    @staticmethod
    def _task_robot(task: Task, action: Optional[str]) -> str:
        if task.available_robots:
            return normalize_robot_id(task.available_robots[0])
        if action in ACTION_ROBOTS:
            return ACTION_ROBOTS[action]
        raise ValueError("task has no available robot")

    @staticmethod
    def _result(
        task: Task,
        robot_id: str,
        status: str,
        start_time: float,
        message: str,
    ) -> TaskResult:
        return TaskResult(
            task_id=task.task_id,
            robot_id=robot_id,
            status=status,
            start_time=start_time,
            end_time=time.time(),
            message=message,
        )

    def _connect(self) -> None:
        if self._bridge.is_connected():
            return
        host = getattr(self._bridge, "host", None)
        port = getattr(self._bridge, "port", None)
        connected = (
            self._bridge.connect(host, port)
            if host is not None and port is not None
            else self._bridge.connect()
        )
        if not connected:
            raise RuntimeError(
                self._bridge.last_error or "cannot connect to CoppeliaSim"
            )

    def _controller_for(self, robot_id: str) -> Any:
        controller = self._controllers.get(robot_id)
        if controller is not None:
            return controller
        if robot_id == "R1":
            controller = self._motion_controller_factory(
                self._bridge,
                plan_path=self._plan_path,
                assembly_lock=self._assembly_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R2":
            controller = self._r2_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                assembly_lock=self._assembly_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R3":
            controller = self._r3_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                assembly_lock=self._assembly_lock,
                inspection_lock=self._inspection_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R4":
            controller = self._r4_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                inspection_lock=self._inspection_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R5":
            controller = self._r5_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                inspection_lock=self._inspection_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        else:
            raise ValueError(f"unsupported robot controller: {robot_id}")
        self._controllers[robot_id] = controller
        return controller

    def _bootstrap_scene_kinematics(self) -> bool:
        """Run the freshly loaded scene once so model joints drive their links.

        Imported CR5 models install part of their joint behaviour from child
        script ``sysCall_init`` callbacks.  Before the first simulation step,
        setting a stopped joint can therefore change its scalar value without
        updating the visible link chain, which makes stopped-scene FK/IK
        preparation invalid.  A short start/stop cycle initializes those
        callbacks and CoppeliaSim then restores the scene's initial state.
        """
        sim = getattr(self._bridge, "sim", None)
        if sim is None or not hasattr(sim, "getSimulationState"):
            return False
        stopped = getattr(sim, "simulation_stopped", None)
        if stopped is None:
            return False
        if sim.getSimulationState() != stopped:
            raise RuntimeError("motion preparation requires a stopped scene")
        if not self._bridge.start_simulation():
            raise RuntimeError(
                self._bridge.last_error or "cannot initialize CoppeliaSim scene"
            )
        for _ in range(2):
            if not self._bridge.step():
                raise RuntimeError(
                    self._bridge.last_error
                    or "cannot step CoppeliaSim scene initialization"
                )
        if not self._bridge.stop_simulation():
            raise RuntimeError(
                self._bridge.last_error
                or "cannot stop CoppeliaSim after scene initialization"
            )
        return True

    def _relocate_camera_station(self) -> dict[str, Any]:
        """Move the complete fixed-camera station outside every arm envelope.

        The camera is only a visual prop in the current simulation workflow;
        inspection quality comes from the software order input.  Keeping the
        lens over the inspection table forced R3 to tilt or take an excessive
        joint-space detour.  Move the common station parent so the lens,
        column, brackets, and view marker stay together and cannot obstruct
        any R1--R5 motion.
        """
        sim = getattr(self._bridge, "sim", None)
        if sim is None:
            return {"camera_station_relocated": False}
        station = sim.getObject(SENSORS["FIXED_VISION_CAMERA"])
        station_position = (1.80, 1.50, 0.0)
        sim.setObjectPosition(station, -1, list(station_position))
        return {
            "camera_station_relocated": True,
            "camera_station_position_m": list(station_position),
            "camera_optical_head_moved": True,
        }

    @staticmethod
    def _quality_action(quality: str) -> str:
        normalized = str(quality).strip().lower()
        if normalized in {"good", "ok"}:
            return R5_SORT_GOOD_DONE
        if normalized in {"defect", "ng"}:
            return R5_SORT_DEFECT_DONE
        raise ValueError("quality must be good/OK or defect/NG")

    @staticmethod
    def _grasp_note(robot_id: str) -> str:
        if robot_id == "R2":
            return "contact-aligned simulated suction; attach pose preserved"
        if robot_id in {"R3", "R5"}:
            return "contact-aligned simulated grip; attach pose preserved"
        if robot_id == "R4":
            return "runtime visual screwdriver; physical torque not validated"
        return "contact-aligned simulated grip; attach pose preserved"

    def _preposition_robots(
        self,
        r2: Any,
        r3: Any,
        r4: Any | None,
        r5: Any | None,
        r5_action: str,
        preposition_front_half: bool = False,
    ) -> float:
        """Optionally step front-half robots to their pick-APP configs.

        Must be called **after** ``enter_ready()`` because
        ``sim.setJointPosition`` in stopped mode is discarded by
        ``startSimulation``.  We use ``setJointTargetPosition`` with the
        already-held stepping to converge each robot without advancing
        simulation time for the subsequent task loop.

        Returns the additional simulation time consumed.
        """
        sim = getattr(self._bridge, "sim", None)
        if sim is None:
            return 0.0  # mock / fake bridge used in tests
        entries = []
        if preposition_front_half:
            # (robot_id, controller, action_key, segment_name)
            # R2 is intentionally excluded for the coordinated front half:
            # waiting at R2_PCB_PICK_APP blocks R1's box transfer. R2 now
            # starts from zero, picks the PCB, moves to a near safe wait, and
            # only enters the assembly area after R1 clears the interference.
            entries.append(
                ("R3", r3, R3_MODULE_PLACED, "initial_to_pick_app")
            )
            if r4 is not None:
                entries.append(("R4", r4, None, "home_to_wait"))
            if r5 is not None:
                entries.append(("R5", r5, r5_action, "home_to_wait"))
        total_sim_time = 0.0
        for robot_id, controller, action_key, segment_name in entries:
            prepared = getattr(controller, "_prepared_paths", None)
            if prepared is None:
                continue
            # R3/R5 store action -> paths; R2/R4 store paths directly.
            paths = (
                prepared.get(action_key, {})
                if action_key is not None
                else prepared
            )
            segment = paths.get(segment_name) if isinstance(paths, dict) else None
            if not segment:
                continue
            config = segment[-1]
            joints = self._bridge.get_robot_joint_handles(robot_id)

            # Enable motion for kinematic joints that have maxVel == 0.
            max_vel = math.radians(60.0)
            original_velocities: list[float] = []
            for joint in joints:
                original_velocities.append(
                    sim.getObjectFloatParam(joint, sim.jointfloatparam_maxvel)
                )
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, max_vel
                )
            for idx, joint in enumerate(joints):
                sim.setJointTargetPosition(joint, float(config[idx]))

            # Converge within the already-held stepping loop.
            sim_before = float(sim.getSimulationTime())
            for _ in range(300):
                if not self._bridge.step():
                    raise RuntimeError(
                        self._bridge.last_error
                        or f"{robot_id} pre-position step failed"
                    )
                current = [
                    float(sim.getJointPosition(joint)) for joint in joints
                ]
                errors = [
                    abs(current[i] - config[i]) for i in range(len(config))
                ]
                if max(errors) <= math.radians(0.12):
                    break
            else:
                raise RuntimeError(
                    f"{robot_id} did not converge to pre-position target"
                )
            sim_after = float(sim.getSimulationTime())
            total_sim_time += sim_after - sim_before

            # Restore original maxVel so later execute() can set its own.
            for joint, original in zip(joints, original_velocities):
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, original
                )

            setter = getattr(controller, "set_pre_positioned", None)
            if setter is not None:
                setter(action_key, list(config))

        return total_sim_time

    def _restore_idle_robots_at_ready(self) -> None:
        """Restore non-R1 robots after stopped-scene IK preparation."""
        sim = getattr(self._bridge, "sim", None)
        if sim is None:
            return
        for robot_id in ("R2", "R3", "R4", "R5"):
            for joint in self._bridge.get_robot_joint_handles(robot_id):
                sim.setJointPosition(joint, 0.0)
                sim.setJointTargetPosition(joint, 0.0)
        if not self._bridge.step():
            raise RuntimeError(
                self._bridge.last_error
                or "cannot apply READY joint-state restoration"
            )
        for robot_id in ("R2", "R3", "R4", "R5"):
            actual = self._bridge.get_robot_joint_positions(robot_id)
            if max(abs(value) for value in actual) > math.radians(0.1):
                raise RuntimeError(
                    f"{robot_id} did not restore its initial READY posture"
                )

    def prepare_cycle(
        self,
        quality: str = "good",
        preload_both_r5: bool = False,
        preposition_front_half: bool = False,
        front_half_only: bool = False,
    ) -> dict[str, Any]:
        """Precompute deterministic paths, then enter the resident READY state."""
        selected_r5_action = self._quality_action(quality)
        started = time.monotonic()
        self._ready = False
        evidence: list[dict[str, Any]] = []
        with self._execution_lock:
            self._connect()
            scene_kinematics_initialized = self._bootstrap_scene_kinematics()
            camera_station = self._relocate_camera_station()
            r1 = self._controller_for("R1")
            r2 = self._controller_for("R2")
            r3 = self._controller_for("R3")
            r4 = None if front_half_only else self._controller_for("R4")
            r5 = None if front_half_only else self._controller_for("R5")

            evidence.append(r1.prepare(R1_BOX_PLACED))
            evidence.append(r2.prepare(R2_PCB_PLACED))
            evidence.append(r3.prepare(R3_MODULE_PLACED))
            evidence.append(r3.prepare(R3_PRODUCT_TO_INSPECTION))
            if not front_half_only:
                assert r4 is not None
                assert r5 is not None
                evidence.append(r4.prepare(R4_SCREW_DONE))
                r5_actions = [selected_r5_action]
                if preload_both_r5:
                    r5_actions = [R5_SORT_GOOD_DONE, R5_SORT_DEFECT_DONE]
                for action in r5_actions:
                    evidence.append(r5.prepare(action))

            # simIK preparation writes temporary model configurations.
            # Re-run the scene initialization cycle after every path has been
            # cached so script-driven links and tool geometry are restored to
            # the exact initial state before READY starts real replay.
            post_planning_scene_reset = self._bootstrap_scene_kinematics()
            camera_station = self._relocate_camera_station()

            for controller in (r1, r2, r3):
                controller.set_continuous_stepping(True)
            if r4 is not None:
                r4.set_continuous_stepping(True)
            if r5 is not None:
                r5.set_continuous_stepping(False)

            ready_state = r1.enter_ready()
            self._restore_idle_robots_at_ready()
            ready_state["scene_kinematics_initialized"] = (
                scene_kinematics_initialized
            )
            ready_state["post_planning_scene_reset"] = (
                post_planning_scene_reset
            )
            ready_state.update(camera_station)

            # Optionally pre-position front-half robots now that the
            # simulation is running with stepping held. The coordinated
            # front-half runner leaves this disabled so R3 cannot visibly
            # move before the PCB-release handoff.
            preposition_sim_s = self._preposition_robots(
                r2,
                r3,
                r4,
                r5,
                selected_r5_action,
                preposition_front_half=preposition_front_half,
            )
            ready_state["preposition_simulation_time_s"] = preposition_sim_s
            ready_state["front_half_prepositioned"] = bool(
                preposition_front_half
            )

            self._ready = True

        path_points = sum(
            sum(int(count) for count in record.get("path_points", {}).values())
            for record in evidence
        )
        return {
            "ready": True,
            "quality_action": selected_r5_action,
            "preloaded_both_r5": bool(preload_both_r5),
            "front_half_only": bool(front_half_only),
            "controllers": evidence,
            "path_points_total": path_points,
            "ready_state": ready_state,
            "prepare_wall_s": time.monotonic() - started,
        }

    @staticmethod
    def _coordinated_task(
        action: str,
        robot_id: str,
        index: int,
        order_id: str,
        process: str = "assemble",
        area: str = "assembly_area",
    ) -> Task:
        return Task(
            task_id=f"{order_id}-COORD-{index:02d}-{action}",
            order_id=order_id,
            product_type="A",
            process=process,
            target_area=area,
            target_point=action,
            available_robots=[robot_id],
        )

    def execute_coordinated_front_half(
        self,
        quality: str = "good",
        order_id: str = "FIVE-ARM-DEMO",
    ) -> dict[str, Any]:
        """Run the R1/R2/R3 front half through the corrected controllers."""
        deferred_quality_action = self._quality_action(quality)
        if not self._ready:
            self.prepare_cycle(quality=quality, front_half_only=True)

        task_entries = [
            (R1_BOX_PLACED, "R1", "assemble", "assembly_area"),
            (R2_PCB_PLACED, "R2", "assemble", "assembly_area"),
            (R1_TERMINAL_PLACED, "R1", "assemble", "assembly_area"),
            (R3_MODULE_PLACED, "R3", "assemble", "assembly_area"),
            (
                R3_PRODUCT_TO_INSPECTION,
                "R3",
                "transfer",
                "inspection_screw_area",
            ),
        ]
        tasks: dict[str, Task] = {}
        for index, entry in enumerate(task_entries, start=1):
            action, robot_id, process, area = entry
            tasks[action] = self._coordinated_task(
                action, robot_id, index, order_id, process, area
            )

        with self._execution_lock:
            self._connect()
            r1 = self._controller_for("R1")
            r2 = self._controller_for("R2")
            r3 = self._controller_for("R3")
            for controller in (r1, r2, r3):
                setter = getattr(controller, "set_continuous_stepping", None)
                if setter is not None:
                    setter(True)
            for controller in (r2, r3):
                setter = getattr(controller, "set_coordinated_mode", None)
                if setter is not None:
                    setter(True)

            records: dict[str, dict[str, Any]] = {}
            errors: dict[str, str] = {}

            def run_action(action: str, controller: Any) -> None:
                task = tasks[action]
                robot_id = task.available_robots[0]
                start_wall = time.time()
                start_sim = float(self._bridge.sim.getSimulationTime())
                with self._state_lock:
                    state = self._robots[robot_id]
                    state.status = RobotStatus.BUSY.value
                    state.current_task = task.task_id
                status = TaskStatus.FINISHED.value
                message = ""
                details: dict[str, Any] | None = None
                try:
                    details = controller.execute(action)
                    message = (
                        f"{action} completed ({self._grasp_note(robot_id)}); "
                        f"{details}"
                    )
                except Exception as exc:
                    status = TaskStatus.FAILED.value
                    message = str(exc)
                    errors[action] = message
                finally:
                    end_wall = time.time()
                    end_sim = float(self._bridge.sim.getSimulationTime())
                    result = TaskResult(
                        task_id=task.task_id,
                        robot_id=robot_id,
                        status=status,
                        start_time=start_wall,
                        end_time=end_wall,
                        message=message,
                    )
                    with self._state_lock:
                        state = self._robots[robot_id]
                        if state.status != RobotStatus.FAULT.value:
                            state.status = RobotStatus.IDLE.value
                        state.current_task = None
                        if status == TaskStatus.FINISHED.value:
                            state.completed_tasks += 1
                            if action == R1_BOX_PLACED:
                                state.position = "R1_TERMINAL_PICK_APP"
                            elif action == R2_PCB_PLACED:
                                state.position = "R2_PCB_PICK_APP"
                            elif action == R3_MODULE_PLACED:
                                state.position = (
                                    "R3_TEMP_CLEAR_FOR_R1_TERMINAL_BEFORE_PRODUCT_PICK_APP"
                                )
                            elif action == R3_PRODUCT_TO_INSPECTION:
                                state.position = R3_PRODUCT_TRANSFER_CLEARANCE
                            else:
                                state.position = "home"
                    record = {
                        "task": task.to_dict(),
                        "coordinated_front_half": True,
                        "start_wall_epoch_s": start_wall,
                        "end_wall_epoch_s": end_wall,
                        "start_simulation_time_s": start_sim,
                        "end_simulation_time_s": end_sim,
                        "wall_duration_s": end_wall - start_wall,
                        "simulation_duration_s": max(0.0, end_sim - start_sim),
                        "motion_timing": {
                            "robot_id": robot_id,
                            "motion_detected": status == TaskStatus.FINISHED.value,
                            "monitor_error": (
                                "new-controller front-half direct run"
                            ),
                        },
                        "result": result.to_dict(),
                    }
                    if details is not None:
                        record["controller_details"] = details
                    records[action] = record

            try:
                sequence = [
                    (R1_BOX_PLACED, r1),
                    (R2_PCB_PLACED, r2),
                    (R1_TERMINAL_PLACED, r1),
                    (R3_MODULE_PLACED, r3),
                    (R3_PRODUCT_TO_INSPECTION, r3),
                ]
                for action, controller in sequence:
                    run_action(action, controller)
                    if action in errors:
                        break
            finally:
                r3.set_assembly_entry_wait(R3_MODULE_PLACED, None)
                r1.set_assembly_entry_wait(R1_TERMINAL_PLACED, None)
                for controller in (r2, r3):
                    setter = getattr(controller, "set_coordinated_mode", None)
                    if setter is not None:
                        setter(False)

            ordered_records = [
                records[action]
                for action in (
                    R1_BOX_PLACED,
                    R2_PCB_PLACED,
                    R1_TERMINAL_PLACED,
                    R3_MODULE_PLACED,
                    R3_PRODUCT_TO_INSPECTION,
                )
                if action in records
            ]
            failed_action = next(
                (
                    action
                    for action in (
                        R1_BOX_PLACED,
                        R2_PCB_PLACED,
                        R1_TERMINAL_PLACED,
                        R3_MODULE_PLACED,
                        R3_PRODUCT_TO_INSPECTION,
                    )
                    if action in errors
                ),
                None,
            )
            return {
                "status": "failed" if failed_action else "finished",
                "tasks": ordered_records,
                "failed_action": failed_action,
                "errors": dict(errors),
                "handoff_ready_for_r4": failed_action is None,
                "handoff_state": (
                    R3_PRODUCT_TRANSFER_CLEARANCE if failed_action is None else ""
                ),
                "front_half_order": "terminal_first",
                "controller_runner": True,
                "single_step_runner": False,
                "parallel_controller_threads": False,
                "deferred_quality_action": deferred_quality_action,
                "details": {
                    "status": "failed" if failed_action else "finished",
                    "actions": [
                        R1_BOX_PLACED,
                        R2_PCB_PLACED,
                        R1_TERMINAL_PLACED,
                        R3_MODULE_PLACED,
                        R3_PRODUCT_TO_INSPECTION,
                    ],
                    "handoff_ready_for_r4": failed_action is None,
                    "handoff_state": (
                        R3_PRODUCT_TRANSFER_CLEARANCE
                        if failed_action is None
                        else ""
                    ),
                    "front_half_order": "terminal_first",
                    "controller_runner": True,
                    "single_step_runner": False,
                    "parallel_controller_threads": False,
                    "new_controller_sequence": [
                        R1_BOX_PLACED,
                        R2_PCB_PLACED,
                        R1_TERMINAL_PLACED,
                        R3_MODULE_PLACED,
                        R3_PRODUCT_TO_INSPECTION,
                    ],
                    "deferred_quality_action": deferred_quality_action,
                },
            }

    def execute_task(self, task: Task) -> TaskResult:
        start_time = time.time()
        action = self._resolve_action(task)
        try:
            robot_id = self._task_robot(task, action)
        except (KeyError, ValueError) as exc:
            self._last_error = str(exc)
            fallback = task.available_robots[0] if task.available_robots else ""
            return self._result(
                task,
                fallback,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )

        if action is None:
            self._last_error = (
                f"unsupported task {task.task_id}: expected one of "
                f"{sorted(SUPPORTED_ACTIONS)} in "
                "scene_command/target_point/process/task_id"
            )
            return self._result(
                task,
                robot_id,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )
        assigned_robot = ACTION_ROBOTS[action]
        if robot_id != assigned_robot:
            self._last_error = (
                f"{action} is assigned to {assigned_robot}, not {robot_id}"
            )
            return self._result(
                task,
                robot_id,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )

        with self._state_lock:
            state = self._robots[robot_id]
            if state.status == RobotStatus.FAULT.value:
                self._last_error = f"{robot_id} is in fault state"
                return self._result(
                    task,
                    robot_id,
                    TaskStatus.FAILED.value,
                    start_time,
                    self._last_error,
                )
            if state.status == RobotStatus.BUSY.value:
                self._last_error = f"{robot_id} is already busy"
                return self._result(
                    task,
                    robot_id,
                    TaskStatus.FAILED.value,
                    start_time,
                    self._last_error,
                )
            state.status = RobotStatus.BUSY.value
            state.current_task = task.task_id

        try:
            if action == COORDINATED_CYCLE:
                # 五臂完整协调: 交由 CoordinatedEngine 执行
                from robot_control.coordinated_engine import CoordinatedEngine

                engine = CoordinatedEngine()
                result = engine.run_cycle(quality=task.quality_result or "good")
                if result.get("status") != "ok":
                    raise RuntimeError(
                        f"coordinated cycle failed: {result.get('message')}"
                    )
                with self._state_lock:
                    state = self._robots[robot_id]
                    state.position = "home"
                    state.completed_tasks += 1
                self._last_error = ""
                return self._result(
                    task,
                    robot_id,
                    TaskStatus.FINISHED.value,
                    start_time,
                    f"{COORDINATED_CYCLE} completed; {result.get('message', '')[:200]}",
                )
            with self._execution_lock:
                self._connect()
                controller = self._controller_for(robot_id)
                details = controller.execute(action)
            with self._state_lock:
                state = self._robots[robot_id]
                if action == R1_BOX_PLACED:
                    state.position = "R1_TERMINAL_PICK_APP"
                elif action == R4_SCREW_DONE:
                    state.position = "home"
                elif action == R3_PRODUCT_TO_INSPECTION:
                    state.position = R3_PRODUCT_TRANSFER_CLEARANCE
                else:
                    state.position = "home"
                state.completed_tasks += 1
            self._last_error = ""
            return self._result(
                task,
                robot_id,
                TaskStatus.FINISHED.value,
                start_time,
                f"{action} completed ({self._grasp_note(robot_id)}); {details}",
            )
        except Exception as exc:
            self._last_error = str(exc)
            return self._result(
                task,
                robot_id,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )
        finally:
            with self._state_lock:
                state = self._robots[robot_id]
                if state.status != RobotStatus.FAULT.value:
                    state.status = RobotStatus.IDLE.value
                state.current_task = None

    def execute_task_async(
        self, task: Task, callback: Callable[[TaskResult], None]
    ) -> None:
        def _run() -> None:
            callback(self.execute_task(task))

        threading.Thread(
            target=_run,
            name=f"robot-task-{task.task_id}",
            daemon=True,
        ).start()

    def move_to_point(self, robot_id: str, point_name: str) -> bool:
        """Accept only an idempotent request for the robot's current endpoint.

        Arbitrary point-to-point motion has not yet received path-level
        collision validation, so this method must not fabricate success.
        """
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError as exc:
            self._last_error = str(exc)
            return False
        with self._state_lock:
            current = self._robots[robot_id].position
        aliases = {
            "HOME": "home",
            **{f"{known_robot}_HOME_REF": "home" for known_robot in ROBOT_IDS},
        }
        requested = aliases.get(point_name.strip().upper(), point_name)
        if current == requested:
            self._last_error = ""
            return True
        self._last_error = (
            f"no independently validated path from {current} to {point_name} "
            f"for {robot_id}; use execute_task"
        )
        return False

    def gripper_open(self, robot_id: str) -> bool:
        return self._set_gripper(robot_id, True)

    def gripper_close(self, robot_id: str) -> bool:
        return self._set_gripper(robot_id, False)

    def _set_gripper(self, robot_id: str, opened: bool) -> bool:
        try:
            robot_id = normalize_robot_id(robot_id)
            self._connect()
            result = self._bridge.set_gripper(robot_id, opened)
            if not result:
                self._last_error = self._bridge.last_error
            return result
        except (KeyError, RuntimeError) as exc:
            self._last_error = str(exc)
            return False

    def screw_execute(self, robot_id: str, point_name: str) -> bool:
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError as exc:
            self._last_error = str(exc)
            return False
        if robot_id != "R4" or point_name.strip().upper() not in {
            "R4_SCREW_TCP",
            "R4_SCREW_PRESS",
            R4_SCREW_DONE,
        }:
            self._last_error = (
                "R4 screw execution requires R4_SCREW_TCP, "
                "R4_SCREW_PRESS, or R4_SCREW_DONE"
            )
            return False
        task = Task(
            task_id=f"R4-SCREW-{time.time_ns()}",
            order_id="R4-DIRECT",
            product_type="A",
            process="screw",
            target_area="inspection_screw_area",
            target_point=R4_SCREW_DONE,
            available_robots=["R4"],
        )
        return self.execute_task(task).status == TaskStatus.FINISHED.value

    def robot_home(self, robot_id: str) -> bool:
        return self.move_to_point(robot_id, f"{robot_id.strip().upper()}_HOME_REF")

    def get_robot_states(self) -> List[RobotState]:
        with self._state_lock:
            return [
                RobotState(
                    robot_id=state.robot_id,
                    status=state.status,
                    current_task=state.current_task,
                    position=state.position,
                    utilization=state.utilization,
                    completed_tasks=state.completed_tasks,
                )
                for state in self._robots.values()
            ]

    def set_robot_fault(self, robot_id: str) -> None:
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError:
            return
        with self._state_lock:
            self._robots[robot_id].status = RobotStatus.FAULT.value

    def clear_robot_fault(self, robot_id: str) -> None:
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError:
            return
        with self._state_lock:
            state = self._robots[robot_id]
            state.status = RobotStatus.IDLE.value
            state.current_task = None


__all__ = [
    "RobotExecutor",
    "R1_BOX_PLACED",
    "R1_TERMINAL_PLACED",
    "R1_COMPLETE_CYCLE",
    "R2_PCB_PLACED",
    "R3_MODULE_PLACED",
    "R3_PRODUCT_TO_INSPECTION",
    "R4_SCREW_DONE",
    "R5_SORT_GOOD_DONE",
    "R5_SORT_DEFECT_DONE",
    "load_r1_plan",
]
