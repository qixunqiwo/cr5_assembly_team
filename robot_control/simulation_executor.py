"""Scheduler adapter for the validated five-arm CoppeliaSim motion cycle."""

from __future__ import annotations

import threading
import time
from typing import Callable, Mapping, Optional

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import RobotState, RobotStatus, Task, TaskResult, TaskStatus
from robot_control.r1_motion import R1_BOX_PLACED, R1_TERMINAL_PLACED
from robot_control.r2_motion import R2_PCB_PLACED
from robot_control.r3_motion import R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION
from robot_control.r4_motion import R4_SCREW_DONE
from robot_control.r5_motion import R5_SORT_DEFECT_DONE, R5_SORT_GOOD_DONE
from robot_control.robot_executor import RobotExecutor, COORDINATED_CYCLE
from sim_bridge.coppelia_client import SimBridge


FRONT_HALF_ACTIONS = (
    R1_BOX_PLACED,
    R2_PCB_PLACED,
    R1_TERMINAL_PLACED,
    R3_MODULE_PLACED,
    R3_PRODUCT_TO_INSPECTION,
)
BACK_HALF_ACTIONS = {
    R4_SCREW_DONE,
    R5_SORT_GOOD_DONE,
    R5_SORT_DEFECT_DONE,
}


class SimulationCellExecutor(IRobotExecutor):
    """Expose R1-R5 motion plus the virtual camera through one interface.

    This executor controls only objects in the open CoppeliaSim scene.  It has
    no ROS driver, controller IP address, or physical-robot transport.
    """

    def __init__(
        self,
        bridge: SimBridge,
        quality_by_order: Optional[Mapping[str, str]] = None,
        default_quality: str = "OK",
        speed_deg_s: float = 50.0,
        hold_seconds: float = 0.25,
        front_half_only: bool = True,
    ):
        self.bridge = bridge
        self.motion = RobotExecutor(
            sim_bridge=bridge,
            speed_deg_s=speed_deg_s,
            hold_seconds=hold_seconds,
        )
        self.quality_by_order = {
            str(key): str(value).strip().upper()
            for key, value in (quality_by_order or {}).items()
        }
        self.default_quality = str(default_quality).strip().upper()
        if self.default_quality not in {"OK", "NG"}:
            raise ValueError("default_quality must be OK or NG")
        self.front_half_only = bool(front_half_only)
        self._camera = RobotState(robot_id="CAMERA")
        self._camera_lock = threading.RLock()
        self._front_half_lock = threading.RLock()
        self._front_half_results_by_order: dict[str, dict[str, TaskResult]] = {}
        self._front_half_details_by_order: dict[str, dict] = {}
        self._prepared = False

    @property
    def prepared(self) -> bool:
        return self._prepared

    def prepare_cycle(self) -> dict:
        """Validate plans, start deterministic stepping and enter READY."""
        self.bridge.set_visual_owner("executor")
        # The signal is consumed by the embedded product-stage script on the
        # first deterministic simulation step performed during READY.
        self.bridge.set_string_signal("cell_product_state", "reset")
        evidence = self.motion.prepare_cycle(
            quality="good",
            preload_both_r5=not self.front_half_only,
            preposition_front_half=False,
            front_half_only=self.front_half_only,
        )
        # CoppeliaSim clears transient string signals when a simulation starts.
        # Re-assert ownership after READY so later scheduler completion signals
        # cannot create duplicate template products on top of the real parts.
        self.bridge.set_visual_owner("executor")
        self._prepared = True
        return evidence

    def _quality_for(self, order_id: str) -> str:
        quality = self.quality_by_order.get(order_id, self.default_quality)
        return quality if quality in {"OK", "NG"} else self.default_quality

    @staticmethod
    def _task_action(task: Task) -> str:
        known = set(FRONT_HALF_ACTIONS) | BACK_HALF_ACTIONS | {COORDINATED_CYCLE}
        for candidate in (
            task.scene_command,
            task.target_point,
            task.process,
            task.task_id,
        ):
            normalized = str(candidate).strip().upper()
            if normalized in known:
                return normalized
        return ""

    def _execute_coordinated_cycle(self, task: Task) -> TaskResult:
        """触发五臂完整协调 (订单首个装配任务)."""
        from robot_control.coordinated_engine import CoordinatedEngine

        started = time.time()
        engine = CoordinatedEngine(bridge=self.bridge)
        result = engine.run_cycle(quality="good")
        if result.get("status") == "ok":
            return TaskResult(
                task_id=task.task_id,
                robot_id="R1",
                status=TaskStatus.FINISHED.value,
                start_time=started,
                end_time=time.time(),
                message=f"coordinated cycle completed; {result.get('message', '')[:200]}",
            )
        return self._failed_result(
            task, f"coordinated cycle failed: {result.get('message')}"
        )

    @staticmethod
    def _finished_result(task: Task, message: str) -> TaskResult:
        return TaskResult(
            task_id=task.task_id,
            robot_id=(task.available_robots[0] if task.available_robots else ""),
            status=TaskStatus.FINISHED.value,
            message=message,
        )

    @staticmethod
    def _failed_result(task: Task, message: str) -> TaskResult:
        now = time.time()
        return TaskResult(
            task_id=task.task_id,
            robot_id=(task.available_robots[0] if task.available_robots else ""),
            status=TaskStatus.FAILED.value,
            start_time=now,
            end_time=now,
            message=message,
        )

    @staticmethod
    def _copy_front_half_result(task: Task, result: TaskResult) -> TaskResult:
        metrics = dict(result.metrics)
        metrics["coordinated_front_half_replay"] = True
        return TaskResult(
            task_id=task.task_id,
            robot_id=result.robot_id,
            status=result.status,
            start_time=result.start_time,
            end_time=result.end_time,
            message=result.message,
            metrics=metrics,
        )

    def _cache_front_half_records(
        self,
        order_id: str,
        front: dict,
    ) -> dict[str, TaskResult]:
        cache: dict[str, TaskResult] = {}
        for record in front.get("tasks", []):
            task_info = record.get("task", {})
            action = str(task_info.get("target_point", "")).strip().upper()
            if action not in FRONT_HALF_ACTIONS:
                continue
            result_info = record.get("result", {})
            available = task_info.get("available_robots", [""])
            fallback_robot = available[0] if available else ""
            metrics = {
                "coordinated_front_half": True,
                "front_half_order": front.get("front_half_order", "terminal_first"),
                "handoff_ready_for_r4": bool(front.get("handoff_ready_for_r4")),
                "handoff_state": front.get("handoff_state", ""),
                "source_task_id": result_info.get("task_id", ""),
                "motion_timing": record.get("motion_timing", {}),
            }
            cache[action] = TaskResult(
                task_id=str(result_info.get("task_id", "")),
                robot_id=str(result_info.get("robot_id", fallback_robot)),
                status=str(result_info.get("status", TaskStatus.FAILED.value)),
                start_time=float(result_info.get("start_time", 0.0)),
                end_time=float(result_info.get("end_time", 0.0)),
                message=str(result_info.get("message", "")),
                metrics=metrics,
            )
        if front.get("status") != "finished":
            message = f"coordinated front half failed: {front.get('errors', {})}"
            now = time.time()
            for action in FRONT_HALF_ACTIONS:
                cache.setdefault(
                    action,
                    TaskResult(
                        task_id=f"{order_id}-{action}",
                        robot_id="R1/R2/R3",
                        status=TaskStatus.FAILED.value,
                        start_time=now,
                        end_time=now,
                        message=message,
                        metrics={
                            "coordinated_front_half": True,
                            "handoff_ready_for_r4": False,
                        },
                    ),
                )
        self._front_half_results_by_order[order_id] = cache
        self._front_half_details_by_order[order_id] = front
        return cache

    def _execute_front_half_task(self, task: Task, action: str) -> TaskResult:
        if not self._prepared:
            return self._failed_result(
                task,
                "CoppeliaSim motion cycle was not prepared",
            )
        with self._front_half_lock:
            cache = self._front_half_results_by_order.get(task.order_id)
            if cache is None:
                quality = self._quality_for(task.order_id)
                front = self.motion.execute_coordinated_front_half(
                    quality=quality,
                    order_id=task.order_id,
                )
                cache = self._cache_front_half_records(task.order_id, front)
            result = cache.get(action)
            if result is None:
                return self._failed_result(
                    task,
                    f"coordinated front half produced no record for {action}",
                )
            return self._copy_front_half_result(task, result)

    def execute_task(self, task: Task) -> TaskResult:
        action = self._task_action(task)
        if action == COORDINATED_CYCLE:
            # 五臂完整协调 (app 下发 COORDINATED_CYCLE 任务)
            from robot_control.coordinated_engine import CoordinatedEngine

            started = time.time()
            engine = CoordinatedEngine(bridge=self.bridge)
            result = engine.run_cycle(quality="good")
            if result.get("status") == "ok":
                return TaskResult(
                    task_id=task.task_id,
                    robot_id="R1",
                    status=TaskStatus.FINISHED.value,
                    start_time=started,
                    end_time=time.time(),
                    message=f"coordinated cycle completed; {result.get('message', '')[:200]}",
                )
            return self._failed_result(
                task, f"coordinated cycle failed: {result.get('message')}"
            )
        if task.process != "inspect" and (
            not task.available_robots or task.available_robots[0] != "CAMERA"
        ):
            if action in FRONT_HALF_ACTIONS or action in BACK_HALF_ACTIONS:
                # 五臂协调模式: 首个装配任务(R1_BOX_PLACED)触发完整协调,
                # 其余任务由协调已覆盖, 直接完成
                if action == R1_BOX_PLACED:
                    return self._execute_coordinated_cycle(task)
                return self._finished_result(
                    task,
                    f"{action} covered by coordinated cycle",
                )
            if not self._prepared:
                return self._failed_result(
                    task,
                    "CoppeliaSim motion cycle was not prepared",
                )
            return self.motion.execute_task(task)

        started = time.time()
        with self._camera_lock:
            if self._camera.status == RobotStatus.FAULT.value:
                return TaskResult(
                    task_id=task.task_id,
                    robot_id="CAMERA",
                    status=TaskStatus.FAILED.value,
                    start_time=started,
                    end_time=time.time(),
                    message="CAMERA is in fault state",
                )
            self._camera.status = RobotStatus.BUSY.value
            self._camera.current_task = task.task_id
        try:
            if self.front_half_only:
                handoff = self._front_half_details_by_order.get(task.order_id, {})
                if not handoff.get("handoff_ready_for_r4"):
                    return TaskResult(
                        task_id=task.task_id,
                        robot_id="CAMERA",
                        status=TaskStatus.FAILED.value,
                        start_time=started,
                        end_time=time.time(),
                        message="front-half handoff is not ready for inspection",
                    )
                return TaskResult(
                    task_id=task.task_id,
                    robot_id="CAMERA",
                    status=TaskStatus.FINISHED.value,
                    start_time=started,
                    end_time=time.time(),
                    message=(
                        "front-half-only checkpoint reached; camera/R4/R5 "
                        "back half deferred"
                    ),
                    metrics={
                        "front_half_only": True,
                        "handoff_ready_for_r4": True,
                        "handoff_state": handoff.get("handoff_state", ""),
                    },
                )
            quality = self._quality_for(task.order_id)
            self.bridge.send_quality_result(quality)
            return TaskResult(
                task_id=task.task_id,
                robot_id="CAMERA",
                status=TaskStatus.FINISHED.value,
                start_time=started,
                end_time=time.time(),
                message=f"virtual camera inspection completed: {quality}",
                quality_result=quality,
            )
        finally:
            with self._camera_lock:
                if self._camera.status != RobotStatus.FAULT.value:
                    self._camera.status = RobotStatus.IDLE.value
                self._camera.current_task = None
                self._camera.completed_tasks += 1

    def execute_task_async(
        self, task: Task, callback: Callable[[TaskResult], None]
    ) -> None:
        threading.Thread(
            target=lambda: callback(self.execute_task(task)),
            name=f"simulation-task-{task.task_id}",
            daemon=True,
        ).start()

    def move_to_point(self, robot_id: str, point_name: str) -> bool:
        return self.motion.move_to_point(robot_id, point_name)

    def gripper_open(self, robot_id: str) -> bool:
        return self.motion.gripper_open(robot_id)

    def gripper_close(self, robot_id: str) -> bool:
        return self.motion.gripper_close(robot_id)

    def screw_execute(self, robot_id: str, point_name: str) -> bool:
        return self.motion.screw_execute(robot_id, point_name)

    def robot_home(self, robot_id: str) -> bool:
        return self.motion.robot_home(robot_id)

    def get_robot_states(self) -> list[RobotState]:
        states = self.motion.get_robot_states()
        with self._camera_lock:
            camera = RobotState(
                robot_id=self._camera.robot_id,
                status=self._camera.status,
                current_task=self._camera.current_task,
                position=self._camera.position,
                utilization=self._camera.utilization,
                completed_tasks=self._camera.completed_tasks,
            )
        return states + [camera]

    def set_robot_fault(self, robot_id: str) -> None:
        if robot_id == "CAMERA":
            with self._camera_lock:
                self._camera.status = RobotStatus.FAULT.value
            return
        self.motion.set_robot_fault(robot_id)

    def clear_robot_fault(self, robot_id: str) -> None:
        if robot_id == "CAMERA":
            with self._camera_lock:
                self._camera.status = RobotStatus.IDLE.value
            return
        self.motion.clear_robot_fault(robot_id)


__all__ = ["SimulationCellExecutor"]
