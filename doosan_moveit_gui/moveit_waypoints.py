#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, MotionPlanRequest
from control_msgs.action import FollowJointTrajectory

import yaml

# ====== Robot config (너 환경 기준) ======
JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
TIP_LINK = "link_6"

SRV_IK = "/compute_ik"
SRV_PLAN = "/plan_kinematic_path"
SRV_FK = "/compute_fk"
ACTION_FJT = "/dsr_moveit_controller/follow_joint_trajectory"

def clamp01(x: float) -> float:
    try:
        x = float(x)
    except Exception:
        return 0.0
    return max(0.0, min(1.0, x))


def blend_seed_dict(cur: Dict[str, float], last: Dict[str, float], alpha: float) -> Dict[str, float]:
    a = clamp01(alpha)
    out: Dict[str, float] = {}
    for j in JOINT_ORDER:
        c = float(cur.get(j, 0.0))
        l = float(last.get(j, c))
        out[j] = (1.0 - a) * c + a * l
    return out

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class MoveItWaypointRunner(Node):
    def __init__(self):
        super().__init__("moveit_waypoints")

        # joint_states subscriber
        self._last_js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # services
        self.ik_cli = self.create_client(GetPositionIK, SRV_IK)
        self.plan_cli = self.create_client(GetMotionPlan, SRV_PLAN)
        self.fk_cli = self.create_client(GetPositionFK, SRV_FK)

        # action
        self.fjt_ac = ActionClient(self, FollowJointTrajectory, ACTION_FJT)

    def _on_js(self, msg: JointState):
        self._last_js = msg

    def wait_for_joint_states(self, timeout_sec: float = 5.0) -> JointState:
        t0 = time.time()
        while rclpy.ok() and self._last_js is None and (time.time() - t0) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._last_js is None:
            raise RuntimeError("No /joint_states received yet.")
        return self._last_js

    def _extract_joint_positions(self, js: JointState) -> Dict[str, float]:
        mp = {}
        for n, p in zip(js.name, js.position):
            mp[n] = float(p)
        # joint_states 순서가 섞여 올 수 있으니 JOINT_ORDER로 재정렬해서 쓸 거임
        return mp

    def call_fk(self, joint_pos: Dict[str, float], frame: str = "world") -> PoseStamped:
        if not self.fk_cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"FK service not available: {SRV_FK}")

        req = GetPositionFK.Request()
        req.fk_link_names = [TIP_LINK]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = JOINT_ORDER
        req.robot_state.joint_state.position = [joint_pos.get(j, 0.0) for j in JOINT_ORDER]

        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None or res.error_code.val != 1 or len(res.pose_stamped) == 0:
            raise RuntimeError(f"FK failed. error_code={None if res is None else res.error_code.val}")

        ps = res.pose_stamped[0]
        # FK 응답 frame이 world로 오는게 일반적. frame을 억지 변환하진 않고 그대로 씀.
        return ps

    def call_ik(
        self,
        target_pose: PoseStamped,
        seed_joint_pos: Dict[str, float],
        avoid_collisions: bool = False,
        ik_timeout: float = 2.0,
    ) -> Dict[str, float]:
        if not self.ik_cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"IK service not available: {SRV_IK}")

        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = TIP_LINK
        req.ik_request.avoid_collisions = bool(avoid_collisions)

        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.joint_state.name = JOINT_ORDER
        req.ik_request.robot_state.joint_state.position = [seed_joint_pos.get(j, 0.0) for j in JOINT_ORDER]

        req.ik_request.pose_stamped = target_pose
        req.ik_request.timeout.sec = int(ik_timeout)
        req.ik_request.timeout.nanosec = int((ik_timeout - int(ik_timeout)) * 1e9)

        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=max(ik_timeout + 1.0, 2.0))
        res = fut.result()
        if res is None:
            raise RuntimeError("IK call failed: no response")
        if res.error_code.val != 1:
            # -31이면 NO_IK_SOLUTION
            raise RuntimeError(f"IK failed. error_code={res.error_code.val}")

        sol = res.solution.joint_state
        out = {}
        for n, p in zip(sol.name, sol.position):
            out[n] = float(p)
        # 혹시 solver가 순서를 다르게 줄 수도 있으니 JOINT_ORDER 기준으로 완성
        for j in JOINT_ORDER:
            out.setdefault(j, seed_joint_pos.get(j, 0.0))
        return out

    def call_plan(
        self,
        start_joint_pos: Dict[str, float],
        goal_joint_pos: Dict[str, float],
        planning_time: float = 2.0,
        plan_attempts: int = 3,
        vel_scale: float = 0.2,
        acc_scale: float = 0.2,
    ):
        if not self.plan_cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Plan service not available: {SRV_PLAN}")

        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP_NAME

        # start state
        mpr.start_state = RobotState()
        mpr.start_state.joint_state.name = JOINT_ORDER
        mpr.start_state.joint_state.position = [start_joint_pos.get(j, 0.0) for j in JOINT_ORDER]

        # goal constraints: joint constraints로 지정 (가장 안정적)
        c = Constraints()
        c.joint_constraints = []
        for j in JOINT_ORDER:
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = goal_joint_pos.get(j, 0.0)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        mpr.goal_constraints = [c]
        mpr.allowed_planning_time = float(planning_time)
        mpr.num_planning_attempts = int(plan_attempts)

        # scaling
        mpr.max_velocity_scaling_factor = float(vel_scale)
        mpr.max_acceleration_scaling_factor = float(acc_scale)

        req.motion_plan_request = mpr

        fut = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=max(planning_time + 1.0, 2.0))
        res = fut.result()
        if res is None:
            raise RuntimeError("Plan call failed: no response")

        mres = res.motion_plan_response
        if mres.error_code.val != 1:
            raise RuntimeError(f"Planning failed. error_code={mres.error_code.val}")

        jt = mres.trajectory.joint_trajectory
        return jt

    def execute_joint_trajectory(self, joint_traj):
        if not self.fjt_ac.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"Action server not available: {ACTION_FJT}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        send_fut = self.fjt_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=5.0)
        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal rejected")

        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=60.0)
        result = res_fut.result().result
        if result.error_code != 0:
            raise RuntimeError(f"Execution failed. error_code={result.error_code} msg={result.error_string}")
        return True


def load_waypoints_yaml(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def pick_rpy(default_rpy, wp) -> Optional[Tuple[float, float, float]]:
    # waypoint에 rpy가 있으면 그걸 쓰고, 없으면 defaults.rpy 사용
    if "rpy" in wp:
        r = wp["rpy"]
        return (float(r[0]), float(r[1]), float(r[2]))
    if default_rpy is not None:
        return (float(default_rpy[0]), float(default_rpy[1]), float(default_rpy[2]))
    return None


def make_target_pose(
    node: MoveItWaypointRunner,
    frame: str,
    wp: Dict,
    cur_joint_pos: Dict[str, float],
    default_rpy: Optional[List[float]],
) -> PoseStamped:
    # 현재 EE pose (world frame) 얻기
    fk = node.call_fk(cur_joint_pos)

    target = PoseStamped()
    target.header.frame_id = frame

    # 기본: 현재 orientation 유지 (단, rpy가 주어지면 그걸 적용)
    rpy = pick_rpy(default_rpy, wp)
    if rpy is None:
        target.pose.orientation = fk.pose.orientation
    else:
        qx, qy, qz, qw = rpy_to_quat(*rpy)
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

    if "abs" in wp:
        a = wp["abs"]
        target.pose.position.x = float(a["x"])
        target.pose.position.y = float(a["y"])
        target.pose.position.z = float(a["z"])
        return target

    if "rel" in wp:
        d = wp["rel"]
        # rel은 “현재 FK(world)”에서 델타 더하는 단순 버전
        # frame을 world로 쓰는게 제일 안전 (너도 지금 world 기준으로 잘 됨)
        target.pose.position.x = float(fk.pose.position.x + float(d.get("dx", 0.0)))
        target.pose.position.y = float(fk.pose.position.y + float(d.get("dy", 0.0)))
        target.pose.position.z = float(fk.pose.position.z + float(d.get("dz", 0.0)))
        return target

    raise ValueError("Waypoint must include either 'abs' or 'rel'")


def main():
    ap = argparse.ArgumentParser(description="Run MoveIt2 waypoints via IK -> plan -> (optional) execute")
    ap.add_argument("--yaml", required=True, help="waypoints.yaml path")
    ap.add_argument("--frame", default=None, help="override default frame (e.g., world)")
    ap.add_argument("--check-only", action="store_true", help="IK only (fast feasibility check)")
    ap.add_argument("--dry-run", action="store_true", help="IK + plan only (no execution)")
    ap.add_argument("--execute", action="store_true", help="Execute trajectory (default if neither check-only nor dry-run)")

    ap.add_argument("--avoid-collisions", action="store_true")
    ap.add_argument("--ik-timeout", type=float, default=2.0)
    ap.add_argument("--plan-time", type=float, default=2.0)
    ap.add_argument("--plan-attempts", type=int, default=3)
    ap.add_argument("--vel", type=float, default=0.2)
    ap.add_argument("--acc", type=float, default=0.2)

    ap.add_argument(
        "--seed-policy",
        choices=["joint_states", "last_ik", "blend"],
        default="last_ik",
        help="IK seed source. last_ik reduces joint-flip; blend mixes joint_states and last_ik.",
    )
    ap.add_argument(
        "--blend-alpha",
        type=float,
        default=0.7,
        help="Used when seed-policy=blend. alpha=weight for last_ik (0..1).",
    )


    args = ap.parse_args()

    # 실행 모드 결정
    if args.check_only:
        mode = "check"
    elif args.dry_run:
        mode = "dry"
    else:
        mode = "exec" if (args.execute or (not args.execute and not args.dry_run and not args.check_only)) else "exec"

    cfg = load_waypoints_yaml(args.yaml)
    defaults = cfg.get("defaults", {})
    seq = cfg.get("sequence", [])
    loop_cfg = cfg.get("loop", {})

    default_frame = defaults.get("frame", "world")
    default_rpy = defaults.get("rpy", None)
    default_dwell = float(defaults.get("dwell", 0.0))

    frame = args.frame if args.frame is not None else default_frame

    rclpy.init()
    node = MoveItWaypointRunner()

    try:
        # loop 설정
        loop_enable = bool(loop_cfg.get("enable", False))
        loop_count = int(loop_cfg.get("count", 1))
        if loop_enable and loop_count == 0:
            loop_count = -1  # infinite

        iter_idx = 0
        while rclpy.ok():
            iter_idx += 1
            node.get_logger().info(f"=== Sequence iteration {iter_idx} (mode={mode}, frame={frame}) ===")

            last_solution_seed: Optional[Dict[str, float]] = None
            for i, wp in enumerate(seq):
                name = wp.get("name", f"wp{i}")
                dwell = float(wp.get("dwell", default_dwell))

                js = node.wait_for_joint_states(timeout_sec=5.0)
                cur_map = node._extract_joint_positions(js)
                cur_j = {j: cur_map.get(j, 0.0) for j in JOINT_ORDER}

                target = make_target_pose(node, frame=frame, wp=wp, cur_joint_pos=cur_j, default_rpy=default_rpy)
                node.get_logger().info(
                    f"[{name}] target pos=({target.pose.position.x:.3f}, {target.pose.position.y:.3f}, {target.pose.position.z:.3f})"
                )

                # IK
                try:
                    # --- seed 선택 (joint-flip 방지 핵심) ---
                    if args.seed_policy == "joint_states" or last_solution_seed is None:
                        seed = cur_j
                    elif args.seed_policy == "last_ik":
                        seed = last_solution_seed
                    else:  # blend
                        seed = blend_seed_dict(cur_j, last_solution_seed, args.blend_alpha)

                    goal_j = node.call_ik(
                        target_pose=target,
                        seed_joint_pos=seed,
                        avoid_collisions=args.avoid_collisions,
                        ik_timeout=args.ik_timeout,
                    )
                except Exception as e:
                    node.get_logger().error(f"[{name}] IK FAIL: {e}")
                    raise

                node.get_logger().info(
                    f"[{name}] IK OK joint_1..6: "
                    + ", ".join([f"{goal_j[j]:.4f}" for j in JOINT_ORDER])
                )
                last_solution_seed = goal_j

                if mode == "check":
                    continue  # IK만 하고 다음으로

                # PLAN
                jt = node.call_plan(
                    start_joint_pos=cur_j,
                    goal_joint_pos=goal_j,
                    planning_time=args.plan_time,
                    plan_attempts=args.plan_attempts,
                    vel_scale=clamp01(args.vel),
                    acc_scale=clamp01(args.acc),
                )
                node.get_logger().info(f"[{name}] PLAN OK points={len(jt.points)}")

                if mode == "dry":
                    continue  # 계획까지만

                # EXECUTE
                node.execute_joint_trajectory(jt)
                node.get_logger().info(f"[{name}] EXEC OK")

                if dwell > 0:
                    time.sleep(dwell)

            if not loop_enable:
                break

            if loop_count > 0 and iter_idx >= loop_count:
                break

        node.get_logger().info("All done.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    
