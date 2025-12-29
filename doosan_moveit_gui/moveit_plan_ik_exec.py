#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, MotionPlanRequest
from moveit_msgs.msg import MoveItErrorCodes

from control_msgs.action import FollowJointTrajectory


JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
TIP_LINK = "link_6"

SRV_IK = "/compute_ik"
SRV_FK = "/compute_fk"
SRV_PLAN = "/plan_kinematic_path"
ACTION_FJT = "/dsr_moveit_controller/follow_joint_trajectory"


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)

    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx, qy, qz, qw)


class MoveItPlanIkExec(Node):
    def __init__(self):
        super().__init__("moveit_plan_ik_exec")

        # /joint_states는 launch에서 TRANSIENT_LOCAL로 나오는 경우가 있어, 그걸 받아주게 QoS를 맞춰줌
        js_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latest_js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_js, js_qos)

        self.cli_fk = self.create_client(GetPositionFK, SRV_FK)
        self.cli_ik = self.create_client(GetPositionIK, SRV_IK)
        self.cli_plan = self.create_client(GetMotionPlan, SRV_PLAN)

        self.act_fjt = ActionClient(self, FollowJointTrajectory, ACTION_FJT)

    def _on_js(self, msg: JointState):
        self.latest_js = msg

    def wait_for_services(self, timeout_sec=10.0) -> bool:
        t0 = time.time()
        for cli, name in [(self.cli_fk, SRV_FK), (self.cli_ik, SRV_IK), (self.cli_plan, SRV_PLAN)]:
            while not cli.wait_for_service(timeout_sec=0.2):
                if time.time() - t0 > timeout_sec:
                    self.get_logger().error(f"Service not available: {name}")
                    return False

        t0 = time.time()
        while not self.act_fjt.wait_for_server(timeout_sec=0.2):
            if time.time() - t0 > timeout_sec:
                self.get_logger().error(f"Action server not available: {ACTION_FJT}")
                return False
        return True

    def wait_for_joint_states(self, timeout_sec=5.0) -> bool:
        end = time.time() + timeout_sec
        while rclpy.ok() and self.latest_js is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_js is not None

    def get_current_joint_positions(self) -> List[float]:
        # /joint_states의 name 순서가 섞여 나와도 JOINT_ORDER로 정렬해서 사용
        js = self.latest_js
        if js is None:
            raise RuntimeError("No joint_states yet")
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        return [float(name_to_pos.get(j, 0.0)) for j in JOINT_ORDER]

    def call_fk(self, joint_pos: List[float]) -> PoseStamped:
        req = GetPositionFK.Request()
        req.fk_link_names = [TIP_LINK]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = JOINT_ORDER
        req.robot_state.joint_state.position = joint_pos

        fut = self.cli_fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("FK service call failed (no response)")
        res = fut.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS or len(res.pose_stamped) == 0:
            raise RuntimeError(f"FK failed. error_code={res.error_code.val}")
        return res.pose_stamped[0]

    def call_ik(self, target_pose: PoseStamped, seed_joint_pos: List[float], avoid_collisions: bool, timeout_sec: float) -> List[float]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = TIP_LINK
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.pose_stamped = target_pose

        # seed(힌트) = 현재 관절값
        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.joint_state.name = JOINT_ORDER
        req.ik_request.robot_state.joint_state.position = seed_joint_pos

        # Humble: attempts 필드 없음. timeout은 PositionIKRequest에 존재.
        req.ik_request.timeout.sec = int(timeout_sec)
        req.ik_request.timeout.nanosec = int((timeout_sec - int(timeout_sec)) * 1e9)

        fut = self.cli_ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec + 2.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("IK service call failed (no response)")

        res = fut.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            # -31이면 NO_IK_SOLUTION가 흔함
            raise RuntimeError(f"IK failed. error_code={res.error_code.val} (NO_IK_SOLUTION?)")

        sol_js = res.solution.joint_state
        name_to_pos = {n: p for n, p in zip(sol_js.name, sol_js.position)}
        return [float(name_to_pos[j]) for j in JOINT_ORDER]

    def call_plan(self, start_joint_pos: List[float], goal_joint_pos: List[float],
                  allowed_time: float, attempts: int, vel_scale: float, acc_scale: float) -> object:
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP_NAME

        mpr.start_state = RobotState()
        mpr.start_state.joint_state.name = JOINT_ORDER
        mpr.start_state.joint_state.position = start_joint_pos

        # goal = joint constraints
        c = Constraints()
        for jn, jp in zip(JOINT_ORDER, goal_joint_pos):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(jp)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        mpr.goal_constraints = [c]

        mpr.num_planning_attempts = int(attempts)
        mpr.allowed_planning_time = float(allowed_time)
        mpr.max_velocity_scaling_factor = float(vel_scale)
        mpr.max_acceleration_scaling_factor = float(acc_scale)

        req.motion_plan_request = mpr

        fut = self.cli_plan.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=allowed_time + 5.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("Planning service call failed (no response)")

        res = fut.result()
        mpres = res.motion_plan_response
        if mpres.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Planning failed. error_code={mpres.error_code.val}")
        return mpres.trajectory.joint_trajectory

    def exec_trajectory(self, joint_traj) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        send_fut = self.act_fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=5.0)
        if not send_fut.done() or send_fut.result() is None:
            raise RuntimeError("Failed to send FollowJointTrajectory goal.")
        gh = send_fut.result()
        if not gh.accepted:
            raise RuntimeError("Trajectory goal was rejected.")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=60.0)
        if not res_fut.done() or res_fut.result() is None:
            raise RuntimeError("No result from trajectory execution.")
        result = res_fut.result().result
        if result.error_code != 0:
            raise RuntimeError(f"Execution failed. error_code={result.error_code}")
        return True





def build_arg_parser():
    p = argparse.ArgumentParser(description="MoveIt2: FK -> IK -> plan_kinematic_path -> FollowJointTrajectory")

    sub = p.add_subparsers(dest="mode", required=True)

    p_abs = sub.add_parser("abs", help="absolute target in frame")
    p_abs.add_argument("--x", type=float, required=True)
    p_abs.add_argument("--y", type=float, required=True)
    p_abs.add_argument("--z", type=float, required=True)

    p_rel = sub.add_parser("rel", help="relative delta from current FK pose")
    p_rel.add_argument("--dx", type=float, default=0.0)
    p_rel.add_argument("--dy", type=float, default=0.0)
    p_rel.add_argument("--dz", type=float, default=0.0)

    # 공통 옵션
    p.add_argument("--frame", type=str, default="world", help="pose frame_id (default: world)")

    # rpy를 안 주면 "현재 자세 유지"
    p.add_argument("--roll", type=float, default=None)
    p.add_argument("--pitch", type=float, default=None)
    p.add_argument("--yaw", type=float, default=None)

    p.add_argument("--avoid-collisions", action="store_true", help="IK collision check")
    p.add_argument("--ik-timeout", type=float, default=2.0)
    p.add_argument("--plan-time", type=float, default=5.0)
    p.add_argument("--plan-attempts", type=int, default=10)
    p.add_argument("--vel", type=float, default=0.2, help="velocity scaling (0~1)")
    p.add_argument("--acc", type=float, default=0.2, help="acceleration scaling (0~1)")
    return p


def main():
    args = build_arg_parser().parse_args()

    rclpy.init()
    node = MoveItPlanIkExec()

    try:
        if not node.wait_for_services(timeout_sec=10.0):
            raise SystemExit(1)

        if not node.wait_for_joint_states(timeout_sec=5.0):
            node.get_logger().error("No /joint_states received yet. (launch가 켜져있는지, QoS가 맞는지 확인)")
            raise SystemExit(2)

        cur_j = node.get_current_joint_positions()
        fk_pose = node.call_fk(cur_j)  # 현재 link_6 pose (보통 frame_id=world)

        # target_pose 만들기
        target = PoseStamped()
        target.header.frame_id = args.frame

        if args.mode == "abs":
            target.pose.position.x = float(args.x)
            target.pose.position.y = float(args.y)
            target.pose.position.z = float(args.z)
        else:
            # rel: FK 기준으로 이동 (FK frame을 쓰는 게 안전)
            target.header.frame_id = fk_pose.header.frame_id
            target.pose.position.x = float(fk_pose.pose.position.x + args.dx)
            target.pose.position.y = float(fk_pose.pose.position.y + args.dy)
            target.pose.position.z = float(fk_pose.pose.position.z + args.dz)

        if args.roll is None or args.pitch is None or args.yaw is None:
            # 자세 유지
            target.pose.orientation = fk_pose.pose.orientation
            rpy_msg = "keep-current"
        else:
            qx, qy, qz, qw = rpy_to_quat(args.roll, args.pitch, args.yaw)
            target.pose.orientation.x = qx
            target.pose.orientation.y = qy
            target.pose.orientation.z = qz
            target.pose.orientation.w = qw
            rpy_msg = f"rpy=({args.roll:.3f},{args.pitch:.3f},{args.yaw:.3f})"

        node.get_logger().info(
            f"Target mode={args.mode} frame={target.header.frame_id} ({rpy_msg})"
        )
        node.get_logger().info(
            f"Target pose: pos=({target.pose.position.x:.3f}, {target.pose.position.y:.3f}, {target.pose.position.z:.3f})"
        )

        # IK
        goal_j = node.call_ik(target, seed_joint_pos=cur_j,
                              avoid_collisions=args.avoid_collisions,
                              timeout_sec=args.ik_timeout)

        node.get_logger().info("IK solution (joint_1..6): " + ", ".join([f"{v:.4f}" for v in goal_j]))

        # Plan
        joint_traj = node.call_plan(
            start_joint_pos=cur_j,
            goal_joint_pos=goal_j,
            allowed_time=args.plan_time,
            attempts=args.plan_attempts,
            vel_scale=args.vel,
            acc_scale=args.acc,
        )

        node.get_logger().info(f"Planned trajectory points: {len(joint_traj.points)}")

        # Execute
        node.exec_trajectory(joint_traj)
        node.get_logger().info("Execution done.")

    except Exception as e:
        node.get_logger().error(str(e))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
