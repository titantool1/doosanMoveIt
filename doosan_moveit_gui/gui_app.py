#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import yaml
import subprocess
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from pathlib import Path

try:
    PKG_SHARE = Path(get_package_share_directory("doosan_moveit_gui"))
except Exception:
    PKG_SHARE = Path(__file__).resolve().parent  # fallback
CONFIG_DIR = PKG_SHARE / "config"

from PyQt5.QtCore import (
    Qt,
    QProcess,
    QThread,
    QObject,
    pyqtSignal,
    pyqtSlot,
    QTimer,
    QProcessEnvironment,
)
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QFileDialog,
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QAbstractItemView,
    QMessageBox,
    QHeaderView,
    QSplitter,
    QFrame,
)


RVIZ_CFG_PATH = "/tmp/moveit_waypoints_gui.rviz"

# RViz2 config (RobotModel=Topic:/robot_description, Fixed Frame=world)
RVIZ_CONFIG_YAML = r"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: world
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Update Interval: 0
    - Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Enabled: true
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Visual Enabled: true
      Description Source: Topic
      Description Topic: /robot_description
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Name: Interact
    - Class: rviz_default_plugins/MoveCamera
      Name: Move Camera
    - Class: rviz_default_plugins/Select
      Name: Select
    - Class: rviz_default_plugins/FocusCamera
      Name: Focus Camera
    - Class: rviz_default_plugins/Measure
      Name: Measure
    - Class: rviz_default_plugins/SetInitialPose
      Name: 2D Pose Estimate
    - Class: rviz_default_plugins/SetGoal
      Name: 2D Goal Pose
    - Class: rviz_default_plugins/PublishPoint
      Name: Publish Point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
      Name: TF
Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 5
    Focal Point:
      X: 0
      Y: 0
      Z: 0
    Name: Orbit (rviz)
    Pitch: 0.785398
    Yaw: 0.785398
"""
def resolve_default_yaml():
    # 1) 설치된 share 경로 우선
    try:
        share = get_package_share_directory("doosan_moveit_gui")
        p = os.path.join(share, "waypoints.yaml")
        if os.path.isfile(p):
            return p
    except (PackageNotFoundError, Exception):
        pass

    # 2) 개발/소스 실행 fallback: 현재 파일(gui_app.py) 옆에 있는 waypoints.yaml
    here = os.path.dirname(os.path.abspath(__file__))
    p2 = os.path.join(here, "waypoints.yaml")
    return p2

def resolve_rviz_config():
    try:
        share = get_package_share_directory("doosan_moveit_gui")
        p = os.path.join(share, "rviz", "robot_tf_only.rviz")
        if os.path.isfile(p):
            return p
    except (PackageNotFoundError, Exception):
        pass

    # fallback: 소스 트리의 rviz 폴더
    # (gui_app.py는 doosan_moveit_gui/ 폴더에 있으니 한 단계 위로 올라가서 rviz/를 찾는 패턴)
    here = os.path.dirname(os.path.abspath(__file__))
    p2 = os.path.join(os.path.dirname(here), "rviz", "robot_tf_only.rviz")
    return p2

def write_rviz_config(path: str = RVIZ_CFG_PATH) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(RVIZ_CONFIG_YAML)
    return path

def ros2_topic_exists(topic_name: str) -> bool:
    """
    'source + unset QT_QPA_PLATFORM + (필요시) software GL' 환경에서
    ros2 topic list로 존재 여부 확인.
    """
    cmd = r"""
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    unset QT_QPA_PLATFORM
    export LIBGL_ALWAYS_SOFTWARE=1
    ros2 topic list
    """
    p = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True)
    if p.returncode != 0:
        # ros2 자체가 안 뜨면(환경 미적용/설치 문제) -> False
        return False
    topics = p.stdout.splitlines()
    return topic_name in topics

# -----------------------------
# Robot config (너 환경 기준)
# -----------------------------
JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
TIP_LINK = "link_6"

SRV_FK = "/compute_fk"
TOPIC_JS = "/joint_states"

# -----------------------------
# MoveIt Servo (Joint Jog)
# -----------------------------
TOPIC_JOINT_JOG = "/servo_node/delta_joint_cmds"  # 환경에 따라 /delta_joint_cmds 일 수도
SERVO_PUB_HZ = 50.0  # 버튼 누르고 있을 때 퍼블리시 주기
SERVO_FRAME_ID = "base_link"  # 보통 base_link (안 맞으면 world/robot_base 등으로)


# 실행할 러너 스크립트 (같은 폴더에 있다고 가정)
DEFAULT_RUNNER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "moveit_waypoints.py")

# RViz config (있으면 사용)
DEFAULT_RVIZ_CFG = os.path.expanduser("~/ros2_ws/config/moveit_view.rviz")


# -----------------------------
# Helpers
# -----------------------------
def clamp01(x: float) -> float:
    try:
        x = float(x)
    except Exception:
        return 0.0
    return max(0.0, min(1.0, x))


def safe_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except Exception:
        return default


def fmt_pose_xyz(p: Tuple[float, float, float]) -> str:
    return f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"


# -----------------------------
# Waypoint model
# -----------------------------
@dataclass
class Waypoint:
    name: str = "wp"
    kind: str = "abs"  # "abs" or "rel"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rpy_enabled: bool = False
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    dwell: float = 0.0

    def to_dict(self) -> Dict:
        d: Dict = {"name": self.name}
        if self.kind == "abs":
            d["abs"] = {"x": float(self.x), "y": float(self.y), "z": float(self.z)}
        else:
            d["rel"] = {"dx": float(self.x), "dy": float(self.y), "dz": float(self.z)}

        if self.rpy_enabled:
            d["rpy"] = [float(self.roll), float(self.pitch), float(self.yaw)]

        if self.dwell and float(self.dwell) > 0.0:
            d["dwell"] = float(self.dwell)

        return d

    @staticmethod
    def from_dict(wp: Dict) -> "Waypoint":
        name = str(wp.get("name", "wp"))
        dwell = float(wp.get("dwell", 0.0))

        rpy_enabled = "rpy" in wp
        roll = pitch = yaw = 0.0
        if rpy_enabled:
            r = wp.get("rpy", [0, 0, 0])
            roll, pitch, yaw = float(r[0]), float(r[1]), float(r[2])

        if "abs" in wp:
            a = wp["abs"]
            return Waypoint(
                name=name,
                kind="abs",
                x=float(a.get("x", 0.0)),
                y=float(a.get("y", 0.0)),
                z=float(a.get("z", 0.0)),
                rpy_enabled=rpy_enabled,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                dwell=dwell,
            )
        if "rel" in wp:
            r = wp["rel"]
            return Waypoint(
                name=name,
                kind="rel",
                x=float(r.get("dx", 0.0)),
                y=float(r.get("dy", 0.0)),
                z=float(r.get("dz", 0.0)),
                rpy_enabled=rpy_enabled,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                dwell=dwell,
            )

        # fallback
        return Waypoint(name=name, dwell=dwell)

# -----------------------------
# ROS Worker (QThread)
# - joint_states 모니터
# - FK 서비스 호출(캡처)
# -----------------------------
class RosWorker(QObject):
    sig_status = pyqtSignal(str)
    sig_joint = pyqtSignal(dict)          # {joint_1: rad, ...}
    sig_fk_pose = pyqtSignal(float, float, float)  # x,y,z
    sig_ros_ok = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._running = False
        self._latest_joint_map: Dict[str, float] = {}
        self._fk_request_pending = False
        # --- jog state ---
        self._jog_active = False
        self._jog_joint = ""
        self._jog_vel = 0.0
        self._jog_last_pub = 0.0

        # trajectory fallback publisher (ros2_control)
        self._traj_pub = None
        self._traj_topic = None

        # rclpy objects (worker thread에서만 사용)
        self._rclpy_inited = False
        self._node = None
        self._executor = None
        self._fk_cli = None
        # ---- jog state init (필수) ----
        self._jog_pub = None
        self._jog_active = False
        self._jog_joint = ""
        self._jog_vel = 0.0
        self._jog_last_pub = 0.0

    @pyqtSlot()
    def start(self):
        self._running = True
        self._run_loop()

    @pyqtSlot()
    def stop(self):
        self._running = False

    @pyqtSlot()
    def request_fk_capture(self):
        # GUI thread -> worker thread queued slot
        self._fk_request_pending = True

    @pyqtSlot(str, float)
    def start_joint_jog(self, joint_name: str, vel_rad_s: float):
        # GUI thread -> worker thread (queued)
        self._jog_joint = str(joint_name)
        self._jog_vel = float(vel_rad_s)
        self._jog_active = True

    @pyqtSlot()
    def stop_joint_jog(self):
        self._jog_active = False
        self._jog_joint = ""
        self._jog_vel = 0.0


    def _run_loop(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from sensor_msgs.msg import JointState
            from moveit_msgs.srv import GetPositionFK
            from moveit_msgs.msg import RobotState
            from control_msgs.msg import JointJog
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
        except Exception as e:
            self.sig_status.emit(f"[ROS] import 실패: {e}")
            self.sig_ros_ok.emit(False)
            return

        try:
            rclpy.init(args=None)
            self._rclpy_inited = True
        except Exception as e:
            self.sig_status.emit(f"[ROS] rclpy.init 실패: {e}")
            self.sig_ros_ok.emit(False)
            return

        class _Node(Node):
            pass

        self._node = _Node("moveit_waypoints_gui_ros")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self.sig_status.emit("[ROS] ROS monitor started. Waiting /joint_states ...")
        self.sig_ros_ok.emit(True)

        # joint_states subscriber
        def on_js(msg):
            mp = {}
            for n, p in zip(msg.name, msg.position):
                mp[str(n)] = float(p)
            # JOINT_ORDER 기준으로 정리
            out = {j: float(mp.get(j, 0.0)) for j in JOINT_ORDER}
            self._latest_joint_map = out
            self.sig_joint.emit(out)

        self._node.create_subscription(JointState, TOPIC_JS, on_js, 10)

        # FK service client
        self._fk_cli = self._node.create_client(GetPositionFK, SRV_FK)
        # Joint jog publisher (MoveIt Servo)
        try:
            self._jog_pub = self._node.create_publisher(JointJog, TOPIC_JOINT_JOG, 10)
            self.sig_status.emit(f"[ROS] JointJog publisher ready: {TOPIC_JOINT_JOG}")
        except Exception as e:
            self._jog_pub = None
            self.sig_status.emit(f"[ROS] JointJog publisher create failed: {e}")
        def _find_joint_traj_topic():
            # ros2_control joint trajectory controller 토픽을 자동으로 찾는다
            try:
                for name, types in self._node.get_topic_names_and_types():
                    if "trajectory_msgs/msg/JointTrajectory" in types and name.endswith("joint_trajectory"):
                        return name
            except Exception:
                pass
            return None

        def _ensure_traj_pub():
            if self._traj_pub is not None:
                return True
            topic = _find_joint_traj_topic()
            if not topic:
                return False
            try:
                self._traj_topic = topic
                self._traj_pub = self._node.create_publisher(JointTrajectory, topic, 10)
                self.sig_status.emit(f"[ROS] Trajectory fallback publisher ready: {topic}")
                return True
            except Exception as e:
                self._traj_pub = None
                self._traj_topic = None
                self.sig_status.emit(f"[ROS] Trajectory publisher create failed: {e}")
                return False
        # FK state machine
        fk_future = None
        fk_sent_time = 0.0

        while self._running:
            # spin once
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                pass
                
            # ---- jog publish (servo if available, else trajectory fallback) ----
            if self._jog_active and self._jog_joint:
                now = time.time()
                period = 1.0 / max(1.0, float(SERVO_PUB_HZ))

                if (now - self._jog_last_pub) >= period:
                    self._jog_last_pub = now

                    # 1) MoveIt Servo가 실제로 구독 중이면 JointJog를 보낸다
                    try:
                        servo_subs = self._node.count_subscribers(TOPIC_JOINT_JOG)
                    except Exception:
                        servo_subs = 0

                    if servo_subs > 0 and self._jog_pub is not None:
                        try:
                            msg = JointJog()
                            msg.header.stamp = self._node.get_clock().now().to_msg()
                            msg.header.frame_id = SERVO_FRAME_ID
                            msg.joint_names = [self._jog_joint]
                            msg.displacements = [0.0]
                            msg.velocities = [float(self._jog_vel)]
                            msg.duration = 0.0
                            self._jog_pub.publish(msg)
                        except Exception as e:
                            self.sig_status.emit(f"[ROS] JointJog publish failed: {e}")
                            self._jog_active = False

                    else:
                        # 2) Servo가 없으면: JointTrajectory로 "조금씩" 목표각을 보내는 폴백
                        if not _ensure_traj_pub():
                            # 컨트롤러 토픽 자체가 없으면 여기서 멈춤
                            continue

                        try:
                            if self._jog_joint not in JOINT_ORDER:
                                continue

                            dt = period
                            step = float(self._jog_vel) * dt  # rad

                            target = [self._latest_joint_map.get(j, 0.0) for j in JOINT_ORDER]
                            idx = JOINT_ORDER.index(self._jog_joint)
                            target[idx] += step

                            traj = JointTrajectory()
                            traj.joint_names = JOINT_ORDER

                            pt = JointTrajectoryPoint()
                            pt.positions = target
                            pt.time_from_start = Duration(sec=0, nanosec=int(0.20 * 1e9))
                            traj.points = [pt]

                            self._traj_pub.publish(traj)

                        except Exception as e:
                            self.sig_status.emit(f"[ROS] Trajectory publish failed: {e}")
                            self._jog_active = False

            # handle FK request
            if self._fk_request_pending:
                self._fk_request_pending = False

                if self._fk_cli is None or not self._fk_cli.service_is_ready():
                    # wait a bit for service (non-blocking-ish)
                    if self._fk_cli is not None:
                        self._fk_cli.wait_for_service(timeout_sec=0.2)

                if self._fk_cli is None or not self._fk_cli.service_is_ready():
                    self.sig_status.emit(f"[ROS] FK service not available: {SRV_FK}")
                    continue

                # build request
                req = GetPositionFK.Request()
                req.fk_link_names = [TIP_LINK]
                req.robot_state = RobotState()
                req.robot_state.joint_state.name = JOINT_ORDER
                req.robot_state.joint_state.position = [self._latest_joint_map.get(j, 0.0) for j in JOINT_ORDER]

                try:
                    fk_future = self._fk_cli.call_async(req)
                    fk_sent_time = time.time()
                    self.sig_status.emit("[ROS] FK request sent.")
                except Exception as e:
                    fk_future = None
                    self.sig_status.emit(f"[ROS] FK call_async 실패: {e}")

            # check FK future
            if fk_future is not None and fk_future.done():
                try:
                    res = fk_future.result()
                    if res is None or res.error_code.val != 1 or len(res.pose_stamped) == 0:
                        self.sig_status.emit(f"[ROS] FK failed. error_code={None if res is None else res.error_code.val}")
                    else:
                        ps = res.pose_stamped[0].pose
                        self.sig_fk_pose.emit(float(ps.position.x), float(ps.position.y), float(ps.position.z))
                        self.sig_status.emit("[ROS] FK OK (captured).")
                except Exception as e:
                    self.sig_status.emit(f"[ROS] FK result 처리 실패: {e}")
                fk_future = None

            # timeout safety
            if fk_future is not None and (time.time() - fk_sent_time) > 3.0:
                self.sig_status.emit("[ROS] FK timeout.")
                fk_future = None

        # shutdown
        try:
            if self._executor and self._node:
                self._executor.remove_node(self._node)
        except Exception:
            pass

        try:
            if self._node:
                self._node.destroy_node()
        except Exception:
            pass

        try:
            if self._rclpy_inited:
                rclpy.shutdown()
        except Exception:
            pass

        self.sig_ros_ok.emit(False)
        self.sig_status.emit("[ROS] Stopped.")


# -----------------------------
# Main GUI
# -----------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MoveIt2 Waypoints GUI (PyQt5)")
        self.resize(1200, 720)

        self.runner_proc: Optional[QProcess] = None
        self.rviz_proc: Optional[QProcess] = None

        # ROS worker thread
        self.ros_thread = QThread(self)
        self.ros_worker = RosWorker()
        self.ros_worker.moveToThread(self.ros_thread)
        self.ros_thread.started.connect(self.ros_worker.start)
        self._connect_ros_signals()

        # UI
        root = QWidget()
        self.setCentralWidget(root)

        splitter = QSplitter(Qt.Horizontal)

        left = self._build_control_panel()
        right = self._build_status_panel()

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([360, 840])

        layout = QHBoxLayout()
        layout.addWidget(splitter)
        root.setLayout(layout)

        # start ROS monitor
        self.ros_thread.start()

        # periodic UI refresh (optional)
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._tick)
        self._ui_timer.start(500)

    def ensure_rviz_process(self):
        # "프로세스 객체"만 준비하고, 여기서는 실행(start)하지 않는다
        if hasattr(self, "rviz_proc") and self.rviz_proc is not None:
            return

        self.rviz_proc = QProcess(self)

        # RViz stdout/stderr -> GUI 로그창
        self.rviz_proc.readyReadStandardOutput.connect(self._rviz_on_stdout)
        self.rviz_proc.readyReadStandardError.connect(self._rviz_on_stderr)
        self.rviz_proc.finished.connect(
            lambda code, status: self.append_log(f"[RViz] finished code={code} status={status}")
        )

    def _rviz_on_stdout(self):
        try:
            data = bytes(self.rviz_proc.readAllStandardOutput()).decode("utf-8", errors="replace")
            if data.strip():
                self.append_log("[RViz] " + data.rstrip())
        except Exception:
            pass

    def _rviz_on_stderr(self):
        try:
            data = bytes(self.rviz_proc.readAllStandardError()).decode("utf-8", errors="replace")
            if data.strip():
                self.append_log("[RViz][ERR] " + data.rstrip())
        except Exception:
            pass

    def on_click_rviz(self):
        self.ensure_rviz_process()

        # 이미 RViz 떠 있으면 토글로 종료
        if self.rviz_proc.state() != QProcess.NotRunning:
            self.stop_rviz()
            return

        if not ros2_topic_exists("/robot_description"):
            self.append_log("[GUI] /robot_description가 없습니다. MoveIt/bringup을 먼저 실행하세요.")
            return

        cfg = resolve_rviz_config()
        self.start_rviz(cfg)

    def start_rviz(self, cfg_path: str):
        self.ensure_rviz_process()

        env = QProcessEnvironment.systemEnvironment()
        if env.contains("QT_QPA_PLATFORM"):
            env.remove("QT_QPA_PLATFORM")
        env.insert("LIBGL_ALWAYS_SOFTWARE", "1")
        env.insert("QT_X11_NO_MITSHM", "1")
        self.rviz_proc.setProcessEnvironment(env)

        # ✅ (추가) Doosan MoveIt 기본 RViz 설정 사용
        use_cfg = cfg_path
        #use_cfg = resolve_rviz_config()

        cmd = f"""
        source /opt/ros/humble/setup.bash
        source ~/ros2_ws/install/setup.bash
        unset QT_QPA_PLATFORM
        export LIBGL_ALWAYS_SOFTWARE=1
        export QT_X11_NO_MITSHM=1
        export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
        rviz2 -d "{use_cfg}"
        """
        self.append_log(f"[GUI] RViz config: {use_cfg}")
        self.append_log("[GUI] RViz starting... (XRDP/가상환경이면 10초 이상 걸릴 수 있어요)")
        self.rviz_proc.start("bash", ["-lc", cmd])
        QTimer.singleShot(3000, lambda: self.append_log(f"[GUI] RViz state={self.rviz_proc.state()} (0=NotRunning, 1=Starting, 2=Running)"))

    def stop_rviz(self):
        if not hasattr(self, "rviz_proc") or self.rviz_proc is None:
            return
        if self.rviz_proc.state() == QProcess.NotRunning:
            return

        self.append_log("[GUI] RViz 종료 요청")
        self.rviz_proc.terminate()
        if not self.rviz_proc.waitForFinished(1500):
            self.append_log("[GUI] RViz 강제 종료(kill)")
            self.rviz_proc.kill()
            



    # -------------------------
    # UI building
    # -------------------------
    def _on_jog_pressed(self, joint_name: str, sign: float):
        # servo topic이 없으면 안내만
        vel = float(self.sp_jog.value()) * float(sign)
        self.append_log(f"[GUI] Jog start: {joint_name} vel={vel:.3f} rad/s")
        self.ros_worker.start_joint_jog(joint_name, vel)

    def _on_jog_released(self):
        self.append_log("[GUI] Jog stop")
        self.ros_worker.stop_joint_jog()

    def _build_control_panel(self) -> QWidget:
        panel = QWidget()
        v = QVBoxLayout(panel)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(8)

        # Control Group
        gb = QGroupBox("Control")
        g = QGridLayout(gb)
        g.setVerticalSpacing(10)
        g.setHorizontalSpacing(8)

        # YAML path
        g.addWidget(QLabel("YAML"), 0, 0)
        #self.ed_yaml = QLineEdit("waypoints.yaml")
        self.ed_yaml = QLineEdit(resolve_default_yaml())
        self.btn_browse = QPushButton("Browse")
        self.btn_browse.clicked.connect(self.on_browse_yaml)
        row0 = QHBoxLayout()
        row0.addWidget(self.ed_yaml, 1)
        row0.addWidget(self.btn_browse)
        wrap0 = QWidget(); wrap0.setLayout(row0)
        g.addWidget(wrap0, 0, 1, 1, 2)

        # Frame override
        g.addWidget(QLabel("Frame override"), 1, 0)
        self.ed_frame = QLineEdit("")
        g.addWidget(self.ed_frame, 1, 1, 1, 2)

        # Mode
        g.addWidget(QLabel("Mode"), 2, 0)
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["exec", "dry", "check"])
        g.addWidget(self.cb_mode, 2, 1, 1, 2)

        # avoid collisions
        self.ck_avoid = QCheckBox("avoid_collisions")
        g.addWidget(self.ck_avoid, 3, 0, 1, 3)

        # IK timeout
        g.addWidget(QLabel("IK timeout (s)"), 4, 0)
        self.sp_ik = QDoubleSpinBox()
        self.sp_ik.setRange(0.1, 10.0)
        self.sp_ik.setSingleStep(0.1)
        self.sp_ik.setValue(2.0)
        g.addWidget(self.sp_ik, 4, 1, 1, 2)

        # Plan time
        g.addWidget(QLabel("Plan time (s)"), 5, 0)
        self.sp_plan_time = QDoubleSpinBox()
        self.sp_plan_time.setRange(0.1, 20.0)
        self.sp_plan_time.setSingleStep(0.1)
        self.sp_plan_time.setValue(2.0)
        g.addWidget(self.sp_plan_time, 5, 1, 1, 2)

        # Plan attempts
        g.addWidget(QLabel("Plan attempts"), 6, 0)
        self.sp_plan_attempts = QSpinBox()
        self.sp_plan_attempts.setRange(1, 50)
        self.sp_plan_attempts.setValue(3)
        g.addWidget(self.sp_plan_attempts, 6, 1, 1, 2)

        # vel/acc
        g.addWidget(QLabel("Vel scale (0..1)"), 7, 0)
        self.sp_vel = QDoubleSpinBox()
        self.sp_vel.setRange(0.01, 1.0)
        self.sp_vel.setSingleStep(0.05)
        self.sp_vel.setValue(0.2)
        g.addWidget(self.sp_vel, 7, 1, 1, 2)

        g.addWidget(QLabel("Acc scale (0..1)"), 8, 0)
        self.sp_acc = QDoubleSpinBox()
        self.sp_acc.setRange(0.01, 1.0)
        self.sp_acc.setSingleStep(0.05)
        self.sp_acc.setValue(0.2)
        g.addWidget(self.sp_acc, 8, 1, 1, 2)

        # Seed policy (네가 쓰던 옵션)
        g.addWidget(QLabel("Seed policy"), 9, 0)
        self.cb_seed = QComboBox()
        self.cb_seed.addItems(["(none)", "joint_states", "last_ik", "blend"])
        self.sp_blend = QDoubleSpinBox()
        self.sp_blend.setRange(0.0, 1.0)
        self.sp_blend.setSingleStep(0.05)
        self.sp_blend.setValue(0.70)
        row_seed = QHBoxLayout()
        row_seed.addWidget(self.cb_seed, 1)
        row_seed.addWidget(QLabel("blend α"))
        row_seed.addWidget(self.sp_blend)
        wrap_seed = QWidget(); wrap_seed.setLayout(row_seed)
        g.addWidget(wrap_seed, 9, 1, 1, 2)

        # Buttons row: Start/Stop + RViz
        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        self.btn_rviz = QPushButton("Open RViz")

        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_rviz.clicked.connect(self.on_click_rviz)

        row_btn = QHBoxLayout()
        row_btn.addWidget(self.btn_start, 1)
        row_btn.addWidget(self.btn_stop, 1)
        row_btn.addWidget(self.btn_rviz, 1)


        wrap_btn = QWidget()
        wrap_btn.setLayout(row_btn)

        # ✅ grid에 "한 번만"
        g.addWidget(wrap_btn, 10, 0, 1, 3)
       
        v.addWidget(gb, 0)

        # Waypoints Editor
        v.addWidget(self._build_waypoints_editor(), 1)

        v.addStretch(0)
        return panel

    def _build_waypoints_editor(self) -> QWidget:
        gb = QGroupBox("Waypoints Editor")
        v = QVBoxLayout(gb)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(8)

        # table
        self.tbl = QTableWidget(0, 9)
        self.tbl.setHorizontalHeaderLabels([
            "name", "kind", "x/dx", "y/dy", "z/dz",
            "rpy?", "roll", "pitch", "yaw"
        ])
        self.tbl.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.itemSelectionChanged.connect(self.on_table_select)

        v.addWidget(self.tbl, 1)

        # form
        form = QGridLayout()
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(6)

        self.ed_wp_name = QLineEdit("A")

        self.cb_kind = QComboBox()
        self.cb_kind.addItems(["abs", "rel"])
        self.cb_kind.currentTextChanged.connect(self._update_xyz_labels)

        self.lb_x = QLabel("x")
        self.lb_y = QLabel("y")
        self.lb_z = QLabel("z")

        self.ed_x = QLineEdit("0.0")
        self.ed_y = QLineEdit("0.0")
        self.ed_z = QLineEdit("0.0")

        self.ck_rpy = QCheckBox("use rpy")
        self.ck_rpy.stateChanged.connect(self._update_rpy_enabled)
        self.ed_roll = QLineEdit("0.0")
        self.ed_pitch = QLineEdit("0.0")
        self.ed_yaw = QLineEdit("0.0")

        self.sp_dwell = QDoubleSpinBox()
        self.sp_dwell.setRange(0.0, 10.0)
        self.sp_dwell.setSingleStep(0.1)
        self.sp_dwell.setValue(0.0)

        # FK capture button
        self.btn_fk = QPushButton("Capture Current Pose (FK)")
        self.btn_fk.clicked.connect(self.on_capture_fk)

        form.addWidget(QLabel("name"), 0, 0)
        form.addWidget(self.ed_wp_name, 0, 1)
        form.addWidget(QLabel("kind"), 0, 2)
        form.addWidget(self.cb_kind, 0, 3)

        form.addWidget(self.lb_x, 1, 0)
        form.addWidget(self.ed_x, 1, 1)
        form.addWidget(self.lb_y, 1, 2)
        form.addWidget(self.ed_y, 1, 3)

        form.addWidget(self.lb_z, 2, 0)
        form.addWidget(self.ed_z, 2, 1)
        form.addWidget(QLabel("dwell(s)"), 2, 2)
        form.addWidget(self.sp_dwell, 2, 3)

        form.addWidget(self.ck_rpy, 3, 0)
        form.addWidget(QLabel("roll"), 3, 1)
        form.addWidget(QLabel("pitch"), 3, 2)
        form.addWidget(QLabel("yaw"), 3, 3)

        form.addWidget(self.ed_roll, 4, 1)
        form.addWidget(self.ed_pitch, 4, 2)
        form.addWidget(self.ed_yaw, 4, 3)

        # action buttons
        self.btn_add = QPushButton("Add / Update")
        self.btn_del = QPushButton("Delete selected")
        self.btn_clear = QPushButton("Clear form")
        self.btn_load = QPushButton("Load YAML → Table")
        self.btn_save = QPushButton("Save Table → YAML")

        self.btn_add.clicked.connect(self.on_add_update)
        self.btn_del.clicked.connect(self.on_delete_selected)
        self.btn_clear.clicked.connect(self.on_clear_form)
        self.btn_load.clicked.connect(self.on_load_yaml_to_table)
        self.btn_save.clicked.connect(self.on_save_table_to_yaml)

        row1 = QHBoxLayout()
        row1.addWidget(self.btn_add, 1)
        row1.addWidget(self.btn_del, 1)
        row1.addWidget(self.btn_clear, 1)

        row2 = QHBoxLayout()
        row2.addWidget(self.btn_fk, 2)
        row2.addWidget(self.btn_load, 1)
        row2.addWidget(self.btn_save, 1)

        v.addLayout(form)
        v.addWidget(self._hline())
        v.addLayout(row1)
        v.addLayout(row2)

        self._update_rpy_enabled()
        self._update_xyz_labels()

        return gb

    def _build_status_panel(self) -> QWidget:
        panel = QWidget()
        v = QVBoxLayout(panel)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(8)

        gb = QGroupBox("Status")
        g = QVBoxLayout(gb)

        self.lb_ros = QLabel("ROS: (starting...)")
        self.lb_ros.setStyleSheet("font-weight: bold;")
        g.addWidget(self.lb_ros)

        self.lb_js = QLabel("joint_states: waiting")
        g.addWidget(self.lb_js)

        self.lb_runner = QLabel("Runner: idle")
        g.addWidget(self.lb_runner)

        g.addWidget(QLabel("Joint angles (rad)"))
        # Jog speed control
        jog_row = QHBoxLayout()
        jog_row.addWidget(QLabel("Jog speed (rad/s)"))
        self.sp_jog = QDoubleSpinBox()
        self.sp_jog.setRange(0.01, 6.0)
        self.sp_jog.setSingleStep(0.01)
        self.sp_jog.setValue(2.00)
        jog_row.addWidget(self.sp_jog)
        jog_row.addStretch(1)
        g.addLayout(jog_row)
        self.joint_labels: Dict[str, QLabel] = {}
        grid = QGridLayout()
        self._jog_buttons = []  # (btn_minus, btn_plus) 보관용

        for i, j in enumerate(JOINT_ORDER):
            grid.addWidget(QLabel(j), i, 0)

            lb = QLabel("0.0000")
            lb.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.joint_labels[j] = lb
            grid.addWidget(lb, i, 1)

            btn_m = QPushButton("-")
            btn_p = QPushButton("+")
            btn_m.setAutoRepeat(False)
            btn_p.setAutoRepeat(False)

            # 누르는 동안 시작, 떼면 정지
            btn_m.pressed.connect(lambda jj=j: self._on_jog_pressed(jj, -1.0))
            btn_m.released.connect(self._on_jog_released)
            btn_p.pressed.connect(lambda jj=j: self._on_jog_pressed(jj, +1.0))
            btn_p.released.connect(self._on_jog_released)

            grid.addWidget(btn_m, i, 2)
            grid.addWidget(btn_p, i, 3)

            self._jog_buttons.append((btn_m, btn_p))
        g.addLayout(grid)

        v.addWidget(gb, 0)

        # Log
        gb2 = QGroupBox("Log")
        vv = QVBoxLayout(gb2)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        vv.addWidget(self.txt_log, 1)
        v.addWidget(gb2, 1)

        return panel

    def _hline(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    # -------------------------
    # ROS signal handlers
    # -------------------------
    def _connect_ros_signals(self):
        self.ros_worker.sig_status.connect(self.append_log)
        self.ros_worker.sig_joint.connect(self.on_joint_update)
        self.ros_worker.sig_fk_pose.connect(self.on_fk_pose)
        self.ros_worker.sig_ros_ok.connect(self.on_ros_ok)

    @pyqtSlot(bool)
    def on_ros_ok(self, ok: bool):
        if ok:
            self.lb_ros.setText("ROS: ROS monitor started.")
            self.lb_ros.setStyleSheet("font-weight: bold; color: green;")
        else:
            self.lb_ros.setText("ROS: stopped")
            self.lb_ros.setStyleSheet("font-weight: bold; color: gray;")

    @pyqtSlot(dict)
    def on_joint_update(self, mp: Dict[str, float]):
        self.lb_js.setText("joint_states: OK")
        self.lb_js.setStyleSheet("color: green;")
        for j in JOINT_ORDER:
            self.joint_labels[j].setText(f"{mp.get(j, 0.0):.4f}")

    @pyqtSlot(float, float, float)
    def on_fk_pose(self, x: float, y: float, z: float):
        # FK 캡처는 abs 입력에 넣어주는게 직관적
        self.cb_kind.setCurrentText("abs")
        self.ed_x.setText(f"{x:.6f}")
        self.ed_y.setText(f"{y:.6f}")
        self.ed_z.setText(f"{z:.6f}")
        self.append_log(f"[GUI] FK captured abs xyz={fmt_pose_xyz((x,y,z))}")

    # -------------------------
    # UI events
    # -------------------------
    def append_log(self, s: str):
        t = time.strftime("%H:%M:%S")
        self.txt_log.append(f"[{t}] {s}")

    def _tick(self):
        # runner 상태 텍스트만 가볍게 유지
        if self.runner_proc and self.runner_proc.state() != QProcess.NotRunning:
            self.lb_runner.setText("Runner: running")
            self.lb_runner.setStyleSheet("color: green;")
        else:
            self.lb_runner.setText("Runner: idle")
            self.lb_runner.setStyleSheet("color: gray;")

    def on_browse_yaml(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select YAML", os.getcwd(), "YAML (*.yaml *.yml)")
        if path:
            self.ed_yaml.setText(path)


    def on_start(self):
        yaml_path = self.ed_yaml.text().strip()
        if not yaml_path:
            QMessageBox.warning(self, "YAML", "YAML path is empty.")
            return

        runner = DEFAULT_RUNNER
        if not os.path.exists(runner):
            QMessageBox.warning(self, "Runner", f"Runner script not found:\n{runner}")
            return

        # stop previous if running
        self.on_stop()

        # mode -> args
        mode = self.cb_mode.currentText().strip()
        frame = self.ed_frame.text().strip()

        args = ["python3", runner, "--yaml", yaml_path]

        # 네 moveit_waypoints.py는 check-only/dry-run/execute 구조를 썼었지.
        # 기존과 호환되게:
        if mode == "check":
            args += ["--check-only"]
        elif mode == "dry":
            args += ["--dry-run"]
        else:
            args += ["--execute"]

        if frame:
            args += ["--frame", frame]

        if self.ck_avoid.isChecked():
            args += ["--avoid-collisions"]

        args += ["--ik-timeout", f"{self.sp_ik.value():.3f}"]
        args += ["--plan-time", f"{self.sp_plan_time.value():.3f}"]
        args += ["--plan-attempts", str(self.sp_plan_attempts.value())]
        args += ["--vel", f"{self.sp_vel.value():.3f}"]
        args += ["--acc", f"{self.sp_acc.value():.3f}"]

        # seed policy (너가 이미 쓰는 버전이면 인식됨)
        seed = self.cb_seed.currentText()
        if seed != "(none)":
            args += ["--seed-policy", seed]
            if seed == "blend":
                args += ["--blend-alpha", f"{clamp01(self.sp_blend.value()):.3f}"]

        # run
        self.runner_proc = QProcess(self)
        self.runner_proc.setProcessChannelMode(QProcess.MergedChannels)
        self.runner_proc.readyReadStandardOutput.connect(lambda: self._drain_proc(self.runner_proc, prefix=""))
        self.runner_proc.finished.connect(self._on_runner_finished)

        self.append_log("[GUI] Running:")
        self.append_log("  " + " ".join(args))

        # NOTE: 환경이 이미 잡혀있다고 가정하고 직접 실행.
        self.runner_proc.start(args[0], args[1:])

    def on_stop(self):
        if self.runner_proc and self.runner_proc.state() != QProcess.NotRunning:
            self.append_log("[GUI] Stop requested.")
            self.runner_proc.terminate()
            if not self.runner_proc.waitForFinished(2000):
                self.runner_proc.kill()
        self.runner_proc = None

    def _drain_proc(self, proc: QProcess, prefix: str = ""):
        if not proc:
            return
        data = proc.readAllStandardOutput()
        try:
            txt = bytes(data).decode("utf-8", errors="replace")
        except Exception:
            txt = str(data)
        for line in txt.splitlines():
            if line.strip():
                self.append_log(prefix + line)

    def _on_runner_finished(self, *args):
        code = self.runner_proc.exitCode() if self.runner_proc else -1
        self.append_log(f"[GUI] Runner finished. exitCode={code}")
        self.runner_proc = None

    # -------------------------
    # Waypoints Editor handlers
    # -------------------------
    def _update_xyz_labels(self):
        kind = self.cb_kind.currentText()
        if kind == "abs":
            self.lb_x.setText("x")
            self.lb_y.setText("y")
            self.lb_z.setText("z")
        else:
            self.lb_x.setText("dx")
            self.lb_y.setText("dy")
            self.lb_z.setText("dz")

    def _update_rpy_enabled(self):
        en = self.ck_rpy.isChecked()
        self.ed_roll.setEnabled(en)
        self.ed_pitch.setEnabled(en)
        self.ed_yaw.setEnabled(en)

    def _current_form_waypoint(self) -> Waypoint:
        wp = Waypoint()
        wp.name = self.ed_wp_name.text().strip() or "wp"
        wp.kind = self.cb_kind.currentText()
        wp.x = safe_float(self.ed_x.text(), 0.0)
        wp.y = safe_float(self.ed_y.text(), 0.0)
        wp.z = safe_float(self.ed_z.text(), 0.0)

        wp.rpy_enabled = self.ck_rpy.isChecked()
        wp.roll = safe_float(self.ed_roll.text(), 0.0)
        wp.pitch = safe_float(self.ed_pitch.text(), 0.0)
        wp.yaw = safe_float(self.ed_yaw.text(), 0.0)

        wp.dwell = float(self.sp_dwell.value())
        return wp

    def on_add_update(self):
        wp = self._current_form_waypoint()

        # if a row selected -> update
        row = self._selected_row()
        if row is None:
            row = self.tbl.rowCount()
            self.tbl.insertRow(row)

        self._set_row_from_waypoint(row, wp)
        self.append_log(f"[GUI] Waypoint saved to table: {wp.name} ({wp.kind})")

    def on_delete_selected(self):
        row = self._selected_row()
        if row is None:
            return
        name = self.tbl.item(row, 0).text() if self.tbl.item(row, 0) else ""
        self.tbl.removeRow(row)
        self.append_log(f"[GUI] Waypoint deleted: {name}")

    def on_clear_form(self):
        self.ed_wp_name.setText("wp")
        self.cb_kind.setCurrentText("abs")
        self.ed_x.setText("0.0")
        self.ed_y.setText("0.0")
        self.ed_z.setText("0.0")
        self.ck_rpy.setChecked(False)
        self.ed_roll.setText("0.0")
        self.ed_pitch.setText("0.0")
        self.ed_yaw.setText("0.0")
        self.sp_dwell.setValue(0.0)
        self.tbl.clearSelection()

    def on_table_select(self):
        row = self._selected_row()
        if row is None:
            return
        wp = self._waypoint_from_row(row)
        self._fill_form_from_waypoint(wp)

    def _selected_row(self) -> Optional[int]:
        sel = self.tbl.selectionModel().selectedRows()
        if not sel:
            return None
        return sel[0].row()

    def _set_row_from_waypoint(self, row: int, wp: Waypoint):
        def put(col: int, val: str):
            it = QTableWidgetItem(val)
            it.setFlags(it.flags() ^ Qt.ItemIsEditable)
            self.tbl.setItem(row, col, it)

        put(0, wp.name)
        put(1, wp.kind)
        put(2, f"{wp.x:.6f}")
        put(3, f"{wp.y:.6f}")
        put(4, f"{wp.z:.6f}")
        put(5, "Y" if wp.rpy_enabled else "N")
        put(6, f"{wp.roll:.6f}")
        put(7, f"{wp.pitch:.6f}")
        put(8, f"{wp.yaw:.6f}")

    def _waypoint_from_row(self, row: int) -> Waypoint:
        def get(col: int, default: str = "") -> str:
            it = self.tbl.item(row, col)
            return it.text() if it else default

        wp = Waypoint()
        wp.name = get(0, "wp")
        wp.kind = get(1, "abs")
        wp.x = safe_float(get(2, "0.0"), 0.0)
        wp.y = safe_float(get(3, "0.0"), 0.0)
        wp.z = safe_float(get(4, "0.0"), 0.0)
        wp.rpy_enabled = (get(5, "N").upper() == "Y")
        wp.roll = safe_float(get(6, "0.0"), 0.0)
        wp.pitch = safe_float(get(7, "0.0"), 0.0)
        wp.yaw = safe_float(get(8, "0.0"), 0.0)
        return wp

    def _fill_form_from_waypoint(self, wp: Waypoint):
        self.ed_wp_name.setText(wp.name)
        self.cb_kind.setCurrentText(wp.kind)
        self.ed_x.setText(f"{wp.x:.6f}")
        self.ed_y.setText(f"{wp.y:.6f}")
        self.ed_z.setText(f"{wp.z:.6f}")
        self.ck_rpy.setChecked(bool(wp.rpy_enabled))
        self.ed_roll.setText(f"{wp.roll:.6f}")
        self.ed_pitch.setText(f"{wp.pitch:.6f}")
        self.ed_yaw.setText(f"{wp.yaw:.6f}")

    def on_load_yaml_to_table(self):
        path = self.ed_yaml.text().strip()
        if not path:
            QMessageBox.warning(self, "YAML", "YAML path is empty.")
            return

        if not os.path.exists(path):
            QMessageBox.warning(self, "YAML", f"File not found:\n{path}")
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            QMessageBox.critical(self, "YAML", f"Load failed:\n{e}")
            return

        seq = cfg.get("sequence", [])
        if not isinstance(seq, list):
            QMessageBox.critical(self, "YAML", "Invalid YAML: sequence must be a list.")
            return

        self.tbl.setRowCount(0)
        for wp_dict in seq:
            wp = Waypoint.from_dict(wp_dict)
            row = self.tbl.rowCount()
            self.tbl.insertRow(row)
            self._set_row_from_waypoint(row, wp)

        # defaults -> frame override 채워주기(있으면)
        defaults = cfg.get("defaults", {})
        if isinstance(defaults, dict):
            frame = defaults.get("frame", "")
            if frame:
                self.ed_frame.setText(str(frame))

        self.append_log(f"[GUI] YAML loaded into table: {path} (rows={len(seq)})")

    def on_save_table_to_yaml(self):
        path = self.ed_yaml.text().strip()
        if not path:
            QMessageBox.warning(self, "YAML", "YAML path is empty.")
            return

        seq: List[Dict] = []
        for row in range(self.tbl.rowCount()):
            wp = self._waypoint_from_row(row)
            seq.append(wp.to_dict())

        cfg: Dict = {"defaults": {}, "sequence": seq}

        # defaults: frame, dwell은 최소만
        frame = self.ed_frame.text().strip()
        if frame:
            cfg["defaults"]["frame"] = frame
        else:
            cfg["defaults"]["frame"] = "world"

        # save
        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
        except Exception as e:
            QMessageBox.critical(self, "YAML", f"Save failed:\n{e}")
            return

        self.append_log(f"[GUI] Table saved to YAML: {path} (rows={len(seq)})")

    # -------------------------
    # FK capture
    # -------------------------
    def on_capture_fk(self):
        # worker thread로 요청 (FK service는 worker가 처리)
        self.append_log("[GUI] FK capture requested...")
        self.ros_worker.request_fk_capture()

    # -------------------------
    # Close / cleanup
    # -------------------------
    def closeEvent(self, event):
        # stop runner
        try:
            self.on_stop()
        except Exception:
            pass

        # stop rviz
        try:
            if self.rviz_proc and self.rviz_proc.state() != QProcess.NotRunning:
                self.rviz_proc.terminate()
                if not self.rviz_proc.waitForFinished(1500):
                    self.rviz_proc.kill()
        except Exception:
            pass

        # stop ros thread
        try:
            self.ros_worker.stop()
        except Exception:
            pass
        try:
            self.ros_thread.quit()
            self.ros_thread.wait(2000)
        except Exception:
            pass

        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
