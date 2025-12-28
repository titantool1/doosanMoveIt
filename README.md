# doosan_moveit_gui (ROS2 Humble)

PyQt5 기반의 Doosan MoveIt2 조작 GUI입니다.  
Waypoints(YAML) 편집/저장 + 현재 로봇 자세(FK) 캡처 + RViz2 시각화(버튼 실행) 기능을 제공합니다.

## Features

- **RViz2 실행 버튼** (TF + RobotModel + Grid 표시)
- **Waypoints Editor**
  - YAML 불러오기 → 테이블 표시
  - 테이블 편집 → YAML 저장
  - abs/rel 이동 타입 지원, rpy 옵션 지원
- **FK Capture**
  - 현재 관절 상태(`/joint_states`) 기반으로 `/compute_fk` 호출
  - 캡처된 end-effector XYZ를 폼에 자동 입력
- **Joint Jog(+/-)**
  - `/servo_node/delta_joint_cmds` (MoveIt Servo) 기반 조그
  - Servo 미사용 시 JointTrajectory 토픽 자동 탐색 후 폴백 전송

---

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Doosan ROS2 packages (dsr_bringup2 / dsr_moveit_config_* 등)
- Python3 + PyQt5
- MoveIt2

---

## Build (colcon)

cd ~/ros2_ws
colcon build --packages-select doosan_moveit_gui
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash




Run

1) 터미널 A: Doosan MoveIt bringup 실행 (e0509)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py model:=e0509

2) 터미널 B: GUI 실행
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run doosan_moveit_gui gui_app


