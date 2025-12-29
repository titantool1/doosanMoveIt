# doosan_moveit_gui (ROS2 Humble)

PyQt5 기반의 Doosan MoveIt2 조작 GUI입니다.  
Waypoints(YAML) 편집/저장 + 현재 로봇 자세(FK) 캡처 + RViz2 시각화(버튼 실행) 기능을 제공합니다.

---

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
- Doosan ROS2 packages (`dsr_bringup2`, `dsr_moveit_config_*`)
- Python3 + PyQt5
- MoveIt2
- python3-yaml는 
---

sudo apt-get update

sudo apt-get install -y \
  python3-pyqt5 \
  python3-yaml \
  ros-humble-rclpy \
  ros-humble-ament-index-python \
  ros-humble-sensor-msgs \
  ros-humble-moveit-msgs \
  ros-humble-control-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-builtin-interfaces \
  ros-humble-rviz2
  ros-humble-moveit \
  ros-humble-moveit-servo
  python3-colcon-common-extensions \
  python3-rosdep \
  git
  dbus-x11 \
  mesa-utils \
  libgl1-mesa-dri
---
sudo rosdep init || true
rosdep update

---

cd ~/ros2_ws/src
git clone https://github.com/DoosanRobotics/doosan-robot2.git

---

이걸로 package.xml에 적어둔 의존성”을 기준으로 알아서 apt를 깔아줘.
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y






---
##Build (colcon)
cd ~/ros2_ws
colcon build --packages-select doosan_moveit_gui
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
---
##Run

중요: GUI는 MoveIt bringup이 떠 있어야 정상 동작합니다. (특히 /robot_description, /compute_fk, /joint_states)

1) Terminal A: Doosan MoveIt bringup (e0509)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py model:=e0509

2) Terminal B: GUI 실행
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run doosan_moveit_gui gui_app

---














# doosan_moveit_gui (ROS 2 Humble)

PyQt5 기반의 Doosan MoveIt2 조작 GUI입니다.  
Waypoints(YAML) 편집/저장 + 현재 로봇 자세(FK) 캡처 + RViz2 시각화(버튼 실행) 기능을 제공합니다.

## Features

- **RViz2 실행 버튼** (TF + RobotModel + Grid 표시)
- **Waypoints Editor**
  - YAML 불러오기 → 테이블 표시
  - 테이블 편집 → YAML 저장
  - abs/rel 이동 타입 지원, rpy 옵션 지원
- **FK Capture**
  - `/joint_states` 기반으로 MoveIt `/compute_fk` 호출
  - 캡처된 end-effector XYZ를 폼에 자동 입력
- **Joint Jog (+/-)**
  - `/servo_node/delta_joint_cmds` (MoveIt Servo) 기반 조그
  - Servo 미사용 시 JointTrajectory 토픽 자동 탐색 후 폴백 전송

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt2 (RViz2 포함)
- Doosan ROS2 packages (예: `dsr_bringup2`, `dsr_moveit_config_*`)
  - ⚠️ 이 레포는 Doosan 패키지를 “포함”하지 않습니다. (사용자 환경에 설치/빌드되어 있어야 함)

---

## Install (apt) — 4 Blocks

> 아래는 “GUI 패키지 실행”을 위한 설치 블록입니다.  
> Doosan 드라이버/bringup은 각자 환경에 맞게 별도로 준비되어 있어야 합니다.

### 1) Minimal (GUI + 기본 메시지)
```bash
sudo apt-get update
sudo apt-get install -y \
  python3-pyqt5 \
  python3-yaml \
  ros-humble-rclpy \
  ros-humble-sensor-msgs \
  ros-humble-control-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-moveit-msgs \
  ros-humble-rviz2
2) Recommended (MoveIt2 기본 구성 + TF)
bash
코드 복사
sudo apt-get update
sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
3) Dev Tools (빌드/의존성)
bash
코드 복사
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  git
4) XRDP/가상환경에서 RViz가 느리거나 안 뜰 때(권장)
bash
코드 복사
sudo apt-get update
sudo apt-get install -y \
  mesa-utils \
  libgl1-mesa-dri
Build (colcon)
bash
코드 복사
cd ~/ros2_ws
# src/ 아래에 이 레포(doosan_moveit_gui)가 들어있다고 가정
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select doosan_moveit_gui
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
Run
Terminal A — Doosan MoveIt bringup (예: e0509)
bash
코드 복사
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py model:=e0509
Terminal B — GUI 실행
bash
코드 복사
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run doosan_moveit_gui gui_app
Quick Check (bringup 확인)
Bringup이 제대로 떴는지 아래로 빠르게 확인할 수 있습니다.

bash
코드 복사
# move_group 떠있나?
ros2 node list | grep move_group

# SRDF(semantic) 로드 됐나? (robot name이 e0509로 나오는지 확인)
ros2 param get /move_group robot_description_semantic

# robot_description 토픽이 있나?
ros2 topic list | grep robot_description

# joint_states 들어오나?
ros2 topic echo /joint_states --once
Troubleshooting
문제 해결 모음: Troubleshooting.md

Screenshots
여기에 스크린샷을 넣으세요 (assets/ 폴더 권장)

GUI main

RViz open

Waypoints editor

FK capture
