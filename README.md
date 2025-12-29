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

















## Build (colcon)

> 아래는 `~/ros2_ws/src/doosan_moveit_gui`에 이 패키지가 있다고 가정합니다.

```bash
cd ~/ros2_ws
colcon build --packages-select doosan_moveit_gui
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
