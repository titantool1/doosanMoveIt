Quick Check (bringup 확인)
Bringup이 제대로 떴는지 아래로 빠르게 확인할 수 있습니다.

# move_group 떠있나?
ros2 node list | grep move_group

# SRDF(semantic) 로드 됐나? (robot name이 e0509로 나오는지 확인)
ros2 param get /move_group robot_description_semantic

# robot_description 토픽이 있나?
ros2 topic list | grep robot_description

# joint_states 들어오나?
ros2 topic echo /joint_states --once



ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py가 두개켜져 있으면 안된다
그래서 켜져 있던걸 잘 끄고 한개를 실행하는 방법


ros2 daemon stop
ros2 daemon start
#2) 남은 프로세스/컨테이너 싹 정리 (중복 launch 방지)
#bash
#코드 복사
pkill -f ros2_control_node || true
pkill -f controller_manager || true
pkill -f move_group || true
pkill -f robot_state_publisher || true
pkill -f run_emulator || true
pkill -f dsr_ || true
docker rm -f emulator 2>/dev/null || true


cd ~/ros2_ws_test
source /opt/ros/humble/setup.bash
source ~/ros2_ws_test/install/setup.bash

#export QT_QPA_PLATFORM=offscreen  # RViz/Qt가 
#display 없다고 죽는 걸 조금이라도 완화(완전 해결은 아님)

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  model:=e0509 \
  host:=127.0.0.1 \
  port:=12345 \
  mode:=virtual \
  rt_host:=127.0.0.1
