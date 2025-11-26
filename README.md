# OpenArm Workspace

OpenArm 프로젝트에서 사용하는 로봇 팔/목 제어 워크스페이스입니다. ROS 2 Humble과 MoveIt Servo를 기반으로 하며, Meta Quest 텔레오퍼레이션과 STS3215 목 모터 실험 코드까지 한곳에서 관리합니다.

## 구성 패키지 한눈에 보기

| 경로 | 설명 |
| --- | --- |
| `neck_motor_v2/` | STS3215 세 모터로 구성된 목 구조 제어 실험. Meta Quest 헤드 트래킹과 키보드/카메라 제어 예제가 들어 있습니다. **자세한 사용법은 `neck_motor_v2/README.md`를 참고하세요.** |
| `src/openarm_can/` | 다미아오 모터용 SocketCAN(CAN 2.0/CAN-FD) C++ 라이브러리와 유틸리티. 시스템 설치 패키지, Python 바인딩 샘플, `configure_socketcan.sh` 같은 인터페이스 스크립트를 제공합니다. |
| `src/openarm_description/` | OpenArm URDF/Xacro, 메쉬, MoveIt 설정에서 사용하는 모델 정의. 다른 패키지에서 로봇 설명을 포함할 때 사용됩니다. |
| `src/openarm_ros2/` | 공식 ROS 2 패키지 모음 (bringup, 하드웨어 플러그인, MoveIt 설정). 문서화된 OpenArm ROS 2 릴리스를 그대로 포함합니다. |
| `src/openarm_servo/` | 이 워크스페이스에서 직접 수정하는 텔레오퍼레이션/Servo 실험 패키지. MoveIt Servo 실행, Quest·키보드 입력 노드, 모니터링 스크립트를 포함합니다. 아래에서 자세히 설명합니다. |

## `openarm_servo` 상세 설명

이 패키지는 MoveIt Servo 기반 실시간 제어를 위한 런치/노드 모음입니다. RViz 시뮬레이션, Meta Quest 입력, 키보드 명령, 그리퍼 액션 및 모니터링을 동일한 패키지에서 제공합니다.

### 디렉터리 구조

| 경로 | 역할 |
| --- | --- |
| `config/openarm_left_simulated_config.yaml`, `openarm_right_simulated_config.yaml` | MoveIt Servo 설정 파일. 대상 팔(좌/우)에 맞춰 조인트 제한, 필터, 스케일 등을 정의합니다. |
| `config/ros2_controllers.yaml` | `ros2_control`의 fake hardware + joint trajectory + gripper 컨트롤러 구성을 포함합니다. |
| `config/openarm_servo*.rviz` | RViz 뷰 구성을 저장한 파일. |
| `launch/servo_left.launch.py`, `servo_right.launch.py`, `servo_bimanual.launch.py` | 각 팔/양팔 시나리오에서 MoveIt Servo 스택과 RViz, 컨트롤러를 기동하는 런치 파일. |
| `openarm_servo/*.py` | 텔레오퍼레이션 및 유틸리티 노드. `quest_*.py`, `keyboard_*.py`, `homing_right.py`, `servo_monitor.py` 등. |

### 런치 파일과 기동 노드

**`ros2 launch openarm_servo servo_left.launch.py`**

- `rviz2` – `config/openarm_servo.rviz` 로딩
- `controller_manager` – `ros2_control_node` + `ros2_controllers.yaml`
  - `joint_state_broadcaster`
  - `left_joint_trajectory_controller`
  - `left_gripper_controller`
- `robot_state_publisher` (Composable)
- `moveit_servo::servo_node_main` (기본 네임스페이스)
- `moveit_servo::JoyToServoPub` + `joy_node` (Composable container)

**`ros2 launch openarm_servo servo_right.launch.py`**

- 왼팔 런치와 동일한 구조지만, `right_joint_trajectory_controller` 및 `right_gripper_controller`를 스폰하고 Servo 설정을 `openarm_right_simulated_config.yaml`로 교체합니다.
- 나머지 구성(RViz, robot_state_publisher, JoyToServoPub)은 동일하게 기동합니다.

**`ros2 launch openarm_servo servo_bimanual.launch.py`**

- RViz 구성 파일을 `config/openarm_servo_bimanual.rviz`로 교체하고, 좌/우 조인트/그리퍼 컨트롤러 네 개를 모두 스폰합니다.
- `robot_state_publisher`, `JoyToServoPub`, `joy_node`는 공용 컨테이너에서 실행됩니다.
- `moveit_servo::servo_node_main`을 **두 번** 띄워 각각 `left/servo_node`와 `right/servo_node` 네임스페이스를 사용합니다. 런치 파일에서 3초 지연 후 `/left/servo_node/start_servo`, `/right/servo_node/start_servo` 서비스를 자동 호출해 양팔 Servo를 동시에 활성화합니다.

위 런치 파일들은 모두 기본값으로 `use_fake_hardware:=true`를 사용하므로 물리 하드웨어 없이도 RViz에서 동작을 확인할 수 있습니다. 실제 장비를 사용할 때는 MoveIt config 매핑에서 `use_fake_hardware`를 `false`로 바꾸고, `ros2_controllers.yaml`을 실제 하드웨어 인터페이스에 맞게 수정하세요.

### 텔레오퍼레이션 노드

| 스크립트 | 설명 / 주입 토픽 |
| --- | --- |
| `quest_servo_left.py`, `quest_servo_right.py`, `quest_servo_both.py` | Meta Quest 헤드셋에서 들어오는 TCP 5454 JSON (`{"head":{"euler":...}}`) 혹은 컨트롤러 pose 데이터를 수신합니다. 3~5초간 평균을 내서 기준 자세를 잡은 뒤, `JointJog`와 `TwistStamped` 명령을 `/servo_node/delta_*` 토픽으로 전송합니다. 그리퍼는 `/left_gripper_controller/gripper_cmd` 또는 `/right_gripper_controller/gripper_cmd` 액션으로 제어합니다. |
| `keyboard_servo_left.py`, `keyboard_servo_right.py`, `keyboard_servo_both.py` | 터미널에서 키 입력을 받아 Cartesian Twist 또는 JointJog 명령을 보냅니다. 화살표/`UOIJLK`로 위치·자세 제어, `1-7`로 개별 조인트 조그, `G/H`로 그리퍼 개폐 등 단축키가 포함되어 있습니다. |
| `homing_right.py` | 오른팔의 `joint1`, `joint4`를 지정된 홈 위치로 순차적으로 이동시키는 간단한 호밍 루틴. Quest 텔레오퍼레이션 전에 기준을 맞출 때 사용합니다. |
| `servo_monitor.py` | `/servo_node/delta_twist_cmds`, `/left_joint_trajectory_controller/joint_trajectory`, TF (`openarm_body_link0 -> openarm_left_hand`)를 동시에 구독하여 명령/결과/EE pose를 터미널에 출력합니다. |

#### 실행 순서 예시

1. **MoveIt Servo 스택 기동**
   ```bash
   source install/setup.bash
   ros2 launch openarm_servo servo_left.launch.py
   ```
2. **제어 노드 실행**
   - Quest 텔레오퍼레이션: `ros2 run openarm_servo quest_servo_left`
   - 키보드 제어: `ros2 run openarm_servo keyboard_servo_left`
   - 모니터링: `ros2 run openarm_servo servo_monitor`
3. **Quest 데이터 소스** – Meta Quest 애플리케이션이 로컬 네트워크에서 TCP 5454로 pose 데이터를 보내야 합니다. 기본 포트/호스트는 각 스크립트 상단 상수로 수정할 수 있습니다.

### 팁

- `quest_servo_*` 스크립트는 5초간 캘리브레이션 구간이 있으므로 Quest 착용자가 중립 자세를 유지해야 합니다.
- 양팔을 동시에 제어하려면 `servo_bimanual.launch.py`를 띄운 후 `quest_servo_both` 또는 `keyboard_servo_both`를 실행합니다. 각 노드는 좌/우 인풋을 분리하여 `/servo_node/delta_*` 토픽을 동시에 발행합니다.
- Fake hardware가 아닌 실제 팔에 명령하려면 `openarm_ros2` 패키지의 하드웨어 드라이버를 올리고, `ros2_controllers.yaml` 및 MoveIt config에서 올바른 컨트롤러/하드웨어 인터페이스로 전환하십시오.

## 추가 자료

- Neck motor 실험: `neck_motor_v2/README.md`
- CAN/URDF/ROS 2 공식 문서: https://docs.openarm.dev/
- Quest pose 스트리밍 참고 스크립트: `quest_get_pose.py`
