# Neck Motor v2 (STS3215)

Meta Quest 헤드 트래킹 데이터와 Feetech STS3215 3축 목 모터를 직접 연동하기 위한 V2 실험 코드 모음입니다. `lerobot`에 의존하지 않고, 순수 Python + vendor SDK(feetech-servo-sdk)만으로 모터를 제어하며, 카메라 스트리밍/키보드 제어/ID 셋업까지 한 폴더에서 관리합니다.

## 의존성

```bash
pip install feetech-servo-sdk opencv-python numpy
```

- `feetech-servo-sdk` (a.k.a. `scservo_sdk`) 필수
- 카메라/비디오 스크립트는 `opencv-python` 필요
- `/dev/ttyUSB*` 접근을 위해 OS에서 dialout 그룹 권한 혹은 udev 규칙을 맞춰 주세요.
- STS/SMS 시리즈: Protocol 0, 기본 보드레이트 1,000,000 bps

## 목 모터 매핑

| 이름 | ID | 동작 |
| --- | --- | --- |
| `tilt`   | 4 | 좌/우 기울기 (roll) |
| `rotate` | 5 | 좌/우 회전 (yaw) |
| `nod`    | 6 | 앞/뒤 끄덕임 (pitch) |

`controller.STS3215Controller`는 위 매핑을 기본값으로 사용합니다. 다른 ID를 쓰려면 `motor_ids` 또는 `motors` 인자를 직접 넘겨 주세요.

## Quick start

```python
from controller import STS3215Controller

controller = STS3215Controller(port="/dev/ttyUSB0")
controller.open()
controller.enable_torque(True)

positions = controller.read_positions()
controller.command_positions({"nod": positions["nod"] + 200})

controller.enable_torque(False)
controller.close()
```

단순 실험은 다음 스크립트로 바로 가능합니다.

```bash
# 실시간 위치 모니터 (토크 OFF, 50Hz 업데이트)
python monitor_motor_value.py

# 화살표/W/S를 이용한 수동 제어
python keyboard_control.py
```

포트(`/dev/ttyUSB*`)나 ID가 다르면 스크립트 상단 상수를 수정하거나 `STS3215Controller` 생성자 인자를 교체하세요.

## 파일 개요

| 파일 | 주요 기능 |
| --- | --- |
| `controller.py` | STS3215 전용 얇은 래퍼. 그룹 읽기/쓰기 준비, 이름 기반 제어, 토크 온/오프 제공. |
| `monitor_motor_value.py` | 토크를 끈 상태에서 현재 위치를 Raw/Deg 표로 출력. 모터를 손으로 움직이며 캘리브레이션 확인할 때 사용. |
| `keyboard_control.py` | 화살표(rotate/nod) + W/S(tilt)로 직접 제어. 초기값/범위 제한 포함. |
| `setup_id.py` | 모터 ID 스캔/재설정 CLI. `--scan`, `--current-id --new-id`, `--sequence` 지원. |
| `quest_sync.py` | Meta Quest → TCP(5454)로 들어오는 `head.euler` 데이터를 받아 모터에 반영. 3초간 자동 캘리브레이션 후 추적. |
| `neck_control_with_camera.py` | `quest_sync` + USB 카메라 뷰어(OpenCV 창). Quest는 TCP 5454, 카메라는 로컬 모니터링. |
| `neck_control_full.py` | Quest 포즈 제어 + USB 카메라 캡처 + Quest로 직접 스트리밍(TCP 5656, JPEG). 헤드셋을 단독 디스플레이로 쓰는 풀 파이프라인. |

### Meta Quest 연동 흐름

1. Quest 측 앱이 TCP 5454로 헤드 추정치를 JSON(`{"head":{"euler":{"x":..,"y":..,"z":..}}}`) 형태로 스트리밍.
2. `quest_sync.py` 혹은 두 컨트롤 스크립트가 첫 3초간 평균값을 취해 기준 자세를 구함.
3. 이후 yaw/pitch/roll 편차를 `SCALE_*`만큼 raw ticks로 변환해 세 모터에 명령.
4. 필요 시 `MAX_ANGLE_DELTA`로 안전 범위를 줄이고, `INITIAL_*` 상수로 중심 위치를 조정.

### 카메라/비디오 파이프라인

- `neck_control_with_camera.py`: OpenCV 윈도우에 USB 카메라를 띄워 조작자가 PC에서 피드백을 확인.
- `neck_control_full.py`: 비디오 서버 스레드가 JPEG를 크기(4바이트 big-endian) + payload로 보내므로 Quest 쪽 클라이언트는 동일 프로토콜을 구현해야 함. 기본 포트 5656, 해상도 640x480, 품질 70.

## 모터 ID 및 캘리브레이션

1. 새 모터는 임의 ID일 수 있으므로 먼저 `python setup_id.py --scan`으로 확인.
2. ID를 순서대로 부여하려면 `python setup_id.py --sequence 4 5 6`; 각 단계마다 단일 모터만 연결.
3. 모터 위치를 손으로 맞출 때 `monitor_motor_value.py`를 실행하면 Raw/Deg 값을 즉시 확인할 수 있음.
4. Quest 데이터 수신 전에 목을 중립 자세로 고정하면 보다 정확한 기준점을 얻을 수 있음.

## 네트워크/시리얼 유의사항

- 모든 Quest 통신 스크립트는 기본적으로 **TCP**를 사용합니다. 포트를 방화벽에서 열어 두고, 동일 네트워크 대역에 있어야 합니다.
- 카메라 장치 번호(`CAMERA_INDEX`)는 시스템에 따라 다르므로 `neck_control_*` 파일 상단을 수정하세요.
- 시리얼 포트 작업 중 예외가 발생해도 토크를 반드시 해제하도록 `Ctrl+C` 이후 로그에 안내가 출력되니, 종료 메시지를 확인하고 포트를 분리하세요.

## 참고 주소 (STS/SMS 시리즈)

- `Torque_Enable (40)`
- `Goal_Position (42)` – 0~4095 (12bit)
- `Present_Position (56)`

필요하다면 `controller.py` 코드를 참고해 직접 다른 제어 테이블 주소를 읽거나 쓸 수 있습니다.
