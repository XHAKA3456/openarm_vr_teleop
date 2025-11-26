# Neck Motor (STS3215) Controller

이 디렉토리는 **lerobot** 패키지 없이 Feetech STS3215 모터 3개(목 구조용)를 제어하기 위한 최소 예제를 담고 있습니다.

## 1. 의존성

```bash
pip install feetech-servo-sdk
```

- STS/SMS 시리즈는 **Protocol 0**, 기본 보드레이트 **1,000,000 bps**를 사용합니다.
- `/dev/ttyUSB*` 장치에 접근하려면 OS에서 dialout 그룹/udev 권한을 미리 맞춰 주세요.

## 2. 파일 구성

| 파일 | 설명 |
| --- | --- |
| `controller.py` | STS3215 모터를 위한 얇은 래퍼. 현재 위치 읽기(`read_positions`), 목표 위치 쓰기(`command_positions`), 토크 온/오프(`enable_torque`) 기능 제공. |
| `monitor.py`   | 3개 모터의 현재 위치를 50Hz로 읽어 표 형태로 표시하는 실시간 모니터. |
| `assign_id.py`  | 버스에 연결된 모터 ID 스캔 및 재설정용 CLI. `--scan`으로 검색, `--current-id/--new-id`로 단일 변경, `--sequence`로 인터랙티브 일괄 등록 가능. |

### 기본 목 모터 매핑

| 이름 | ID | 동작 |
| --- | --- | --- |
| `nod` | 1 | 고개 끄덕임 (pitch) |
| `brace` | 2 | 목 굽힘/펴기 (roll/brace) |
| `rotate` | 3 | 좌우 회전 (yaw) |

`controller.py`는 위 매핑을 기본값으로 사용하므로 별도 설정 없이도 `controller.command_positions({"nod": 2000, ...})`처럼 이름 기반 제어가 가능합니다.

## 3. 사용 방법

```bash
cd neck_motor
python controller.py

# 실시간 모니터링 (표 형태 출력)
python monitor.py
```

스크립트는 기본적으로 `/dev/ttyUSB0` 포트와 ID `(1, 2, 3)` 모터를 대상으로 합니다. 환경에 맞춰 `STS3215Controller` 생성자 인자를 수정하세요.

```python
from controller import STS3215Controller, MotorConfig

controller = STS3215Controller(
    port="/dev/ttyUSB1",
    motors=(
        MotorConfig("nod", 1),
        MotorConfig("brace", 2),
        MotorConfig("rotate", 3),
    ),
)
controller.open()
controller.enable_torque(True)

positions = controller.read_positions()
print("현재 위치:", positions)

controller.command_positions({
    "nod": 1800,
    "brace": 2000,
    "rotate": 2200,
})

controller.enable_torque(False)
controller.close()
```

`monitor.py`는 토크를 자동으로 끈 상태에서 3개 모터의 현재 위치를 50Hz로 읽고, 터미널에 표 형태(`Name/ID/Raw/Deg`)로 갱신해 줍니다. 손으로 모터를 움직이면 값이 실시간으로 변하는 것을 확인할 수 있습니다.

### ID 재설정 (단일)

```bash
# 연결된 모터 검색
python assign_id.py --scan

# 예: ID 1을 11로 변경
python assign_id.py --current-id 1 --new-id 11
```

### ID 재설정 (순차/인터랙티브)

모터를 하나씩 드라이버에 연결하고, 순서대로 ID를 부여하려면 `--sequence`에 목표 ID 나열:

```bash
python assign_id.py --sequence 1 2 3
```

명령을 실행하면 각 ID마다 “모터를 연결하고 Enter” 안내가 나오며, 매번 버스에서 단일 모터를 자동 감지해 새 ID로 재설정합니다.

## 4. 주요 주소 (STS/SMS 시리즈)

- `Torque_Enable (40)`
- `Goal_Position (42)` – 0~4095 (12bit)
- `Present_Position (56)`

필요 시 다른 제어 테이블 주소를 `packet_handler.write*`/`read*` 메서드로 직접 다룰 수 있습니다.
