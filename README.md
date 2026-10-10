# ADAS Wildlife Deterrent

Arducam B0538C 영상에서 멧돼지·고라니를 탐지하고, 선택한 동물의 bbox 중심으로
카메라를 조준하며, YOLO가 판별한 종에 맞는 WAV를 재생한다. 모터 사용 여부는
`settings.py`에서 정한다. 동물 YOLO 가중치는 `assets/models/weights.pt`에 포함돼 있다.

```text
카메라 → 동물 YOLO → 대상 선택 ┬→ DM4310 카메라 조준 (ENABLE_MOTOR)
                              └→ YOLO 클래스 → 재생 정책 → 무작위 WAV
```

## 구성

- `main.py`: 동물 탐지 실행 진입점과 화면 표시
- `settings.py`: 모델·카메라·모터·음향 설정 상수
- `camera.py`: Arducam YUY2 캡처, 최신 프레임 전달, 재연결
- `wildlife.py`: YOLO 결과·대상 선택·종 분류·재생 정책과 모터 어댑터
- `audio.py`: WAV 목록 검증·무작위 선택·재생
- `assets/models/weights.pt`: 로컬 동물 탐지 YOLO 가중치
- `assets/audio/`: 멧돼지·고라니 WAV와 출처 매니페스트
- `motor/`: Damiao DM4310-2EC 2축 CAN 제어
- `tools/zero_actuator.py`: 모터 영점 설정
- `tools/actuator_conn_test.py`, `actuator_test.sh`: CAN ID 설정·통신·위치·영점 점검
- `tools/motor_goto.py`, `motor_start.sh`: 지정한 픽셀 좌표로 두 축 이동 후 유지
- `tools/motor_manual_control.py`: 키보드 수동 제어와 가상 좌표 추적 점검

## 준비

```bash
uv sync
```

기본으로 `assets/models/weights.pt`의 가중치를 사용한다. 다른 모델을 쓰려면
`settings.py`의 `MODEL_PATH`를 수정한다. `CLASS_TO_AUDIO`는 실제 YOLO 클래스 ID를 `boar` 또는
`water_deer` 음원 그룹에 연결한다. `EXPECTED_MODEL_NAMES`에는 해당 ID의 실제
`model.names` 값을 적는다. 이름이 다르면 카메라·모터를 열기 전에 실행을
중단한다. 현재 로컬 모델의 클래스와 설정은 다음과 같다.

| 클래스 ID | 모델 이름 | 음원 그룹 |
| --- | --- | --- |
| 0 | `boar` | `boar` (멧돼지용) |
| 1 | `deer` | `water_deer` (고라니용) |
| 2 | `torso` | 조준·재생 대상에서 제외 |

카메라는 기본 인덱스 4의 Arducam B0538C(2592×1944, YUY2)를 사용한다.
인덱스 4는 아직 실측 전이므로 연결 후 `/dev/video*`를 확인하고
`settings.py`의 `CAMERA_INDEX`를 수정한다. 모터를 사용할 경우 Damiao
USB-CAN 연결과 `motor/dm4310_tracker.py`의 가동범위·제어 게인을 실제 기구에
맞게 확인한다. 기본 시리얼 장치는 `/dev/ttyACM0`이며 `DEVICENAME`에서 바꾼다.
현재 기구에서 확인한 CAN ID는 Yaw=`0x01`, Pitch=`0x02`이고,
`SIGN_YAW = SIGN_PITCH = -1`, `KP = 500.0`, `KD = 5.0`을 유지한다.
`Tracker`는 초기화할 때 두 축을 MIT 모드로 전환한다.

## 실행

```bash
uv run main.py
```

처음 점검할 때 `settings.py`의 `DRY_RUN = True`로 두면 음원 경로만 출력한다.
소리를 내려면 `DRY_RUN = False`, 모터를 사용하려면 `ENABLE_MOTOR = True`로
설정한다. 화면에서 `q`는 종료, `e`는 모터 중앙 복귀다. 화면 없이 실행하려면
`HEADLESS = True`로 바꾸고 Ctrl+C로 종료한다. WAV 재생 명령은 Linux에서
`aplay`, macOS에서 `afplay`이며 `AUDIO_PLAYER_COMMAND`로 바꿀 수 있다.

모델·카메라 없이 클래스별 음성 재생을 확인하려면 `tools/play_audio.py`의 `CLASS_ID`를
설정하고 실행한다. 현재 매핑에서는 `0`이 멧돼지용, `1`이 고라니용 음원이다. 한 번 실행할
때 해당 그룹에서 WAV 하나를 무작위로 선택해 재생한다.

```bash
uv run python tools/play_audio.py
```

동물 YOLO가 종을 직접 구분해야 현재 기본 분류기를 사용할 수 있다. 동물만
탐지하는 모델이라면 별도 종 분류 단계를 추가해야 한다. 대상은 프레임마다
허용된 클래스 중 신뢰도가 가장 높은 하나를
고른다. 여러 동물이 있을 때 객체 ID를 유지하는 추적 기능은 아직 없다.

기본 재생 규칙은 3프레임 연속 탐지, 탐지 사건당 1회, 모든 클래스 공통 10초
간격이다. 음원별 출처와 이용 조건은 [매니페스트](assets/audio/manifest.json)에
있다. 실제 동물 회피 효과와
카메라·모터·스피커를 연결한 동작은 검증 전이다.

## 모터 점검 도구

카메라·YOLO 없이 모터를 점검하는 도구다. 수동 제어와 좌표 이동은 실제
모터를 구동하며, 좌표 기준은 `motor/dm4310_tracker.py`의 2592×1944다.

```bash
# CAN ID 찾기·변경, 통신·위치·영점 점검 메뉴
./actuator_test.sh

# 지정 좌표로 두 축 이동 후 유지, Ctrl+C로 종료
./motor_start.sh 1200 900

# 키보드 수동 제어와 가상 좌표 추적
uv run python tools/motor_manual_control.py

# 두 축의 영점 설정
uv run python tools/zero_actuator.py
```

영점 설정 도구는 모터를 활성화하지 않고, 손으로 맞춘 위치를 영점으로
지정한 뒤 플래시에 저장한다. CAN ID 변경도 플래시에 저장한다.
`actuator_test.sh`의 게인 설정은 해당 메뉴 실행 동안만 적용되며 운영
`Tracker`의 `KP`/`KD`를 변경하지 않는다. 수동 제어 메뉴의 `START` 항목은
아직 구현되지 않았다.
