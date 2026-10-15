# ADAS Project

**같은 Damiao DM-J4310-2EC 2축(Yaw/Pitch) CAN 짐벌 + Arducam B0538C 카메라**
하드웨어를 공유하는 애플리케이션 두 개를 담은 저장소.

| 앱 | 진입점 | 하는 일 |
| --- | --- | --- |
| 야생동물 음향 퇴치 | `main.py` | 멧돼지·고라니 탐지 → bbox 중심으로 짐벌 조준 → 종에 맞는 WAV 재생 |
| 얼굴 추적 | `people/main_facetrack.py` | 손 든 사람 확정 → 코 좌표로 짐벌 추적 → 상황별 안내 음성 |

모터(`motor/`)와 테스트 도구(`tools/`, `start.sh`, `actuator_test.sh`)는 두 앱이 공유한다.

## 목차

- [Setup](#setup)
- [구성](#구성)
- [야생동물 탐지 — `main.py`](#야생동물-탐지--mainpy)
- [얼굴 추적 — `people/main_facetrack.py`](#얼굴-추적--peoplemain_facetrackpy)
- [공용 모터 — `motor/dm4310_tracker.py`](#공용-모터--motordm4310_trackerpy)
- [테스트 도구](#테스트-도구)
- [음향 테스트 데이터 — `audio_test/`](#음향-테스트-데이터--audio_test)
- [튜닝 중인 값 / 알아둘 것](#튜닝-중인-값--알아둘-것)

## Setup

```bash
uv sync
```

하드웨어 준비 체크리스트:

| 항목 | 확인된 값 |
| --- | --- |
| 카메라 | Arducam B0538C(OG05B1B), 인덱스 **4** = `/dev/video4`. USB 3.0 연결 시 2592×1944@25fps(USB 2.0은 5fps로 급감) |
| CAN 동글 | Damiao 전용 USB-CAN, 이 프로젝트 기준 `/dev/ttyACM0` (동글 칩셋에 따라 `/dev/ttyUSB0`일 수도 있음 — `ls /dev/ttyUSB* /dev/ttyACM*`로 재확인) |
| CAN ID | Yaw=`0x01`, Pitch=`0x02` (MasterID `0x11`/`0x12`) — `./actuator_test.sh` 1번(스캔)으로 확인/변경 가능 |
| MIT 모드 | `Tracker` 초기화 시 자동 전환, 별도 사전 설정 불필요 |

카메라 인덱스를 바꾸려면 야생동물 쪽은 `animal/settings.py`의 `CAMERA_INDEX`,
얼굴추적 쪽은 `people/main_facetrack.py`의 `CAMERA_IDX`. 게인·가동범위 등
튜닝값은 [튜닝 중인 값 / 알아둘 것](#튜닝-중인-값--알아둘-것) 참고.

## 구성

```
main.py                        야생동물 탐지+음향퇴치 실행 진입점
start.sh                       모터+카메라 통합 테스트 메뉴 (4개 항목)
actuator_test.sh               액추에이터 단독 점검 메뉴 (8개 항목)

animal/                        main.py 가 쓰는 모듈 (main.py 자신만 빼고 전부 이 안에)
  settings.py                    모델·카메라·모터·음향 설정 상수
  camera.py                      Arducam YUY2 캡처, 최신 프레임 전달, 재연결
  wildlife.py                    YOLO 결과·대상 선택·종 분류·재생 정책·모터 어댑터
  audio.py                       WAV 목록 검증·무작위 선택·재생
  mp3_player.py                  MP3 재생기 (pygame) — people/, tools/ 도 같이 씀
assets/models/weights.pt       동물 탐지 YOLO 가중치
assets/audio/                  멧돼지·고라니 WAV + 출처 매니페스트

people/
  main_facetrack.py             얼굴 추적 실행 진입점 (구 main.py)
detection/
  base.py                        PoseDetector 인터페이스, PersonPose 데이터 구조
  yolo_pose.py                   YOLOv8 pose 기반 PoseDetector 구현체
assets/models/yolov8n-pose.pt  YOLOv8 pose 가중치 (동물 탐지와 별개)
audio/                         얼굴추적 상황별 안내 음성 (mp3)
legacy/                        이전 세대 코드 (Dynamixel 트래커, 라즈베리파이 스트림 서버 — 참고용)

motor/                         공용 — 모터/CAN
  dm4310_tracker.py              Damiao DM-J4310-2EC 트래커 (Tracker 클래스)
  DM_CAN.py                      Damiao 저수준 CAN 드라이버 (벤더링)

tools/                         공용 — 점검 도구
  actuator_conn_test.py           액추에이터 1개 연결/신호조회/위치신호/제로잉
  camera_test.py                  카메라 단독 테스트
  motor_goto.py                   지정 좌표로 이동 후 유지 (독립 실행)
  motor_manual_control.py         start.sh 가 실행하는 실제 구현
  zero_actuator.py                영점 설정
  play_audio.py                   클래스별 음성 재생만 확인 (야생동물용)

audio_test/                    스피커 음향 측정 데이터 시트 (CSV)
```

## 야생동물 탐지 — `main.py`

```text
카메라 → 동물 YOLO → 대상 선택 ┬→ DM4310 카메라 조준 (ENABLE_MOTOR)
                              └→ YOLO 클래스 → 재생 정책 → 무작위 WAV
```

기본 가중치는 `assets/models/weights.pt`. 모델을 바꾸려면 `animal/settings.py`의
`MODEL_PATH`를 수정하고, `CLASS_TO_AUDIO`(클래스ID→음원 그룹)와
`EXPECTED_MODEL_NAMES`(해당 ID의 실제 모델 이름)를 실제 모델에 맞게 고친다 —
안 맞으면 카메라·모터를 열기 전에 실행이 중단된다. 현재 로컬 모델 기준:

| 클래스 ID | 모델 이름 | 음원 그룹 |
| --- | --- | --- |
| 0 | `boar` | `boar` (멧돼지) |
| 1 | `deer` | `water_deer` (고라니) |
| 2 | `torso` | 조준·재생 대상 제외 |

```bash
uv run main.py
```

| 설정 (`animal/settings.py`) | 기본값 | 의미 |
| --- | --- | --- |
| `DRY_RUN` | `False` | `True`면 음원 경로만 출력(소리 안 냄) — 처음 점검할 때 추천 |
| `ENABLE_MOTOR` | `False` | `True`면 모터로 실제 조준 |
| `HEADLESS` | `False` | `True`면 화면 없이 실행(Ctrl+C로 종료) |
| `AUDIO_PLAYER_COMMAND` | 자동(`aplay`/`afplay`) | WAV 재생 명령 override |

실행 중 키: `q`=종료, `e`=모터 중앙 복귀.

재생 규칙: 3프레임 연속 탐지 시 탐지 사건당 1회, 클래스 공통 쿨다운 10초.
대상은 프레임마다 허용 클래스 중 신뢰도 최고 1개(여러 동물 간 ID 유지 추적은
아직 없음). 음원 출처는 [매니페스트](assets/audio/manifest.json) 참고.

모델·카메라 없이 음성만 확인:

```bash
uv run python tools/play_audio.py   # tools/play_audio.py 의 CLASS_ID로 그룹 지정
```

> 실제 동물 회피 효과, 카메라·모터·스피커 통합 야외 동작은 아직 검증 전이다.

## 얼굴 추적 — `people/main_facetrack.py`

```text
Arducam(25fps) → YOLOv8 pose → 손든지 ARM_UP_THRESHOLD(3초) 최대박스 확정
              → 코 좌표(얼굴박스로 노이즈 필터) → Tracker.update(x, y)
              → Yaw 각도+박스 크기로 구간이탈 판정 → 방향별 안내 음성
```

```bash
uv run people/main_facetrack.py           # 카메라 + YOLO 추적만
uv run people/main_facetrack.py --motor   # + Damiao 모터 제어
```

| 키 | 동작 |
| --- | --- |
| `w` | 재추적 (현재 트래킹 해제, 손든 사람 대기로 복귀) |
| `e` | `w` + 모터 중앙 복귀 |
| `s` | 샘플 음성 재생 |
| `+` / `-` | 볼륨 ±10% |
| `0` | 볼륨 기본값 복귀 |
| `o` / `p` | 안내 음성(greenlight/finish) 재생 |
| `q` | 종료 |

- `detection/base.py`의 `PoseDetector` 인터페이스(`detect(frame) -> List[PersonPose]`)만
  지키면 어떤 모델로도 교체 가능 — 기본 구현은 `detection/yolo_pose.py`의
  `YoloPoseDetector`(YOLOv8 pose, `assets/models/yolov8n-pose.pt`, GPU 자동 사용).
  파인튜닝 모델로 바꾸려면 `people/main_facetrack.py`의 `MODEL_PATH`만 수정.
- `animal/mp3_player.py`/`audio/*.mp3`가 안내 음성을 담당.
- `legacy/`: `face_tracker_xl430_06_14.py`(이전 세대 Dynamixel 트래커),
  `local.py`(라즈베리파이 영상 스트림 서버, 별개 용도) — 둘 다 참고용.

## 공용 모터 — `motor/dm4310_tracker.py`

Yaw/Pitch DM4310 모터 2개를 Damiao USB-CAN 동글(921600 baud, 저수준 프레이밍은
`motor/DM_CAN.py`)로 MIT 모드 제어한다.

```
torque = Kp·(q_des − q) + Kd·(dq_des − dq) + tau_ff
```

이 식은 **모터 자신의 MCU**가 계산한다. 호스트(`Tracker`)는 목표 위치
`q_des`만 CAN으로 계속 갱신해서 보내고, 부드러움은 MIT 임피던스(Kp/Kd)가
물리적으로 담당한다.

- **클로즈드 루프**: 매 CAN 사이클 모터가 돌려주는 실측 위치/속도/토크를
  다음 목표 계산의 기준으로 삼는다(호스트 시뮬레이션 값이 아님).
- **제어 스레드**: `Tracker` 인스턴스당 1개, 목표 `CONTROL_FREQ` Hz로 계속
  스트리밍. 실측 주파수는 `get_actual_freq()`, 그 외에도 카메라→목표좌표
  Hz(`get_update_freq()`), CAN 송신 Hz(`get_can_tx_freq()`), 축별 수신 Hz
  (`get_rx_freq(axis)`)를 전부 실측할 수 있다 — `start.sh` 1·2번 화면에
  실시간으로 표시됨.
- **`Tracker.update(x, y)`**: 픽셀 오차 × `RAD_PER_PIXEL_X/Y` → 각도 스텝 →
  실측 현재 위치에 더해서 다음 목표로 삼는다.
- **`Tracker.go_to(yaw_rad, pitch_rad, duration)`**: 임의의 목표로 선형 램프
  이동 (`go_center()`는 이걸 호출하는 (0,0) 특수 케이스).
- 축별 임피던스 게인(`KP_YAW`/`KD_YAW`, `KP_PITCH`/`KD_PITCH`)이 서로
  독립이고, Pitch 전용 중력보상 피드포워드 `TAU_FF_PITCH`로 자중으로 인한
  steady-state 오차를 보정한다 (Yaw는 `TAU_FF_YAW=0` 고정) — 배경은
  [튜닝 중인 값](#튜닝-중인-값--알아둘-것) 참고.

## 테스트 도구

### `./actuator_test.sh` — 액추에이터 단독 점검

카메라·YOLO 없이 모터 1개씩 점검한다. 포트는 `motor/dm4310_tracker.py`의
`DEVICENAME`을 그대로 쓰므로, 포트가 바뀌면 그 한 곳만 고치면 된다.

| # | 메뉴 | 비고 |
| --- | --- | --- |
| 1 | CAN ID 찾기/할당 (스캔) | CAN ID 몰라도 됨 — 1~10 스캔, 모터 1대만 연결 |
| 2 | CAN ID 설정 (직접 입력) | ID를 이미 알 때 |
| 3 | 신호 조회 | 토크 없이 통신 생존만 확인 |
| 4 | 위치 신호 (MIT 테스트) | 작게 왕복 |
| 5 | 위치 이동 | 목표로 이동 후 유지(왕복 안 함), Ctrl+C로 종료 |
| 6 | 제로잉 | 영점 설정 |
| 7 | 게인 설정 | 이번 실행에만 적용(파일 상수는 안 바뀜) |
| 8 | 카메라 테스트 | 모터/CAN 무관 |

3~6번은 1·2번으로 CAN ID가 지정돼야 실행된다.

### `./start.sh` — 모터+카메라 통합 테스트

`tools/motor_manual_control.py`를 실행한다.

| # | 메뉴 | 키 |
| --- | --- | --- |
| 1 | 동물 추적 + 종별 음향 재생 | `w`=조준 시작(타겟모드), `r`=재추적, `q`=복귀 |
| 2 | YOLO 코 좌표 추적 | 손 2초 이상 들면 자동 확정, `r`=재추적, `q`=복귀 |
| 3 | 방향키 위치 제어 | `a`/`d`=yaw, `w`/`s`=pitch, `q`=복귀 |
| 4 | 음향테스트용 위치 고정 (0°,0°) | `q`=복귀 |

공통 규칙:
- **박스 색**: 탐지되면 노란 박스, 추적이 확정(타겟모드)되면 초록 박스로 바뀐다.
- **화면 하단 키 힌트바**가 고정 표시된다.
- `q`/`ESC`로 **정상** 종료하면 "숙인위치"(yaw=0°, pitch=-4°)로 1초간 복귀한
  뒤 메뉴로 돌아간다. `Ctrl+C` 강제중지는 복귀 없이 바로 멈춘다(의도된 동작).
- 1·2번은 포트를 `Tracker`로 새로 열어야 해서 메뉴용 연결을 잠깐 닫았다가
  다시 연다. 3·4번은 그 연결을 그대로 쓴다.

**1번 상세**: `main.py`와 동일한 `animal/wildlife.py`/`camera.py`/`settings.py`를
재사용. 탐지(노란 박스)는 시작하자마자 바로 되지만, `w`를 눌러야 실제 조준이
시작된다(`main.py` 자체는 이 게이팅 없이 탐지 즉시 조준+재생함 — 테스트
메뉴에서만 시작 시점을 수동으로 통제). WAV 재생은 `w`와 무관하게 탐지 즉시 동작.

**2번 상세**: `people/main_facetrack.py`와 같은 `YoloPoseDetector`. 대상 몸
박스와 코 좌표를 항상 표시하고, 손을 `ARM_UP_THRESHOLD`(2초) 이상 들어야
모터가 실제로 따라간다(확정 순간 `audio/track_start2.mp3` 1회 재생).
확정이 아니면 기본 자세(0°, 30°)로 복귀한다.

```bash
# 개별 스크립트로 직접 실행하고 싶을 때
uv run python tools/motor_goto.py 1200 900     # 좌표 하나로 이동 후 유지
uv run python tools/camera_test.py             # 카메라만 미리보기
uv run python tools/camera_test.py --list      # 카메라 인덱스 스캔
uv run python tools/zero_actuator.py           # 영점 설정 (enable 안 함)
```

## 음향 테스트 데이터 — `audio_test/`

스피커 거리·각도·주파수별 음압(dB) 측정 시트. 자세한 열 구조는
[`audio_test/README.md`](audio_test/README.md) 참고.

## 튜닝 중인 값 / 알아둘 것

하드웨어 실기 테스트로 계속 바뀌는 값들 — **정확한 현재 값은 각 파일에서
직접 확인**하고(여기 못 박아두면 튜닝할 때마다 문서도 고쳐야 해서 일부러
수치를 안 적는다), 아래 "왜"만 기억해두면 된다.

| 값 | 위치 | 왜 임의로 되돌리면 안 되는지 |
| --- | --- | --- |
| `SIGN_YAW`/`SIGN_PITCH` | `motor/dm4310_tracker.py` | 이론 유도는 `-1`이지만 실측 확정값은 `+1` — 실기로 검증된 값, 다른 기구/배선이면 재검증 필요 |
| `KP_YAW`/`KD_YAW`, `KP_PITCH`/`KD_PITCH` | 〃 | 축별로 완전히 독립 — 실기로 계속 튜닝 중 |
| `TAU_FF_PITCH` | 〃 | MIT 제어식엔 적분항이 없어 Pitch 자중으로 영구적 steady-state 오차가 생김 — 이 피드포워드로 보정 중. 한 지점(30°)에서 역산한 값이라 가동범위 전체에선 `cos(θ)` 비례 오차가 남을 수 있음(완전 보정하려면 상수→각도의 함수로) |
| `HFOV_DEG`/`VFOV_DEG` | 〃 | 렌즈 스펙 이론값(82°/66°)에서 출발했지만 짐벌 기어비 미반영으로 실측이 그보다 많이 작음 — 정상이니 이론값으로 되돌리지 말 것 |
| `YAW_RANGE_RAD`/`PITCH_RANGE_RAD` | 〃 | 실제 기구 한계에 맞춰 튜닝 |

기타 확정된 값(바뀔 일 없음): 카메라 인덱스 4, CAN ID Yaw=`0x01`/Pitch=`0x02`,
제로잉/CAN ID 변경 후 `save_motor_param()`으로 플래시 저장(안 하면 전원
재인가 시 초기화), 모터 고장 코드(ERR 니블)는 `Motor.getError()`로 조회
가능(`tools/actuator_conn_test.py`가 자동 점검).
