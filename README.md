# ADAS Project

카메라로 사람을 찾아 YOLOv8 pose로 손 든 사람을 확정한 뒤, 그 사람의 코 좌표를
따라 Damiao DM-J4310-2EC 2축(Yaw/Pitch) 짐벌이 CAN 통신으로 고속(최대 1000Hz)
추적하는 프로젝트. 트래킹 확정 및 구간 이탈 시 상황에 맞는 안내 음성도 재생한다.

## 현재 진행 상황 / 알아둘 것

- **카메라 미연결 — `CAMERA_IDX`는 아직 미검증값이다.** Arducam B0538C를 아직
  실제로 연결하지 않아서 `/dev/video*`로 몇 번이 잡힐지 모른다. 지금
  `main.py`의 `CAMERA_IDX = 4`는 임시값이고, 실제 연결 후
  `ls /dev/video*` / `v4l2-ctl --list-devices`로 재확인·수정이 필요하다 —
  USB-CAN 동글도 예상했던 `/dev/ttyUSB0`가 아니라 `/dev/ttyACM0`으로 잡힌
  전례가 있어서, 카메라도 예상과 다른 인덱스로 잡힐 가능성을 열어둘 것.
- **액추에이터 CAN ID 확정**: Yaw = CAN ID `0x01`, Pitch = CAN ID `0x02`
  (`motor/dm4310_tracker.py`).
- **Yaw/Pitch 부호(SIGN) 실측 확인 및 버그 수정 완료**: 실제 하드웨어로
  "yaw/pitch에 +각도를 명령하면 추적 대상의 픽셀 좌표가 (0,0)(좌상단) 쪽으로
  가까워진다"는 동작을 확인했다. 이에 따라 두 축 모두 `SIGN_YAW = SIGN_PITCH
  = -1`로 맞춰져 있다 — 원래 `SIGN_PITCH = +1`로 잘못 들어가 있어서 pitch
  축이 중앙으로 수렴하지 않고 발산하던 버그를 이번에 고쳤다.
- **임피던스 게인(KP/KD)은 500.0 / 5.0(DM4310 하드웨어 허용 최댓값)으로
  운용 중이다.** 사용자가 실제 하드웨어로 검증 후 의도적으로 정한 값이니
  "안전하게 낮추자"며 임의로 되돌리지 말 것.
- **YOLO 모델은 추후 자체 파인튜닝 버전으로 교체 예정**이다. 지금은
  Ultralytics 기본 제공 `yolov8n-pose.pt`를 쓰고 있지만, `detection/`
  패키지(`PoseDetector` 인터페이스)로 이미 분리해둬서 `main.py`의
  `MODEL_PATH`만 바꾸면 교체된다 — `Tracker.update(nose_x, nose_y)`로
  이어지는 흐름 자체는 모델이 바뀌어도 그대로 유지된다.
- **제로잉/CAN ID 변경이 전원 재인가 후 사라지던 버그 수정 완료**:
  `set_zero_position()`/`change_motor_param()` 직후 `save_motor_param()`
  (플래시 저장) 호출이 빠져 있었던 걸 `test/actuator_conn_test.py`,
  `test/zero_actuator.py`에서 고쳤다.

## 구성

```
main.py                    실행 진입점 — 카메라 + 검출 + 트래킹 통합 루프
motor_start.sh             지정한 픽셀 좌표로 Yaw/Pitch 둘 다 이동 후 유지 (카메라 없이)
actuator_test.sh           액추에이터 테스트 메뉴(포트지정/신호조회/위치신호/제로잉) 셸 스크립트
detection/
  base.py                  PoseDetector 인터페이스, PersonPose 데이터 구조
  yolo_pose.py              YOLOv8 pose 기반 PoseDetector 구현체
motor/
  dm4310_tracker.py        Damiao DM-J4310-2EC 트래커 (Tracker 클래스)
  DM_CAN.py                 Damiao 저수준 CAN 드라이버 (벤더링된 외부 라이브러리)
mp3_player.py               MP3 재생기 (pygame 기반)
audio/                      상황별 안내 음성 파일
legacy/
  face_tracker_xl430_06_14.py   이전 세대 Dynamixel XL430 트래커 (참고용, 현재 미사용)
  local.py                  라즈베리파이 영상 스트림 웹 서버 (별도 용도)
test/
  face_tracker_test.py      Haar cascade 기반 얼굴 방향 테스트 (YOLO 없이)
  motor_test.py              모터 단독 테스트 (카메라 없이 키보드로 좌표 입력)
  actuator_conn_test.py      액추에이터 1개 연결/신호조회/위치신호(MIT)/제로잉
  zero_actuator.py           DM-J4310-2EC 영점(0도) 설정 유틸리티
  motor_goto.py              motor_start.sh 가 호출하는 실제 구현 (Tracker 두 축 모두 구동)
  motor_manual_control.py    Yaw/Pitch 방향키(a/d, w/s) 수동 위치 제어 + 중앙 복귀
```

## motor_start.sh — 픽셀 좌표로 짐벌 이동

카메라/YOLO 없이, 픽셀 좌표만 직접 줘서 실제 운영 `Tracker`(Yaw+Pitch 둘 다)를
단독으로 구동해본다. `Tracker.update(x, y)`를 한 번 호출하면 내부 1000Hz
제어 스레드가 그 목표를 계속 유지하므로, 이 스크립트는 상태를 주기적으로
출력하며 대기하다가 **Ctrl+C로 종료**(모터 비활성화 + 포트 닫힘)한다.

```bash
./motor_start.sh 1200 900   # 그 좌표로 이동 후 유지
./motor_start.sh            # 대화형으로 좌표 입력받음
```

좌표계는 `motor/dm4310_tracker.py`의 `FRAME_W`/`FRAME_H`(기본 2592×1944,
카메라 해상도) 기준 — 좌측 상단이 `(0,0)`.

## test/motor_manual_control.py — 방향키 수동 제어

Yaw(CAN 1)/Pitch(CAN 2)를 방향키로 직접 조금씩 밀어보는 테스트 도구.
`motor/dm4310_tracker.py`와 같은 연결 정보·게인(KP/KD)을 그대로 쓴다.

```bash
uv run python test/motor_manual_control.py
```

실행하면 메뉴를 고르기 전에 먼저 **두 축을 중앙(0°, 0°)으로 램프
이동**(`GO_CENTER_DURATION`=1.5초 재사용)시킨 뒤 메뉴를 보여준다.

```
1. START (아직 구현되지 않음)
2. 방향키로 위치 제어 (모터 각도 직접 조그)
3. 가상 코 좌표 테스트 (wasd로 코 좌표 이동 → 실제 Tracker)
0. 종료
```

**1번(START)**은 자리만 잡아둔 상태다 — 나중에 `main.py`의 자동 추적
(카메라+YOLO+`Tracker`)과 연결될 예정이라고 이미 정해져 있다.

**2번**: `a`/`d` = yaw +/-, `w`/`s` = pitch +/-, `q` = 메뉴로 복귀. 모터
각도를 **직접** 명령하는 raw 조그 모드. 한 번 입력될 때마다 `STEP_DEG`
(기본 0.3°, 보수적으로 작게 잡음)만큼 누적되고, 키를 꾹 누르고 있으면
터미널의 자동반복 입력으로 계속 쌓인다.

안전 가동범위(도, 나중에 상수만 고치면 됨):
- Yaw: `YAW_MIN_DEG`/`YAW_MAX_DEG` = -175 / 175
- Pitch: `PITCH_MIN_DEG`/`PITCH_MAX_DEG` = -30 / 180

**3번**: 2번과 달리 모터 각도를 직접 보내지 않고, **"코 좌표"**(노란 점,
`main.py`가 YOLO 검출로 넘겨주는 값과 같은 역할)를 `a`/`d`/`w`/`s`로
움직여 실제 운영 `Tracker.update(x, y)`에 그대로 흘려보낸다 — 픽셀→각도
환산(`RAD_PER_PIXEL_X/Y`), 부호(`SIGN_YAW`/`SIGN_PITCH`), 가동범위
(`YAW_RANGE_RAD`/`PITCH_RANGE_RAD`) 등 `motor/dm4310_tracker.py`의 실제
로직 전체를 그대로 검증할 수 있다. 코 좌표는 항상 프레임 중앙(Tracker
기준 0°, 0°)에서 시작하며, OpenCV 창에 초록 중앙 십자선과 노란 점으로
현재 위치를 보여준다. `q` 또는 `ESC`로 메뉴 복귀.

2번과 3번은 같은 시리얼 포트를 써서 동시에 열 수 없으므로, 3번에
들어가면 2번용 연결을 잠깐 닫았다가 메뉴로 돌아오면 다시 연다
(화면에 "재연결 중..." 표시됨).

### main.py — 실행 흐름

1. Arducam B0538C(OG05B1B, 카메라 인덱스 4)에서 2592×1944 @ 25fps로 영상을 받는다.
   UVC 표준 YUY2로 출력하는 카메라라 별도 libv4l2 변환 없이 OpenCV가 바로 읽는다.
2. `detection.yolo_pose.YoloPoseDetector`가 프레임마다(카메라 fps 한도인 ~25fps)
   사람의 관절을 검출해 코 좌표, 손 든 상태(arm_up), 몸 박스, 얼굴 박스를 담은
   `PersonPose` 목록을 반환한다.
3. 손을 `ARM_UP_THRESHOLD`(3초) 이상 든 사람 중 몸 박스가 가장 큰 사람을
   트래킹 대상으로 확정한다. 손이 `ARM_DOWN_GRACE`(10프레임) 넘게 내려가면
   추적을 해제한다.
4. 확정된 사람의 코 좌표를 얼굴 박스로 한 번 더 필터링(노이즈 좌표 제거)한 뒤
   `Tracker.update(x, y)`를 호출한다. 실제 모터 구동은 `motor/dm4310_tracker.py`가
   전담한다.
5. Yaw 모터의 실제 각도와 검출 박스 크기로 "구간 이탈" 여부를 판정해 방향별
   안내 음성을 재생한다.
6. 화면에는 추적 상태, YOLO 처리 시간, 구간 이탈 상태 등을 오버레이로 표시한다.
   `w`(재추적), `e`(중앙 복귀), `s`(샘플 음성), `+`/`-`/`0`(볼륨), `o`/`p`(안내
   음성), `q`(종료) 키를 지원한다.

`--motor` 옵션 없이 실행하면 모터 없이 화면 표시만 한다.

### detection/ — 검출기 추상화

- `base.py`의 `PoseDetector`는 `detect(frame) -> List[PersonPose]` 하나만
  구현하면 되는 인터페이스다. `PersonPose`는 `nose`, `arm_up`, `box`, `face_box`를
  담은 데이터 클래스.
- `yolo_pose.py`의 `YoloPoseDetector`가 기본 구현체로, Ultralytics YOLOv8 pose
  모델(`yolov8n-pose.pt`)을 로드해 COCO 17-keypoint 기준으로 코 좌표·팔 든
  상태·얼굴 박스를 계산한다. GPU(cuda:0)가 있으면 자동으로 쓰고 없으면 CPU로
  폴백한다.
- 다른(예: 파인튜닝된) YOLO pose 모델로 바꾸려면 `main.py`의 `MODEL_PATH`만
  바꾸면 되고, 키포인트 레이아웃이 다른 모델을 쓰려면 `PoseDetector`를 새로
  구현하면 된다 — `main.py`를 포함한 상위 코드는 그대로 재사용된다.

### motor/dm4310_tracker.py — Damiao DM-J4310-2EC 트래커

- Yaw/Pitch 두 개의 DM4310 모터를 Damiao 전용 USB-CAN 동글(시리얼, 921600 baud)로
  제어한다. 저수준 CAN 프레이밍은 `motor/DM_CAN.py`가 담당한다.
- 제어 모드는 MIT 모드(임피던스 제어). 모터 내부 MCU가
  `torque = Kp*(q_des - q) + Kd*(dq_des - dq) + tau_ff` 식으로 자체 전류 루프를
  항상 돌리고 있고, 호스트는 목표 위치(`q_des`)만 계속 갱신해서 보낸다 —
  실질적으로 위치 제어를, 부드러움은 MIT 임피던스가 물리적으로 담당하는 구조.
- 매 CAN 사이클(`controlMIT()` 호출 직후)마다 모터가 응답으로 돌려주는 실제
  위치/속도/토크를 `self._current`로 읽어와 다음 목표 계산의 기준(current
  value)으로 삼는 클로즈드 루프 구조다. `update(nose_x, nose_y)`는 검출
  루프(카메라 fps 한도인 ~25fps)에서 호출되며, 픽셀 오차를 각도 스텝으로
  환산해 "실제 현재 위치 + 스텝"을 목표값으로 설정한다.
- 별도 스레드(`_control_thread`)가 목표 주파수 1000Hz(`CONTROL_FREQ`)로
  목표값을 MIT 프레임에 실어 CAN에 계속 스트리밍한다. 실측 주파수는
  `get_actual_freq()`로 확인 가능.
- `go_center()`는 현재 실측 위치에서 0 rad(중앙)까지 선형 램프로 서서히
  복귀시킨다.
- 가로(Yaw)/세로(Pitch) 픽셀→각도 환산계수 `RAD_PER_PIXEL_X`/`_Y`는 Arducam
  B0538C 렌즈 스펙(HFOV 82°/VFOV 66°, EFL 3.6mm, 픽셀피치 2.2µm)으로 이론
  유도한 값이다. 레티리니어 렌즈는 광축에서 x만큼 떨어진 점의 각도가
  `atan(x/f)`이고, 코를 화면 중앙 근처로 유지하는 제어 특성상 x가 작을 때
  `atan(x/f) ≈ x/f`로 선형 근사가 잘 맞으므로 화면 전체에 각도가 픽셀에 선형
  비례한다고 가정해 `RAD_PER_PIXEL = radians(FOV) / 해상도`로 계산했다
  (`motor/dm4310_tracker.py`의 상수 정의부 주석에 유도 과정과 초점거리·
  픽셀피치 기반 교차검증이 있다). 단, 짐벌 기어비(모터-카메라 간 감속비)는
  반영돼 있지 않으므로 1:1이 아니면 그 비율만큼 보정이 필요하고, 실측 후
  미세조정도 필요하다. 가동범위(`YAW_RANGE_RAD`/`PITCH_RANGE_RAD`), 임피던스
  게인(`KP`/`KD`)은 실제 기구 한계·응답 특성에 맞춰 튜닝해야 하는 값이다.

### mp3_player.py, audio/

상황별 안내 음성(추적 시작, 좌/우/구간 이탈, 신호 안내, 완주 등)을 재생하는
모듈과 음성 파일들이다. 이번 작업에서는 손대지 않았다 — 기존 그대로다.

### legacy/

- `face_tracker_xl430_06_14.py`: 이전에 쓰던 Dynamixel XL430 기반 2축 트래커.
  더 이상 `main.py`에서 쓰이지 않고 참고용으로만 남아 있다.
- `local.py`: 라즈베리파이 영상 스트림 기반 웹 서버로, 메인 추적 파이프라인과는
  별개의 용도다.

### test/

- `face_tracker_test.py`: OpenCV Haar cascade로 얼굴 박스만 찾아 방향(LEFT/
  RIGHT/UP/DOWN)을 표시하는 가벼운 테스트. YOLO 없이 카메라 프레임 좌표 계산과
  `Tracker` 연동만 확인하고 싶을 때 쓴다.
- `motor_test.py`: 카메라 없이 키보드로 가상 코 좌표를 입력해 `Tracker.update()`를
  직접 호출하는 모터 단독 테스트. `--motor` 없이 실행하면 어떤 좌표가 전송될지만
  출력하는 DRY_RUN이다. `--motor`로 실제 모터를 구동하면 매 명령 후, 그리고
  자동 순환(`a`) 각 방향 정착 후 실측 제어 주파수와 Yaw/Pitch 각 축의 실제
  위치·속도·토크(`get_actual_freq()`/`get_state()`)를 출력해 실제로 목표에
  도달했는지 확인할 수 있다.
- `zero_actuator.py`: DM-J4310-2EC의 영점(0 rad)을 설정하는 유틸리티. 모터를
  활성화(enable)하지 않은 상태로 열기 때문에 손으로 자유롭게 돌릴 수 있다 —
  짐벌을 원하는 정면/중앙 자세로 손으로 맞춘 뒤 Enter를 누르면 그 자리를 모터
  내부 비휘발성 메모리에 0도로 저장한다(전원을 꺼도 유지됨). 조립 시, 또는
  출력 디스크를 다시 조립했을 때 한 번만 실행하면 된다.
- `actuator_conn_test.py`: 새 액추에이터 1개를 Yaw/Pitch 역할에 배선하기 전,
  ①CAN ID 1~N번을 순서대로 찔러보는 스캔으로 포트/ID를 찾고 원하면 그 자리에서
  새 ID로 바꿔 모터 플래시에 저장 ②토크 없이 상태 조회만 반복하는 신호 조회
  ③짧게 활성화해 작은 각도(기본 0.1rad)만 안전하게 왕복시키는 위치 신호(MIT
  모드) 테스트 ④토크 없이 손으로 0도 위치를 맞춘 뒤 영점을 저장하는 제로잉
  ⑤지정한 각도로 이동해서 그 자리를 계속 유지하는 위치 이동(`goto`)까지
  확인하는 스크립트. `--step id`/`ping`/`mit`/`zero`/`goto`로 한 단계만
  실행할 수 있고(보통 `actuator_test.sh` 메뉴가 이 방식으로 호출함), ID
  변경은 `motor/DM_CAN.py`의 `read_motor_param`/`change_motor_param`/
  `save_motor_param`을 사용 — Damiao 자체 Windows 설정 프로그램 없이도 CAN
  ID/Master ID 변경이 가능하다.

## actuator_test.sh — 액추에이터 테스트 메뉴

프로젝트 루트의 `actuator_test.sh`는 `actuator_conn_test.py`를 번호로 골라
실행하는 대화형 메뉴다. CLI 옵션 없이 그냥 실행한 뒤 번호만 입력하면 된다.

```bash
./actuator_test.sh
```

```
1. CAN ID 찾기/할당 (스캔)
2. CAN ID 설정 (직접 입력)
3. 신호 조회 (통신 생존 확인)
4. 위치 신호 (MIT 모드 테스트)
5. 위치 이동 (특정 각도로 이동 후 유지)
6. 제로잉 (영점 설정)
7. 게인 설정 (Kp/Kd/이동각/유지시간, 이번 실행만 적용)
0. 종료
```

**포트(시리얼 장치 경로)는 이 메뉴에서 따로 설정하지 않는다.**
`test/actuator_conn_test.py`가 `motor/dm4310_tracker.py`의 `DEVICENAME`
상수를 그대로 기본값으로 쓰기 때문에, 실제 포트가 바뀌면 **`motor/
dm4310_tracker.py`의 `DEVICENAME` 한 곳만 고치면** 이 메뉴와 `main.py`
운영 코드 모두에 일괄 적용된다. 메뉴 상단에는 지금 쓰이는 포트 값을
그 상수에서 그대로 읽어와 표시만 해준다(수정은 안 됨, 확인용).

**1번은 CAN ID를 몰라도 된다** — 1~10번을 순서대로 찔러봐서(스캔) 응답하는
ID를 찾고(모터 1대만 연결돼 있어야 함), 찾으면 원하는 새 ID로 바꿀지
물어본다. 최종적으로 응답하는 ID(바꿨다면 새 ID, 안 바꿨다면 원래 ID)가
자동으로 CAN ID로 지정된다.

**2번은 CAN ID를 이미 알 때** 스캔 없이 바로 타이핑해서 지정하는 메뉴다
(예: Yaw=0x01, Pitch=0x02 관례를 이미 안다면 매번 스캔할 필요 없음).

**3~6번(신호 조회/위치 이동/위치 신호/제로잉)은 1번 또는 2번으로 CAN ID가
지정된 뒤에만 실행된다** — 안 되어 있으면 안내하고 메뉴로 돌아간다.

Yaw/Pitch 양쪽을 다 점검하려면 1번 또는 2번을 다시 선택해서 CAN ID만
바꿔주면 된다.

**5번(위치 이동)**은 4번(위치 신호)과 달리 **왕복하지 않는다** — 입력한
목표 각도(도)로 이동한 뒤 그 자리를 계속 유지한다(내부적으로 `controlMIT()`을
~100Hz로 계속 스트리밍). 멈추려면 **Ctrl+C**를 눌러야 하고, 그러면 모터가
비활성화(토크 OFF)되고 포트가 닫힌다. "특정 위치로 보내고 그대로 두고
싶다"는 용도에 맞춘 메뉴다.

**7번(게인 설정)**은 4·5번에 쓸 `Kp`/`Kd`/이동각(`delta`, 도)/
목표 유지시간(`hold`, 초)을 이 스크립트를 실행하는 "동안만" 바꾸는
메뉴다. 여기서 바꾼 값은 `test/actuator_conn_test.py` 맨 위의
`DEFAULT_KP`/`DEFAULT_KD`/`DEFAULT_DELTA_RAD`/`DEFAULT_HOLD_SEC`
**상수 자체를 바꾸지 않는다** — 스크립트를 종료했다 다시 켜면 다시 그
파일의 기본값(8.0 / 0.6 / 0.1rad / 1.0s)으로 시작한다. "항상 이 값으로
시작하고 싶다"면 `actuator_conn_test.py` 상단의 해당 상수를 직접
수정해야 한다(각 상수 위에 의미·단위·안전 범위 주석이 달려 있다).

## Setup

```bash
# 의존성 설치 및 가상환경 동기화
uv sync
```

하드웨어 준비 (실제 모터 투입 전 필수 확인):
- 카메라: Arducam B0538C(OG05B1B, 5MP 흑백 글로벌 셔터, USB 3.0). **아직
  미연결 상태** — 연결 전까지는 `main.py`의 `CAMERA_IDX = 4`가 실제로
  맞을지 알 수 없다. 연결 후 반드시 `ls /dev/video*` 등으로 실제 인덱스를
  확인하고 다르면 수정할 것. USB 3.0 연결 시 2592×1944 @ 25fps가 스펙상
  최대치(USB 2.0은 5fps로 급감)
- Damiao 전용 USB-CAN 동글 — 이 프로젝트 기준 `/dev/ttyACM0`으로 잡힘(동글
  칩셋에 따라 `/dev/ttyUSB0`로 잡힐 수도 있음, `motor/dm4310_tracker.py`의
  `DEVICENAME`에 실측값 반영돼 있음). 다른 PC에 연결하면 `ls /dev/ttyUSB*
  /dev/ttyACM*`로 재확인할 것
- Yaw=CAN ID `0x01`, Pitch=CAN ID `0x02`(MasterID `0x11`/`0x12`)로 확정·
  설정됨 — `./actuator_test.sh`의 "CAN ID 찾기/할당"으로 Damiao 공식
  소프트웨어 없이 CAN 통신만으로 확인/변경 가능. MIT 모드는 `Tracker`
  초기화 시 자동으로 전환하므로 별도 사전 설정 불필요
- `motor/dm4310_tracker.py`의 `KP`/`KD`(임피던스 게인)는 **실측을 거쳐
  500.0 / 5.0(DM4310 하드웨어 허용 최댓값)으로 확정**돼 있다 — 의도적으로
  검증된 값이니 "안전하게 낮추자"며 임의로 바꾸지 말 것. `YAW_RANGE_RAD`/
  `PITCH_RANGE_RAD`(가동범위)는 실제 기구 한계에 맞춰 튜닝할 것. 픽셀→각도
  환산계수 `RAD_PER_PIXEL_X`/`_Y`는 렌즈 스펙으로 이론 유도돼 있지만(위
  "motor/dm4310_tracker.py" 설명 참고), 짐벌 기어비가 반영돼 있지 않으니
  실제 기어비·실측 결과에 맞춰 마지막으로 미세조정할 것. `SIGN_YAW`/
  `SIGN_PITCH`는 실측 완료(둘 다 `-1`) — 다른 기구/배선으로 바뀌면 재검증
  필요

## Run

```bash
# 카메라 + YOLO 추적 실행
uv run main.py

# 카메라 + YOLO 추적 + Damiao DM-J4310-2EC 모터 제어
uv run main.py --motor

# 라즈베리파이 영상 스트림 기반 웹 서버 실행
uv run python legacy/local.py
```

## Keys

```bash
# 실행 화면 종료
q

# sample 음성 재생
s
```

## Test

```bash
# 얼굴 검출/방향 표시 테스트
uv run python test/face_tracker_test.py

# 얼굴 검출/방향 표시 + Damiao DM-J4310-2EC 모터 제어 테스트
uv run python test/face_tracker_test.py --motor

# 모터 좌표 입력 테스트(DRY_RUN)
uv run python test/motor_test.py

# 실제 Damiao DM-J4310-2EC 모터 좌표 입력 테스트
uv run python test/motor_test.py --motor

# DM-J4310-2EC 영점(0도) 설정 (조립 시 1회)
uv run python test/zero_actuator.py

# 액추에이터 1개 CAN ID 스캔/신호조회/위치신호/제로잉 메뉴
./actuator_test.sh

# 지정한 픽셀 좌표로 Yaw+Pitch 이동 후 유지 (카메라 없이)
./motor_start.sh 1200 900

# Yaw/Pitch 방향키 수동 제어 + 가상 코 좌표 테스트 메뉴
uv run python test/motor_manual_control.py
```
