# ADAS Project

카메라로 사람을 찾아 YOLOv8 pose로 손 든 사람을 확정한 뒤, 그 사람의 코 좌표를
따라 Damiao DM4310-2EC 2축(Yaw/Pitch) 짐벌이 CAN 통신으로 고속(최대 1000Hz)
추적하는 프로젝트. 트래킹 확정 및 구간 이탈 시 상황에 맞는 안내 음성도 재생한다.

## 구성

```
main.py                    실행 진입점 — 카메라 + 검출 + 트래킹 통합 루프
detection/
  base.py                  PoseDetector 인터페이스, PersonPose 데이터 구조
  yolo_pose.py              YOLOv8 pose 기반 PoseDetector 구현체
motor/
  dm4310_tracker.py        Damiao DM4310-2EC 트래커 (Tracker 클래스)
  DM_CAN.py                 Damiao 저수준 CAN 드라이버 (벤더링된 외부 라이브러리)
mp3_player.py               MP3 재생기 (pygame 기반)
audio/                      상황별 안내 음성 파일
legacy/
  face_tracker_xl430_06_14.py   이전 세대 Dynamixel XL430 트래커 (참고용, 현재 미사용)
  local.py                  라즈베리파이 영상 스트림 웹 서버 (별도 용도)
test/
  face_tracker_test.py      Haar cascade 기반 얼굴 방향 테스트 (YOLO 없이)
  motor_test.py              모터 단독 테스트 (카메라 없이 키보드로 좌표 입력)
```

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

### motor/dm4310_tracker.py — Damiao DM4310-2EC 트래커

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

## Setup

```bash
# 의존성 설치 및 가상환경 동기화
uv sync
```

하드웨어 준비 (실제 모터 투입 전 필수 확인):
- 카메라: Arducam B0538C(OG05B1B, 5MP 흑백 글로벌 셔터, USB 3.0). 연결 후 실제
  `/dev/video*` 인덱스를 확인해 `main.py`의 `CAMERA_IDX`(기본값 4)를 맞출 것.
  USB 3.0 연결 시 2592×1944 @ 25fps가 스펙상 최대치(USB 2.0은 5fps로 급감)
- Damiao 전용 USB-CAN 동글을 `/dev/ttyUSB0`(또는 `motor/dm4310_tracker.py`의
  `DEVICENAME`)로 연결
- Damiao 튜닝 소프트웨어로 Yaw=CAN ID `0x01`, Pitch=CAN ID `0x02`,
  MasterID `0x11`/`0x12`로 사전 설정, 두 모터 모두 MIT 모드 활성화
- `motor/dm4310_tracker.py`의 `YAW_RANGE_RAD`/`PITCH_RANGE_RAD`(가동범위),
  `KP`/`KD`(임피던스 게인)를 실제 기구에 맞게 튜닝. `KP`/`KD`는 낮은 값에서
  시작해 진동 없이 안정적으로 따라올 때까지 올릴 것. 픽셀→각도 환산계수
  `RAD_PER_PIXEL_X`/`_Y`는 렌즈 스펙으로 이론 유도돼 있지만(위 "motor/
  dm4310_tracker.py" 설명 참고), 짐벌 기어비가 반영돼 있지 않으니 실제
  기어비·실측 결과에 맞춰 마지막으로 미세조정할 것

## Run

```bash
# 카메라 + YOLO 추적 실행
uv run main.py

# 카메라 + YOLO 추적 + Damiao DM4310-2EC 모터 제어
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

# 얼굴 검출/방향 표시 + Damiao DM4310-2EC 모터 제어 테스트
uv run python test/face_tracker_test.py --motor

# 모터 좌표 입력 테스트(DRY_RUN)
uv run python test/motor_test.py

# 실제 Damiao DM4310-2EC 모터 좌표 입력 테스트
uv run python test/motor_test.py --motor
```
