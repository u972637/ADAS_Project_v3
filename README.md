# ADAS Project

이 저장소는 **같은 Damiao DM-J4310-2EC 2축(Yaw/Pitch) CAN 짐벌 + Arducam
B0538C 카메라 하드웨어를 공유하는 애플리케이션 두 개**를 담고 있다.

- **`main.py` — 야생동물 음향 퇴치**: Arducam 영상에서 멧돼지·고라니를
  탐지하고, 선택한 동물의 bbox 중심으로 카메라를 조준하며, YOLO가 판별한
  종에 맞는 WAV를 재생한다.
- **`main_facetrack.py`(구 `main.py`) — 사람 얼굴 추적**: YOLOv8 pose로
  손 든 사람을 확정한 뒤, 그 사람의 코 좌표를 따라 짐벌이 추적하고 상황별
  안내 음성을 재생한다.

모터(`motor/`)와 모터 점검 도구(`tools/`)는 두 애플리케이션이 공유한다.

## 현재 진행 상황 (최근 작업 기록)

이 저장소는 원래 **얼굴 추적 짐벌**(이 세션 이전의 `main.py`, 지금의
`main_facetrack.py`) 프로젝트였는데, 협업자 kweonminsung이 같은 모터·카메라
하드웨어를 쓰는 **야생동물 음향 퇴치**(`feat/wildlife-audio-deterrent`)
브랜치를 `origin/main`에 먼저 merge해 올렸다. `joonhyoung` 브랜치에서
`origin/main`을 pull해보니 git이 `main.py`를 포함한 상당 부분을 line-based
diff 한계로 "충돌 없음"으로 잘못 판단해 조용히 덮어쓸 뻔한 걸 발견했고,
그래서 매 충돌/결정마다 사용자에게 확인을 받으며 아래와 같이 정리했다.

- **`main.py`/`main_facetrack.py` 분리**: `origin/main`의 야생동물 코드가
  `main.py`를 그대로 차지하고, 기존 얼굴 추적 코드는 `main_facetrack.py`로
  옮겼다(파일 내용은 그대로, 이름만 바뀜).
- **공용 스크립트 폴더 통일**: 기존에 `test/`에 있던 모터 점검 스크립트
  (`actuator_conn_test.py`, `camera_test.py`, `motor_goto.py`,
  `motor_manual_control.py`, `zero_actuator.py`)를 협업자 쪽 컨벤션인
  `tools/`로 통일했다. 얼굴 추적 전용 테스트(`face_tracker_test.py`,
  `motor_test.py`)는 `test/`에 그대로 남아 있다.
- **의존성/경로 복구**: merge가 떨어뜨린 `flask` 의존성(`legacy/local.py`가
  사용)을 `pyproject.toml`에 복구했고, YOLO pose 모델(`yolov8n-pose.pt`)을
  `assets/models/`로 옮겨 야생동물 쪽 가중치(`weights.pt`)와 한 곳에
  모았다. `actuator_test.sh`/`camera_moter_test.sh`의 깨진 `test/` 경로
  참조도 `tools/`로 고쳤다.

**`camera_moter_test.sh`(`tools/motor_manual_control.py`)를 모터+카메라
통합 테스트 메뉴로 재구성**했다 — 원래는 "START(미구현)"/"방향키 조그"
두 항목뿐이었는데, 지금은:

1. **동물 추적 + 종별 음향 재생** — `main.py`가 쓰는 `wildlife.py`/
   `camera.py`/`settings.py`를 그대로 재사용해, 이 메뉴 안에서도 멧돼지/
   고라니 탐지→모터 조준→WAV 재생 전체를 확인할 수 있다.
2. **YOLO 코 좌표 추적** — `main_facetrack.py`와 같은 `YoloPoseDetector`로
   코 좌표를 인식하되, 실제 모터 추적은 손을 `ARM_UP_THRESHOLD`(2초) 이상
   들어야 시작된다("락온"). 락온되는 그 순간 `audio/track_start2.mp3`를
   1회 재생한다.
3. **방향키 수동 조그** — 기존 raw 각도 조그 기능 그대로.

1·2·3번 모두 `q`/`ESC`로 **정상** 종료하면 공통으로 "숙인위치"(yaw=0°,
pitch=-4°)로 1초에 걸쳐 복귀한다(`motor/dm4310_tracker.py`에 범용
`Tracker.go_to()`를 추가해 구현 — 기존 `go_center()`는 이걸 호출하는
(0,0) 특수 케이스로 재구현됨). `Ctrl+C` 강제중지는 의도적으로 이 복귀
없이 바로 멈춘다.

**임피던스 게인을 Yaw/Pitch 축별로 분리**했다(`KP_YAW`/`KD_YAW`/
`KP_PITCH`/`KD_PITCH`, 기존엔 공용 `KP`/`KD` 하나였음). MIT 제어식
(`torque = Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff`)은 I(적분)항이 없는
순수 PD 제어라, Pitch축처럼 자중으로 인한 일정한 외란 토크가 걸리는
경우 `q_actual = q_des - τ_dist/Kp` 형태의 **영구적인 steady-state
오차**가 생긴다는 걸 확인했고, 이를 `TAU_FF_PITCH`(피드포워드 중력보상
토크, Yaw는 항상 0 고정)로 보정하는 중이다 — 보정값은
"`τ_dist ≈ KP_PITCH × 정상상태 오차(rad)`" 식으로 실측·역산해서 시작점을
잡고, 실기로 미세조정하고 있다.

**현재 열려 있는 항목(튜닝 진행 중, 계속 바뀔 수 있음)**:
- `HFOV_DEG`/`VFOV_DEG`(픽셀→각도 환산 기준각)는 렌즈 스펙 기반 이론값
  (82°/66°)에서 시작했지만, 실기 테스트로 계속 실측 보정 중이라 현재 값은
  그보다 훨씬 작다 — 이론 유도 주석과 실제 상수값이 달라도 정상이니,
  "이론값으로 되돌리자"며 임의로 고치지 말 것.
- `TAU_FF_PITCH`는 한 지점(기본 자세 30°)에서 역산한 초기값이라, 가동범위
  전체에서는 중력 토크가 `cos(θ)`에 비례해 변하므로 각도에 따라 오차가
  남을 수 있다. 완전한 보정을 원하면 상수가 아니라 각도의 함수로 바꿔야
  한다.
- `SIGN_YAW`/`SIGN_PITCH`는 이론 유도(`-1`)와 실측 확정값(`+1`)이 반대라,
  코드 위 주석에 그 불일치와 이유를 남겨뒀다 — 다른 기구/배선으로 바뀌면
  재검증 필요.

## 공용 하드웨어 — 현재 확인된 값 (알아둘 것)

- **카메라 인덱스**: Arducam B0538C는 `/dev/video4` — 실측 확인됨
  (`tools/camera_test.py` / `actuator_test.sh` 8번으로 검증).
- **액추에이터 CAN ID**: Yaw = CAN ID `0x01`, Pitch = CAN ID `0x02`
  (`motor/dm4310_tracker.py`).
- **Yaw/Pitch 부호(SIGN)**: 이론 유도로는 `-1`이 수렴 방향이지만, 실기로
  두 축을 번갈아 테스트한 결과 `SIGN_YAW = SIGN_PITCH = +1`일 때 실제로
  대상을 따라가며 중앙으로 수렴함을 확인했다 — 실측으로 검증·확정된
  값이니 임의로 바꾸지 말 것.
- **임피던스 게인(축별 분리)**: `KP_YAW`/`KD_YAW`, `KP_PITCH`/`KD_PITCH`로
  Yaw/Pitch 게인이 서로 독립이다(하나를 바꿔도 다른 축에 영향 없음).
  `TAU_FF_PITCH`(Pitch 전용 중력보상 피드포워드)로 steady-state 오차를
  보정 중 — 현재 값들은 실기로 계속 튜닝되고 있는 값이니 "임의로 되돌리지
  말 것"은 동일하게 적용된다. 정확한 현재 값은 `motor/dm4310_tracker.py`를
  직접 확인할 것(이 문서에 못 박아두면 다음 튜닝 때마다 또 고쳐야 해서
  일부러 안 적는다).
- **제로잉/CAN ID 변경 영속성**: `set_zero_position()`/
  `change_motor_param()` 직후 `save_motor_param()`(플래시 저장) 호출이
  빠져 전원 재인가 시 초기화되던 버그를 `tools/actuator_conn_test.py`,
  `tools/zero_actuator.py`에서 고쳤다.
- **모터 고장 코드 진단**: 피드백 프레임의 ERR 니블(과전압/저전압/
  과전류/MOS과열/코일과열/통신끊김/과부하)을 `motor/DM_CAN.py`가 파싱해
  `Motor.getError()`로 조회 가능 — `tools/actuator_conn_test.py`가
  enable 직후·이동 후·신호조회 중 자동으로 점검해 경고한다.

## 구성

```
main.py                     야생동물 탐지+음향퇴치 실행 진입점
main_facetrack.py           사람 얼굴 추적 실행 진입점 (구 main.py)
motor_start.sh              지정한 픽셀 좌표로 Yaw/Pitch 이동 후 유지 (공용)
camera_moter_test.sh        동물 추적+음향, YOLO 코 좌표 추적(손들기로 시작), 방향키 모터 조그 메뉴
actuator_test.sh            액추에이터 테스트 메뉴 (공용)

# 야생동물 탐지 (main.py)
settings.py                 모델·카메라·모터·음향 설정 상수
camera.py                   Arducam YUY2 캡처, 최신 프레임 전달, 재연결
wildlife.py                 YOLO 결과·대상 선택·종 분류·재생 정책과 모터 어댑터
audio.py                    WAV 목록 검증·무작위 선택·재생
assets/models/weights.pt    로컬 동물 탐지 YOLO 가중치
assets/audio/               멧돼지·고라니 WAV와 출처 매니페스트

# 얼굴 추적 (main_facetrack.py)
detection/
  base.py                   PoseDetector 인터페이스, PersonPose 데이터 구조
  yolo_pose.py               YOLOv8 pose 기반 PoseDetector 구현체
assets/models/yolov8n-pose.pt  YOLOv8 pose 가중치 (얼굴 추적용, 동물 탐지와 별개)
mp3_player.py                MP3 재생기 (pygame 기반)
audio/                       상황별 안내 음성 파일 (mp3)
legacy/
  face_tracker_xl430_06_14.py   이전 세대 Dynamixel XL430 트래커 (참고용)
  local.py                   라즈베리파이 영상 스트림 웹 서버 (별도 용도)
test/
  face_tracker_test.py       Haar cascade 기반 얼굴 방향 테스트 (YOLO 없이)
  motor_test.py               모터 단독 테스트 (카메라 없이 키보드로 좌표 입력)

# 공용 — 모터/CAN
motor/
  dm4310_tracker.py          Damiao DM-J4310-2EC 트래커 (Tracker 클래스)
  DM_CAN.py                   Damiao 저수준 CAN 드라이버 (벤더링된 외부 라이브러리)
tools/
  zero_actuator.py            모터 영점 설정
  actuator_conn_test.py       액추에이터 1개 연결/신호조회/위치신호(MIT)/제로잉
  motor_goto.py                motor_start.sh 가 호출하는 실제 구현 (두 축 모두 구동)
  motor_manual_control.py     동물 추적+음향, YOLO 코 좌표 추적, Yaw/Pitch 방향키 수동 위치 제어
  camera_test.py               카메라 단독 테스트 (모터/YOLO 없이 미리보기만)
  play_audio.py                클래스별 음성 재생만 확인 (야생동물용)
```

## 야생동물 탐지 (`main.py`)

```text
카메라 → 동물 YOLO → 대상 선택 ┬→ DM4310 카메라 조준 (ENABLE_MOTOR)
                              └→ YOLO 클래스 → 재생 정책 → 무작위 WAV
```

기본으로 `assets/models/weights.pt`의 가중치를 사용한다. 다른 모델을 쓰려면
`settings.py`의 `MODEL_PATH`를 수정한다. `CLASS_TO_AUDIO`는 실제 YOLO 클래스
ID를 `boar` 또는 `water_deer` 음원 그룹에 연결한다. `EXPECTED_MODEL_NAMES`에는
해당 ID의 실제 `model.names` 값을 적는다. 이름이 다르면 카메라·모터를 열기
전에 실행을 중단한다. 현재 로컬 모델의 클래스와 설정은 다음과 같다.

| 클래스 ID | 모델 이름 | 음원 그룹 |
| --- | --- | --- |
| 0 | `boar` | `boar` (멧돼지용) |
| 1 | `deer` | `water_deer` (고라니용) |
| 2 | `torso` | 조준·재생 대상에서 제외 |

```bash
uv run main.py
```

처음 점검할 때 `settings.py`의 `DRY_RUN = True`로 두면 음원 경로만 출력한다.
소리를 내려면 `DRY_RUN = False`, 모터를 사용하려면 `ENABLE_MOTOR = True`로
설정한다. 화면에서 `q`는 종료, `e`는 모터 중앙 복귀다. 화면 없이 실행하려면
`HEADLESS = True`로 바꾸고 Ctrl+C로 종료한다. WAV 재생 명령은 Linux에서
`aplay`, macOS에서 `afplay`이며 `AUDIO_PLAYER_COMMAND`로 바꿀 수 있다.

모델·카메라 없이 클래스별 음성 재생을 확인하려면 `tools/play_audio.py`의
`CLASS_ID`를 설정하고 실행한다. 현재 매핑에서는 `0`이 멧돼지용, `1`이
고라니용 음원이다. 한 번 실행할 때 해당 그룹에서 WAV 하나를 무작위로
선택해 재생한다.

```bash
uv run python tools/play_audio.py
```

동물 YOLO가 종을 직접 구분해야 현재 기본 분류기를 사용할 수 있다. 동물만
탐지하는 모델이라면 별도 종 분류 단계를 추가해야 한다. 대상은 프레임마다
허용된 클래스 중 신뢰도가 가장 높은 하나를 고른다. 여러 동물이 있을 때
객체 ID를 유지하는 추적 기능은 아직 없다.

기본 재생 규칙은 3프레임 연속 탐지, 탐지 사건당 1회, 모든 클래스 공통 10초
간격이다. 음원별 출처와 이용 조건은 [매니페스트](assets/audio/manifest.json)에
있다. 실제 동물 회피 효과와 카메라·모터·스피커를 연결한 동작은 검증 전이다.

## 얼굴 추적 (`main_facetrack.py`)

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

```bash
uv run main_facetrack.py           # 카메라 + YOLO 추적 실행
uv run main_facetrack.py --motor   # + Damiao 모터 제어
```

`--motor` 옵션 없이 실행하면 모터 없이 화면 표시만 한다.

### detection/ — 검출기 추상화

- `base.py`의 `PoseDetector`는 `detect(frame) -> List[PersonPose]` 하나만
  구현하면 되는 인터페이스다. `PersonPose`는 `nose`, `arm_up`, `box`, `face_box`를
  담은 데이터 클래스.
- `yolo_pose.py`의 `YoloPoseDetector`가 기본 구현체로, Ultralytics YOLOv8 pose
  모델(`assets/models/yolov8n-pose.pt`)을 로드해 COCO 17-keypoint 기준으로
  코 좌표·팔 든 상태·얼굴 박스를 계산한다. GPU(cuda:0)가 있으면 자동으로
  쓰고 없으면 CPU로 폴백한다.
- 다른(예: 파인튜닝된) YOLO pose 모델로 바꾸려면 `main_facetrack.py`의
  `MODEL_PATH`만 바꾸면 되고, 키포인트 레이아웃이 다른 모델을 쓰려면
  `PoseDetector`를 새로 구현하면 된다.

### mp3_player.py, audio/, legacy/

`mp3_player.py`/`audio/*.mp3`는 얼굴 추적 쪽 상황별 안내 음성이다.
`legacy/face_tracker_xl430_06_14.py`는 이전 세대 Dynamixel XL430 기반
트래커(참고용, 미사용), `legacy/local.py`는 라즈베리파이 영상 스트림
웹 서버(별개 용도)다.

### test/ (얼굴 추적 전용)

- `face_tracker_test.py`: OpenCV Haar cascade로 얼굴 박스만 찾아 방향(LEFT/
  RIGHT/UP/DOWN)을 표시하는 가벼운 테스트. YOLO 없이 카메라 프레임 좌표 계산과
  `Tracker` 연동만 확인하고 싶을 때 쓴다.
- `motor_test.py`: 카메라 없이 키보드로 가상 코 좌표를 입력해 `Tracker.update()`를
  직접 호출하는 모터 단독 테스트. `--motor` 없이 실행하면 어떤 좌표가 전송될지만
  출력하는 DRY_RUN이다. `--motor`로 실제 모터를 구동하면 매 명령 후, 그리고
  자동 순환(`a`) 각 방향 정착 후 실측 제어 주파수와 Yaw/Pitch 각 축의 실제
  위치·속도·토크(`get_actual_freq()`/`get_state()`)를 출력해 실제로 목표에
  도달했는지 확인할 수 있다.

## motor/dm4310_tracker.py — Damiao DM-J4310-2EC 트래커 (공용)

- Yaw/Pitch 두 개의 DM4310 모터를 Damiao 전용 USB-CAN 동글(시리얼, 921600 baud)로
  제어한다. 저수준 CAN 프레이밍은 `motor/DM_CAN.py`가 담당한다.
- 제어 모드는 MIT 모드(임피던스 제어). 모터 내부 MCU가
  `torque = Kp*(q_des - q) + Kd*(dq_des - dq) + tau_ff` 식으로 자체 전류 루프를
  항상 돌리고 있고, 호스트는 목표 위치(`q_des`)만 계속 갱신해서 보낸다 —
  실질적으로 위치 제어를, 부드러움은 MIT 임피던스가 물리적으로 담당하는 구조.
- 매 CAN 사이클(`controlMIT()` 호출 직후)마다 모터가 응답으로 돌려주는 실제
  위치/속도/토크를 `self._current`로 읽어와 다음 목표 계산의 기준(current
  value)으로 삼는 클로즈드 루프 구조다(얼굴 추적 쪽 `update(nose_x, nose_y)`
  기준 설명).
- 별도 스레드(`_control_thread`)가 목표 주파수 1000Hz(`CONTROL_FREQ`)로
  목표값을 MIT 프레임에 실어 CAN에 계속 스트리밍한다. 실측 주파수는
  `get_actual_freq()`로 확인 가능.
- `go_center()`는 현재 실측 위치에서 0 rad(중앙)까지 선형 램프로 서서히
  복귀시킨다.
- 가로(Yaw)/세로(Pitch) 픽셀→각도 환산계수 `RAD_PER_PIXEL_X`/`_Y`는 Arducam
  B0538C 렌즈 스펙(`HFOV_DEG`/`VFOV_DEG`, EFL 3.6mm, 픽셀피치 2.2µm)으로
  이론 유도한 식을 쓰지만(`motor/dm4310_tracker.py`의 상수 정의부 주석에
  유도 과정과 초점거리·픽셀피치 기반 교차검증이 있다), 짐벌 기어비(모터-
  카메라 간 감속비)가 반영돼 있지 않아 실제 `HFOV_DEG`/`VFOV_DEG` 값은
  이론값(82°/66°)에서 많이 벗어난 채 실기로 계속 미세조정되고 있다 —
  이론값으로 되돌리지 말 것. 가동범위(`YAW_RANGE_RAD`/`PITCH_RANGE_RAD`),
  축별 임피던스 게인(`KP_YAW`/`KD_YAW`/`KP_PITCH`/`KD_PITCH`)과 Pitch
  중력보상 피드포워드(`TAU_FF_PITCH`)도 실제 기구 한계·응답 특성에 맞춰
  튜닝해야 하는 값이다(자세한 배경은 위 "현재 진행 상황" 참고).

## 모터 점검 도구 (공용)

카메라·YOLO 없이 모터를 점검하는 도구다. 수동 제어와 좌표 이동은 실제
모터를 구동하며, 좌표 기준은 `motor/dm4310_tracker.py`의 2592×1944다.

```bash
# CAN ID 찾기/변경, 신호조회, 위치신호/이동, 제로잉, 카메라 테스트 메뉴
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
8. 카메라 테스트 (CAN ID 불필요)
0. 종료
```

**포트(시리얼 장치 경로)는 이 메뉴에서 따로 설정하지 않는다.**
`tools/actuator_conn_test.py`가 `motor/dm4310_tracker.py`의 `DEVICENAME`
상수를 그대로 기본값으로 쓰기 때문에, 실제 포트가 바뀌면 **`motor/
dm4310_tracker.py`의 `DEVICENAME` 한 곳만 고치면** 이 메뉴와 운영 코드
모두에 일괄 적용된다.

- **1번**은 CAN ID를 몰라도 된다 — 1~10번을 순서대로 찔러봐서(스캔) 응답하는
  ID를 찾고(모터 1대만 연결돼 있어야 함), 찾으면 원하는 새 ID로 바꿀지
  물어본다.
- **2번**은 CAN ID를 이미 알 때 스캔 없이 바로 타이핑해서 지정하는 메뉴다
  (예: Yaw=0x01, Pitch=0x02).
- **3~6번**은 1번 또는 2번으로 CAN ID가 지정된 뒤에만 실행된다.
- **5번(위치 이동)**은 4번과 달리 왕복하지 않는다 — 목표 각도로 이동한 뒤
  그 자리를 계속 유지한다(`controlMIT()`을 ~100Hz로 계속 스트리밍). 멈추려면
  Ctrl+C.
- **7번(게인 설정)**에서 바꾼 값은 그 실행 동안만 적용되며, `tools/
  actuator_conn_test.py` 상단의 `DEFAULT_KP`/`DEFAULT_KD`/
  `DEFAULT_DELTA_RAD`/`DEFAULT_HOLD_SEC` 상수 자체는 바꾸지 않는다.
- **8번(카메라 테스트)**은 모터/CAN과 무관하게 바로 실행된다. 카메라
  인덱스를 물어보되 Enter만 누르면 기본값(4)으로 시도한다.

```bash
# 지정 좌표로 두 축 이동 후 유지, Ctrl+C로 종료
./motor_start.sh 1200 900

# Yaw/Pitch 방향키 수동 제어 + 실제 카메라+YOLO 코 좌표 추적 테스트 메뉴
uv run python tools/motor_manual_control.py
./camera_moter_test.sh

# 카메라만 단독 테스트 (모터/YOLO 없이 미리보기)
uv run python tools/camera_test.py
uv run python tools/camera_test.py --index 0   # 인덱스 모르면 0,1,2... 시도
uv run python tools/camera_test.py --list      # 0~9번을 순서대로 열어보고 결과만 출력

# 두 축의 영점 설정
uv run python tools/zero_actuator.py
```

```
1. 동물 추적 + 종별 음향 재생 (멧돼지/고라니 탐지)
2. YOLO 코 좌표 추적 (손들기로 시작, 인식모드 진입 시 안내음)
3. 방향키로 위치 제어 (모터 각도 직접 조그)
0. 종료
```

1·2·3번 모두 **`q` 또는 `ESC`로 정상 종료하면** 공통으로 "숙인위치"
(`BOW_YAW_DEG`=0°, `BOW_PITCH_DEG`=-4°)로 `BOW_DURATION`(1초)에 걸쳐
복귀한 뒤 메뉴로 돌아온다 — `Ctrl+C`로 강제중지한 경우는 복귀 없이 바로
모터를 끈다(의도적으로 다르게 동작: 강제중지는 "최대한 빨리 멈추기"가
목적이므로 추가 이동을 끼워 넣지 않는다).

- **1번**: `main.py`(야생동물 음향 퇴치)와 동일한 `wildlife.py`/`camera.py`/
  `settings.py` 조합을 그대로 재사용해 멧돼지/고라니를 탐지하고, 탐지된
  bbox 중심으로 `Tracker`를 조준한 뒤 종에 맞는 WAV를 무작위로 재생한다.
  `main.py`를 그대로 실행하면 `Tracker`를 자기 안에 숨겨 갖고 있어 종료
  시 숙인위치로 보낼 수 없으므로, 이 메뉴는 `Tracker`를 직접 들고 있는
  채로 `main.py`의 `run()`과 같은 루프를 재구현한다. `settings.py`의
  `CAMERA_INDEX`/`MODEL_PATH`/`CLASS_TO_AUDIO` 등을 그대로 따른다.
- **2번**: `main_facetrack.py`와 같은 `detection.yolo_pose.YoloPoseDetector`로
  실제 카메라 화면에서 매 프레임 코 좌표·팔든상태를 인식한다. 손들기로
  추적이 확정되는 그 순간(락온)에 `audio/track_start2.mp3`를 1회
  재생한다(추적 중엔 다시 재생하지 않음). 화면에는 검출된 코 좌표를 항상
  표시하지만(대기 중엔 노란 점), 실제로
  `motor/dm4310_tracker.py`의 `Tracker.update(x, y)`를 호출해 모터를
  움직이는 건 그 사람이 `ARM_UP_THRESHOLD`(2초) 이상 손을 들어야 시작되고
  (확정되면 빨간 점으로 바뀜), `ARM_DOWN_GRACE`(10프레임) 넘게 손을
  내리면 멈춘다 — 코 좌표 인식과 실제 모터 추적을 손들기로 분리해서
  검증한다. "타겟 모드"(손들기로 추적 확정)가 아니면 — 코가 보이든 안
  보이든, 손을 드는 중(아직 `ARM_UP_THRESHOLD` 못 채움)이든 — 즉시
  `Tracker.go_to()`로 기본 자세(`DEFAULT_YAW_DEG`=0°,
  `DEFAULT_PITCH_DEG`=30°)로 1회 복귀시키고, 추적이 확정되면 재무장된다.
  카메라는 `tools/camera_test.py`의 `open_camera()`/`read_frame()`을
  재사용(`DEFAULT_CAMERA_IDX=4`).
- **3번**: `a`/`d` = yaw +/-, `w`/`s` = pitch +/-, `q` = 메뉴로 복귀. 모터
  각도를 직접 명령하는 raw 조그 모드. 한 번 입력될 때마다 `STEP_DEG`
  (기본 0.3°, 보수적으로 작게 잡음)만큼 누적되고, 키를 꾹 누르고 있으면
  터미널의 자동반복 입력으로 계속 쌓인다. 안전 가동범위(도, 나중에
  상수만 고치면 됨): Yaw `YAW_MIN_DEG`/`YAW_MAX_DEG` = -175 / 175,
  Pitch `PITCH_MIN_DEG`/`PITCH_MAX_DEG` = -30 / 180.
- 1·2번과 3번은 같은 시리얼 포트를 써서 동시에 열 수 없으므로, 1번이나
  2번에 들어가면 3번용 연결을 잠깐 닫았다가 메뉴로 돌아오면 다시 연다.

영점 설정 도구(`tools/zero_actuator.py`)는 모터를 활성화하지 않고, 손으로
맞춘 위치를 영점으로 지정한 뒤 플래시에 저장한다. CAN ID 변경도 플래시에
저장한다.

## Setup

```bash
uv sync
```

하드웨어 준비 (실제 모터 투입 전 필수 확인):
- 카메라: Arducam B0538C(OG05B1B, 5MP 흑백 글로벌 셔터, USB 3.0). 인덱스 4가
  `/dev/video4`로 맞게 잡히는 것 실측 확인됨. USB 3.0 연결 시 2592×1944 @
  25fps가 스펙상 최대치(USB 2.0은 5fps로 급감). 야생동물 쪽은
  `settings.py`의 `CAMERA_INDEX`, 얼굴추적 쪽은 `main_facetrack.py`의
  `CAMERA_IDX`에서 바꾼다.
- Damiao 전용 USB-CAN 동글 — 이 프로젝트 기준 `/dev/ttyACM0`으로 잡힘(동글
  칩셋에 따라 `/dev/ttyUSB0`로 잡힐 수도 있음, `motor/dm4310_tracker.py`의
  `DEVICENAME`에 실측값 반영돼 있음). 다른 PC에 연결하면 `ls /dev/ttyUSB*
  /dev/ttyACM*`로 재확인할 것
- Yaw=CAN ID `0x01`, Pitch=CAN ID `0x02`(MasterID `0x11`/`0x12`)로 확정·
  설정됨 — `./actuator_test.sh`의 "CAN ID 찾기/할당"으로 Damiao 공식
  소프트웨어 없이 CAN 통신만으로 확인/변경 가능. MIT 모드는 `Tracker`
  초기화 시 자동으로 전환하므로 별도 사전 설정 불필요
- `motor/dm4310_tracker.py`의 임피던스 게인은 Yaw/Pitch 축별로 분리돼
  있다(`KP_YAW`/`KD_YAW`, `KP_PITCH`/`KD_PITCH`). 실기로 계속 튜닝 중인
  값이니 "안전하게 낮추자"며 임의로 되돌리지 말 것 — 현재 값은 파일에서
  직접 확인할 것. `TAU_FF_PITCH`는 Pitch 축 자중으로 인한 steady-state
  오차를 보정하는 피드포워드 토크로, 역시 실측 기반으로 계속 조정 중이다
  (Yaw는 `TAU_FF_YAW=0` 고정). `YAW_RANGE_RAD`/`PITCH_RANGE_RAD`(가동범위)
  는 실제 기구 한계에 맞춰 튜닝할 것. 픽셀→각도 환산계수
  `RAD_PER_PIXEL_X`/`_Y`(및 그 기준인 `HFOV_DEG`/`VFOV_DEG`)는 렌즈 스펙
  이론값에서 출발했지만 짐벌 기어비 미반영으로 실기 미세조정 중이라 현재
  값이 이론값과 많이 다르다 — 정상이니 이론값으로 되돌리지 말 것.
  `SIGN_YAW`/`SIGN_PITCH`는 실측 완료(둘 다 `+1`) — 다른 기구/배선으로
  바뀌면 재검증 필요
- 야생동물 쪽: 기본으로 `assets/models/weights.pt` 가중치를 쓴다. 다른
  모델을 쓰려면 `settings.py`의 `MODEL_PATH`를 수정하고, `CLASS_TO_AUDIO`/
  `EXPECTED_MODEL_NAMES`가 실제 모델의 클래스 ID·이름과 일치하는지 확인할 것
  (안 맞으면 카메라·모터를 열기 전에 실행이 중단된다)
