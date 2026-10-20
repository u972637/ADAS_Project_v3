# ADAS Project — 야생동물 퇴치 시스템

카메라 + YOLO 객체 인식 + 2축(Yaw/Pitch) 짐벌 모터 + 지향성 스피커로, 영상에
잡힌 멧돼지/고라니를 조준하고 종별 경계음을 재생해 쫓아내는 프로젝트다.
기존 보행자 안내 시스템(`people/`)의 카메라·모터·음향 구조를 재사용해서 만들었다.

## 빠른 시작

```bash
uv sync
uv run python main.py
```

이 한 줄로 **카메라 → 동물 검출/추적 → 모터 조준 → 종별 경계음 재생 →
정지 시 기본 자세 복귀**까지 전부 돈다. 모터는 기본 ON, 시작하자마자
탐색 상태로 들어간다.

```bash
uv run python main.py --start-paused   # W(재개) 입력 전까지 조준/재생 대기
uv run python main.py --dry-run        # 실제 CAN·오디오 없이 모의 실행
uv run python main.py --headless       # 미리보기 창 없이 터미널 명령으로 운용
uv run python main.py --no-motor       # 검출만 확인, 모터/재생 없음
```

옵션은 조합 가능하며, 생략한 값은 `animal/settings.py` 기본값을 따른다.

### 조작

- 창: **R / E / Space = 정지**, **W = 재개**, **Q / ESC = 종료**, **U = 현재 빨간점(추적 중인) torso의 픽셀 크기를 `log/torso_size.csv`에 기록**
- 터미널: `stop` / `resume` / `quit` + Enter (추론 중에도 별도 스레드가 받음)

## 실험 메뉴 (`./start.sh` = `tools/motor_manual_control.py`)

정식 운용(`main.py`)과는 독립된 번들이며, 서로 호출하지 않는다.

| 메뉴 | 내용 |
| --- | --- |
| 1 | 동물 모델로 정식 코어(`animal/runtime.py`) 그대로 실행 — 조준·재생 포함, W로 시작 |
| 2 | 사람을 동물처럼 대역 투입해 같은 코어를 시험 (`tools/person_proxy.py`) |
| 3 | 사람이 손을 들면 추적 시작하는 별도 데모 — 코 좌표를 쫓아가며, **A=멧돼지 경보음 / S=고라니 경보음 / D=정지** 키로 독립된 음향을 수동 재생 |
| 4 | 방향키로 모터 각도 직접 조그 |
| 5 | 음향 측정용으로 yaw=0°, pitch=0° 고정 |

## 전체 흐름

```
카메라(animal/camera.py) → YOLO 검출 + 종별 ByteTrack(animal/inference.py)
  → 대상 선정/도주 판정(animal/perception.py: EscapeTracker)
  → 상태 머신(animal/wildlife.py: WildlifeDeterrent)
  → 조준 평균/데드밴드(animal/aiming.py) → 모터(motor/)
  → 종별 경계음 재생(animal/audio.py)
```

### 검출/추적 — `animal/inference.py`, `animal/perception.py`

- 멧돼지(`boar`)·고라니(`water_deer`)는 각각 **별도 ByteTrack**으로 추적해 종
  간 ID 재사용을 막는다(`AnimalIdentityTracker`).
- `torso`(종 무관, 조준 기준점)는 ByteTrack에 넣지 않고, 매 프레임 동물
  bbox와의 포함관계/거리로 가장 가까운 것을 매칭한다.
- `EscapeTracker`가 한 번에 하나의 대상만 락온한다. 동물 bbox가 잠깐
  안 보여도 직전 torso 위치(`_last_torso`) 주변에서 대체 torso를 찾아
  **추적을 끊지 않고 이어준다**(단, 그사이 종이 바뀐 걸로 보이면 대체하지
  않음). 이 대체 매칭도 안 되는 채로 `TRACK_LOST_SECONDS`(2초)가 지나면
  그제서야 추적 유실로 확정한다.

### 상태 머신 — `animal/wildlife.py`

`Phase`는 `STOPPED / SEARCH / TRACKING / FAULT` 네 가지뿐이다. 과거처럼
"조준 → 고정 → 관찰" 단계를 나누지 않고, 대상이 잡히면 **모터는 매 프레임
계속 조준하고**, 재생 여부만 별도 조건으로 켜고 끈다.

**빨간점(화면에 표시) = 그 프레임에 모터가 실제로 조준 명령을 보냈는지**
여부다 (`matched_torso` 존재 여부). 재생 중인지와는 무관하다 — 소리가
꺼져 있어도 모터는 계속 추적할 수 있고, 반대로 소리가 나는 동안에도
빨간점이 꺼질 수 있다(이 경우는 추적 유실 전 과도기로, 곧 노란점/종료로
이어진다).

재생(경계음)은 빨간점과는 별도 조건으로 켜고 끈다:

| 동작 | 조건 |
| --- | --- |
| 재생 **시작** | 조준점(matched_torso)이 화면 중앙 ±5%(`PLAYBACK_ENTER_FRACTION`) 이내. 동물 bbox가 이번 프레임에 안 보여도(대체 매칭 중이어도) 이 조건만 맞으면 시작한다 |
| 재생 **유지** | 아무 조건 없이 계속 — 한 번 시작하면 다음 조건으로 꺼질 때까지 무작위 클립을 0.3초(`PLAYBACK_GAP_SECONDS`) 간격으로 이어 재생 |
| 재생 **종료** | 동물 bbox가 실제로 보이고(대체 매칭이 아니고) 조준점이 중앙 ±15%(`PLAYBACK_EXIT_FRACTION`) 밖으로 나갈 때. bbox가 잠깐 안 보이는 동안에는 이 조건을 평가하지 않아 재생이 끊기지 않는다 |

### 추적 종료 조건

| 조건 | 결과 |
| --- | --- |
| bbox 면적이 기준 대비 50%(`AREA_RATIO_THRESHOLD`) AND 높이 80%(`HEIGHT_RATIO_THRESHOLD`)로 줄어든 채 유지 | 이탈 종료 |
| bbox 높이가 절대 100px(`MIN_BBOX_HEIGHT_PX`) 미만 | 너무 멀어서 종료 |
| 대체 매칭도 2초(`TRACK_LOST_SECONDS`) 넘게 실패 | 추적 유실 종료 |

종료되면 재생을 끄고, 기본 자세로 **8초(`TERMINATION_RETURN_SECONDS`)에
걸쳐 천천히** 복귀한다(급하게 홱 움직이지 않도록).

### 조준 — `animal/aiming.py`

torso 중심 좌표를 3프레임(`AIM_AVERAGE_FRAMES`) 평균하고, 중앙 ±0.3%
데드밴드, 고립된 큰 점프(`AIM_MAX_JUMP_FRACTION=15%`)는 이상치로 한 번
거부한다(연속 2프레임 같은 방향이면 실제 재배치로 받아들임). 모터에는
매 프레임 계속 갱신값을 보낸다 — 평균 채우는 중/이상치 거부 시에는
직전 목표를 유지.

## 모터 — `motor/`

`motor/config.py`가 좌표계·게인·통신의 단일 설정 위치다.

| 항목 | 값 |
| --- | --- |
| 논리 각도 | 오른쪽 Yaw+, 위쪽 Pitch+ (raw 모터 각도는 양 축 모두 반대 부호) |
| 제어 주파수 | 500 Hz |
| 유효 수평/수직 화각 | 27.5° / 20° |
| Yaw / Pitch 범위 | ±1.2 / ±0.6 rad |
| 공유 기본 복귀 자세 | 0° / 15° |
| 프로그램 종료(Q) 전용 자세 | 0° / −4° |
| USB-CAN | `/dev/ttyACM0`, 921600 baud |

두 가지 제어 스타일을 `CONTROL_MODE`(`"trajectory"` 또는 `"step"`)로
고르며, `motor/factory.py` 한 곳에서만 실제 Tracker를 생성한다.

- **`trajectory`**(`motor/dm4310_tracker.py`): Ruckig으로 속도·가속도·jerk
  상한을 지키는 연속 기준 궤적을 생성.
- **`step`**(`motor/dm4310_tracker_step.py`): 매 프레임 스텝 위치 목표만
  주고 PD 루프 자체 댐핑에 맡기는 이전 방식. 현재 기본값.

각 모드는 Kp/Kd 등 튜닝값을 완전히 분리해서 가진다(`TRAJECTORY_*` /
`STEP_*`).

### 프로세스 분리 (`MOTOR_PROCESS`)

`animal/settings.py`의 `MOTOR_PROCESS = True`(기본)면 500 Hz 모터 제어
루프가 **별도 OS 프로세스**(`motor/ipc_tracker.py`)에서 돈다. 메인 프로세스의
무거운 YOLO 추론이 같은 프로세스의 GIL을 쥐고 있어도 모터 제어 스레드가
굶지 않게 하기 위함이다. `False`로 바꾸면 기존처럼 한 프로세스 안 스레드로
돈다(`motor/factory.create_tracker`).

## 음원 — `animal/audio.py`, `assets/warnings_pitch_eq/`

종 매핑: 모델 `0=boar`→`boar`, `1=deer`→`water_deer`, `2=torso`→조준 전용
(소리 없음). 경계음 파일은 `assets/warnings_pitch_eq/<종>/*.wav` 한 곳만
쓴다(피치 시프트 + EQ로 스피커 지향성이 잘 드러나는 1.5~2kHz 대역에 에너지를
몰아준 버전). 종 폴더가 비어 있으면 해당 종은 경고만 찍고 재생을 안 한다
(프로그램이 멈추지 않음).

재생은 외부 플레이어(`aplay`/`afplay`)를 서브프로세스로 띄워서 하며,
종료 시 `terminate()`만 보내고 실제 프로세스 정리는 백그라운드 스레드가
맡는다 — 메인 루프(카메라/모터)가 재생 종료 때문에 멈추지 않도록 하기 위함.

카메라·모터 없이 음원만 들어보기:

```bash
uv run python tools/play_audio.py --label boar
uv run python tools/play_audio.py --label water_deer --dry-run
```

`tools/eq_filter.py`, `tools/pitch_shift.py`는 예전 다중 버전(EQ/피치)
자산을 만들던 도구로, 지금은 쓰지 않는 폴더 구조를 참조하고 있어 **그대로
두면 바로 실행은 안 된다** — 필요해지면 그때 정리한다.

## 로그 — `log/`

메뉴 1/2 화면에서 **U 키**를 누르면, 그 순간 빨간점(모터가 실제로 조준
중인 torso)이 있을 때만 `log/torso_size.csv`에 한 줄이 추가된다:
`timestamp, width_px, height_px, area_px`. 빨간점이 아닐 때는 눌러도
아무 일도 없다. 거리별 픽셀 크기 캘리브레이션 등에 쓴다.

## 음향 측정 데이터 — `audio_test/`

스피커 거리·각도·주파수별 음압(dB) 측정 결과를 기록하는 CSV 모음.
자세한 열 설명은 [`audio_test/README.md`](audio_test/README.md) 참고.

## 테스트

```bash
uv run python -m unittest discover -s tests -v
```

하드웨어 없이 상태 머신, 조준 평균/이상치 거부, 이탈 판정, 재생
시작/유지/종료 하이스테리시스, 모터 부호/변환, 궤적 생성기 등을 검증한다.
실제 카메라·모터·스피커 동작 자체는 이 테스트 범위 밖이다(실기에서 별도
확인 필요).

## 디렉터리 개요

| 경로 | 역할 |
| --- | --- |
| `main.py` | 정식 운용 진입점 |
| `animal/` | 검출~재생 전체 파이프라인(정식 코어) |
| `motor/` | 모터 제어(trajectory/step), 프로세스 분리, 좌표계 설정 |
| `tools/` | 실험 메뉴, 음원 가공 도구, 수동 모터 조그 — 정식 코어가 이 폴더를 import하지 않음 |
| `people/` | 기존 보행자 안내 앱(`main_facetrack.py`) — 이 프로젝트와 별개 용도로 공존 |
| `detection/` | pose 검출 인터페이스/구현(사람 대역, 얼굴 추적용) |
| `assets/` | 모델 가중치, 경계음, 안내 음성 |
| `audio_test/` | 음압 측정 데이터 |
| `log/` | 실측 로그(torso 픽셀 크기 등) |
| `tests/` | 하드웨어 없는 회귀 테스트 |
| `legacy/` | 이전 세대 자료(참고용, 운영 경로에서 쓰지 않음) |
