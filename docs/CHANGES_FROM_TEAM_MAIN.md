# 팀원 최종 main 이후 전체 변경 내역

작성: 2026-10-18 KST. **이 문서는 마지막 커밋 하나의 변경 로그가 아니다.**
팀원들이 통합 작업 전에 마지막으로 main에 올린
[`9eaa1485e3076d591c53ae0249af5a5c6308f40b`](https://github.com/u972637/ADAS_Project_v3/commit/9eaa1485e3076d591c53ae0249af5a5c6308f40b)
(`퇴치 음원 음높이를 1.5~2kHz 쪽으로 올린 버전 추가 (pitch, pitch+EQ)`)를 기준으로,
그 이후 이 문서가 포함된 커밋까지의 통합 수정 전체를 설명한다.

인수인계 순서: 이 문서의 전후 비교 → [역할 구조도·상태 머신](INTEGRATION_HANDOFF.md)
→ [실행 안내](../README.md) → 제어 담당자는 [궤적 상세](MOTION_CONTROL.md).

## 1. 비교 범위와 작업 이력

| 변경 | 내용 |
| --- | --- |
| 기준 `9eaa148` | 팀원 최종 main. 기존 동물/torso 탐지·ID 고정·도주 감지, DM4310 제어, 종별 WAV와 EQ/pitch 변형 보유 |
| `8a96e3d` | 통합 상태 머신, 조준 수렴/고정, 경계음→관찰→위협음, 공통 런타임·설정, 오류 처리와 회귀 테스트 |
| `baebcc9` | 팀원의 `feat/escape-and-height` `32882c1` 병합. 높이 AND 면적, 캡처 시각 기준 이동 속도를 운영 시나리오에 연결 |
| `a9f93d2` | main 단독 정식 실행, 실험 어댑터 분리, 수렴 속도 0.2 rad/s, 설정 기반 default 복귀 |
| 이 문서를 포함한 커밋 | 연속 p/v/a 궤적, v/a/jerk 상한, q/dq 송신, 감속·기준 수집 연동, 계산 벤치마크와 전체 인수인계 갱신 |

기준 이후 차이는 다음 명령으로 재확인할 수 있다. 향후 추가 변경이 생기면 `HEAD` 대신
이 문서를 처음 추가한 커밋을 지정하면 이번 인수인계 범위만 비교된다.

```bash
git log --first-parent --oneline 9eaa148..HEAD
git diff --stat 9eaa148 HEAD
git diff --find-renames 9eaa148 HEAD
git log --diff-filter=A --format='%H %s' -- docs/CHANGES_FROM_TEAM_MAIN.md
```

## 2. 실행 구조와 상태 소유권

| 항목 | 기준 main | 현재 |
| --- | --- | --- |
| 정식 실행 | main에 카메라·추론·키 처리 루프, 모터 기본 OFF | `main.py` → `animal.runtime.run()`. 실제 모터 기본 ON, 전체 야생동물 시나리오 실행 |
| 메뉴 1/2 | 수동 도구에 별도 운영 루프와 W 게이트. W 전에도 탐지 기반 음향 가능 | 같은 `run_session()` 재사용. W/resume 전 조준·음향 모두 대기 |
| 정식/실험 의존성 | 사람 대역이 `animal/person_proxy.py`와 공통 settings에 존재 | 사람 대역·전용 설정은 `tools/person_proxy.py`. 정식 코드는 tools를 import하지 않음 |
| start.sh | 메뉴 동작 설명을 가진 실행 스크립트 | 선택적 실험 launcher. main에서 호출하지 않으며 정식 실행에 필요 없음 |
| 재생 결정 | 독립 `TriggerPolicy`: 연속 탐지·쿨다운 등으로 재생 결정 | `WildlifeDeterrent` 한 곳이 대상/조준/고정/재생/종료를 결정 |
| 영상 형식 | main과 메뉴/카메라 도구의 변환 경로 차이 | `animal.camera`의 packed YUY2 처리 재사용 |
| 좌우 좌표 | main과 기존 보행자 경로에서 영상 반전, 실험 경로와 차이 | 반전 없는 영상으로 통일. 모터 어댑터만 부호 변환 |
| 입력/추론 대기 | 메인 루프에서 키 처리; 추론/영상 대기 시 대응 지연 | 별도 RuntimeGuard가 터미널 stop과 watchdog을 약 50 ms 주기로 처리 |

main 옵션: `--start-paused`, `--dry-run`, `--headless`, `--no-motor`.
`--dry-run`은 실제 카메라/모델을 사용하면서 CAN·음향만 모의 실행한다.
`--no-motor`는 실제 조준 수렴을 확인할 수 없으므로 자동 음향을 시작하지 않는다.
메뉴 3의 손들기 데모와 기존 `people/main_facetrack.py`는 정식 야생동물 상태 머신과 별개다.
main와 메뉴 1/2의 정상 종료는 공유 default로 복귀한다. 별도 실험 메뉴 3/4/5의
정상 종료에는 기존 측정용 숙인 자세 Yaw 0°/Pitch −4°를 유지하며, 이동 궤적 생성만 공유한다.

## 3. 검출·개체 연결·이탈 판정

| 항목 | 기준 main | 현재 |
| --- | --- | --- |
| ByteTrack 입력 | `model.track()`에 동물과 torso 클래스가 함께 들어감 | raw `predict()` 후 동물 종별 ByteTrack, torso는 추적기에 넣지 않음 |
| ID 격리 | 클래스가 다른 박스 사이 ID 이동 가능 | `(종, 내부 track ID)`를 세션 ID로 변환. 없는 종도 매 프레임 tracker 갱신 |
| 낮은 신뢰도 torso | ByteTrack의 track 생성 문턱으로 소실 가능 | raw torso를 confidence 0.2부터 유지, 동물 bbox와 일대일 연결 |
| torso 연결 | 기존 포함관계/거리 기반 매칭 보유 | 같은 개념을 perception으로 분리. torso가 없으면 전체 bbox 중심으로 대체하지 않음 |
| 사람 대역 | tracking 결과와 pose 인덱스 순서가 어긋날 여지 | raw box/keypoint의 원래 정렬에서 어깨·골반 4점 torso를 만든 뒤 공통 추적에 연결 |
| 대상 확정 | 기존 ID 고정과 재생 정책이 분리 | 같은 ID·종 3프레임을 확인하고 조준 시작 |
| 면적 기반 후퇴 | 면적 ≤50% | 면적 ≤50% **AND** 높이 ≤80% |
| 기준/평활/조건 유지 | 1.0 / 0.5 / 1.0초 | 0.4 / 0.35 / 0.5초 |
| 빠른 이동 | 최근 5점의 프레임당 이동량, 폭의 0.15배 문턱 | 캡처 시각 차이로 속도 계산. 0.10×명목 25 FPS = 초당 bbox 폭 2.5배 |
| 카메라가 조준 중 | 화면 이동도 도주로 처리할 수 있음 | 조준·감속 중 관찰 이력을 버림. 고정 완료 후 촬영된 영상부터 기준 수집 |
| 이탈 프레임 | 종료 판정과 해당 프레임 조준/재생이 섞일 수 있음 | 종료를 최우선 처리하고 즉시 return. 같은 프레임 조준/재생 금지 |
| 재선정 | ID 고정 해제와 유예 중심 | 1초 유예 + 이탈 ID가 계속 보이면 제외, 2초 미검출 후 제외 해제 |

동물 bbox의 **2초 유실** 기준은 유지한다. 짧은 누락은 기존 대상의 복귀를 기다리며
모터 목적지를 유지한다. 재생 중 짧은 누락도 즉시 성공/종료로 오인하지 않는다.
크기·이동 이력은 경계음 전 고정과 위협음 전 재고정에서 각각 새로 수집한다.

높이 AND 면적과 시간 정규화는 **팀원 브랜치 `32882c1`의 기여**다.
그 알고리즘을 분리된 `perception.py`에 이식하고, 움직이는 카메라 제외·종료 우선·
음향 중단·테스트와 연결한 부분이 이번 통합 작업이다. 원본의 540개 시뮬레이션 성능
주장은 재현 자료가 없으므로 검증된 실측 결과로 문서에 사용하지 않았다.

빠른 이동은 **가속도 측정이 아닌 속도 휴리스틱**이다. 실제 보호 구역 밖으로 나갔는지,
음향 때문에 도망갔는지, 퇴치에 성공했는지는 아직 판단하지 못한다.

## 4. 조준·모터 제어

| 항목 | 기준 main | 현재 |
| --- | --- | --- |
| 시각 좌표 필터 | torso 좌표를 목표 보정으로 전달 | 3프레임 평균, 중앙 deadband, 고립된 큰 점프 거부; 연속 2회 일치하는 큰 이동은 허용 |
| subpixel | bbox 중심은 float 계산 | float 유지. 조준/제어에 정수 반올림 추가 없음. 화면 표시만 int |
| 중앙 검출 | 오차 양 축 0이면 return하여 이전 비중앙 목표가 남을 수 있음 | 중앙에서도 목적지를 최신 실측 위치로 갱신 |
| 조준 완료 | 통합 재생용 수렴 판정 없음 | raw/평균 각각 중앙 ±1.5%, 3샘플 이상, 양 축 실측 속도 ≤0.2 rad/s, 0.5초 유지 |
| 축 부호 | 영상 반전·오차식·SIGN에 분산 | 논리 오른쪽/위쪽 +, raw Yaw/Pitch 모두 −. 위치·속도·피드백을 어댑터에서 변환 |
| 설정 | 운영/수동의 주파수·범위와 저수준 게인 기본값 불일치 | `motor/config.py` 공유. 수동도 500 Hz·Yaw ±1.2/Pitch ±0.6 rad |
| 추적 명령 | 계단형 q 목표, dq=0. 부드러움은 PD에 의존 | 기존 명령 p/v/a를 잇는 궤적에서 q_ref/dq_ref 송신 |
| 목적지 처리 | 실측 위치 + 영상 오차 | 같은 목적지 계산 유지. 새 목적지만 교체하며 지난 프레임을 큐에 쌓지 않음 |
| 궤적 상한 | 1회 목표 증분/각도 범위만 제한 | v/a/jerk 제한 추가, 생성 궤적 전체 위치 극값 검사 |
| 복귀 | 선형 ramp, 새 검출이 취소 가능 | 제한된 궤적, stop 상태를 명시적으로 해제하기 전 검출이 복귀를 취소하지 못함 |
| default | 운영 go_center는 0°/0°, 수동 실험 default는 0°/+30° | 공통 `go_default()`는 0°/+30°. 설정 한 곳에서 변경; `go_center()`는 0°/0° 유지 |
| 고정 | 별도 단계와 감속 완료 확인 없음 | 제한 내 감속→최종 위치 유지. 완료·실측 속도 확인 후 새 영상 기준 수집 |

이번 통합 중 처음 추가했던 수렴 속도 0.05 rad/s는 사용자 요청으로 0.2로 완화했다.
기준 main과 비교할 때는 **수렴 판정 자체가 새 기능**이다.

기존 운영 튜닝 **500 Hz, Kp/Kd 30/1.8, FOV 28°/20°, raw Pitch +2.3 N·m,
1회 각도 보정 최대 0.6 rad**는 유지한다. `DEADZONE_PX`는 0→8이며 조준 어댑터의
0.003 화면 비율 deadband도 있다. 양 축 부호는 사용자가 확인해 준 방향을 따른다.
실제 기구로 게인·FOV·상한을 새로 검증했다는 뜻은 아니다.

새 v/a/jerk 상한은 Yaw **1.5/8/100**, Pitch **1.2/6/80**(rad/s, rad/s², rad/s³).
상한 선택·40 ms와 이동 시간의 차이·벤치마크는 [MOTION_CONTROL](MOTION_CONTROL.md)에 있다.

## 5. 음향과 장치 오류 처리

| 항목 | 기준 main | 현재 |
| --- | --- | --- |
| 기존 WAV의 역할 | 탐지 시 종별 WAV 선택 | 기존 각 종 10개 WAV는 위협음으로 사용 |
| 경계음 | 별도 단계/목록 없음 | `assets/warnings`와 `warnings_eq` 분리. 아직 파일 없음 |
| 단계 | 탐지 조건에 따른 재생·쿨다운 | 조준→감속/기준→경계음 1회→정상 종료 후 5초→재조준/기준→위협음 1회→고정 관찰 |
| 누락 음원 | 경계음 개념 없음 | 경계음 누락 상태로 대기. 위협음으로 대체하지 않음 |
| 완료/실패 | 프로세스 실행 여부 중심, stderr 폐기 | idle/playing/completed/failed/stopped와 returncode/stderr 보존 |
| dry-run | 재생을 즉시 끝난 것으로 취급 | WAV 실제 길이만큼 재생 시간을 모의 실행 |
| 초기 모터 | MIT 확인 실패 때 경고, 응답 없는 기본 위치를 쓸 가능성 | MIT 확인과 양 축 실제 피드백 확보 전 시작 불가 |
| 운전 중 피드백 | 캐시된 값과 통신 예외를 상위 상태로 연결하지 못함 | 0.5초 stale, 비유한 값, 고장 코드, 스레드 종료를 감시하고 음향 중지/고장 전달 |
| 영상/추론 지연 | 별도 통합 watchdog 없음 | 캡처/추론 각 2초 timeout, 1초 초과 추론 결과 폐기, stop 전 in-flight 결과도 세대 번호로 폐기 |
| 제어 루프 지연 | 실제 주파수 표시 중심 | 계산/틱/CAN/목표 반영 지연 기록. 40 ms 초과 감지 시 고장 및 양 축 disable 시도 |

원래 있던 무작위 비반복 선택, WAV 검사, original/EQ/pitch/pitch_eq 폴더와 생성 도구는
유지한다. 이번 통합에서 음원 파일·EQ 계수·pitch 알고리즘을 새로 만들거나 재측정하지 않았다.
재생기 exit=0은 소프트웨어 완료 확인이며 실제 스피커 음압 확인은 아니다.

## 6. 파일별 인수인계 목록

아래는 기준 main 이후 변경된 파일 전체를 기능별로 묶은 목록이다. 테스트 폴더는 전부 새로 추가했다.

| 파일 | 변경 및 확인 위치 |
| --- | --- |
| `main.py` | CLI 옵션만 해석하는 정식 진입점. 공통 런타임 호출 |
| `start.sh` | 실험 도구 exec launcher |
| `animal/runtime.py` (신규) | 공통 영상/추론/입력 루프, 독립 watchdog, 시작 옵션과 종료 복귀 |
| `animal/wildlife.py` | 독립 TriggerPolicy를 상태 머신으로 대체; 검출 자료구조/알고리즘 분리; 기존 일부 import 이름 호환 유지 |
| `animal/perception.py` (신규) | Detection/Event, 결과 파싱·모델 클래스 확인, torso 매칭, ID 잠금, 면적/높이·속도 이탈 |
| `animal/inference.py` (신규) | raw predict, 종별 ByteTrack, torso 분리, 실험 torso reader 주입 |
| `animal/aiming.py` (신규) | 좌표 필터, 수렴 판정, 모터 고정/복귀 어댑터, 명시적 모의 제어 |
| `animal/audio.py` | 재생 상태와 오류 보존, 중지 처리, 실제 WAV 길이의 dry-run |
| `animal/camera.py` | 최신 캡처 시각을 watchdog에 제공. 기존 공통 YUY2 변환을 다른 도구에서도 재사용 |
| `animal/settings.py` | 시나리오·조준·이탈·음향·실행 기본값 정리. 옛 cooldown/release 설정 제거, 실험 설정 이동 |
| `animal/person_proxy.py` → `tools/person_proxy.py` | 실험 전용으로 이동; raw pose 정렬 보존, 종 매핑/모델/키포인트 설정과 실행 어댑터 포함 |
| `motor/config.py` (신규) | 장치·좌표·게인·범위·default·궤적 상한·시간 제한의 공통 설정 |
| `motor/dm4310_tracker.py` | 초기 응답 확인, 논리 좌표 API, 연속 궤적 q/dq 송신, 최신 목표 교체, 고정/복귀/오류/지연 관측 |
| `motor/trajectory.py` (신규) | I/O 없는 Ruckig wrapper. p/v/a 연결, 제한·범위 검사, 상태와 계산 시간 |
| `tools/motor_manual_control.py` | 메뉴 1/2의 복제 운영 루프 제거, 공통 runtime 호출. 4/5 공통 Tracker 재사용, 완료 대기 |
| `tools/actuator_conn_test.py` | 기본 Kp/Kd를 공통 설정에서 읽음, 상태 0/1과 고장 코드 구별 |
| `tools/actuator_test.sh` | Python 도구의 기본 게인·각도·유지 시간을 읽어 메뉴와 일치 |
| `tools/camera_test.py` | 자체 변환을 제거하고 공통 camera 함수·설정 사용 |
| `tools/motor_goto.py` | 실제 500 Hz 동작에 맞춰 설명 수정; 공통 Tracker 변경도 적용됨 |
| `tools/play_audio.py` | 가짜 탐지로 재생시키던 도구를 직접 warning/threat·종 선택 CLI로 교체 |
| `tools/benchmark_motion.py` (신규) | 장치 없는 계산 벤치마크와 이동 계획 시간 출력 |
| `people/main_facetrack.py` | 영상 반전 제거, 논리 방향/500 Hz 주석 일치. 기존 보행자 앱 유지 |
| `pyproject.toml`, `uv.lock` | Ruckig 0.19.4 추가·잠금. 기존 모델/오디오 패키지 버전을 이번에 일괄 업그레이드하지 않음 |
| `assets/warnings/{boar,water_deer}/.gitkeep` | 경계음 원본의 두 빈 폴더 |
| `assets/warnings_eq/{boar,water_deer}/.gitkeep` | 경계음 EQ의 두 빈 폴더 |
| `tests/__init__.py` | 회귀 테스트 패키지 |
| `tests/test_aiming.py` | 평균·이상치·중앙 취소·수렴·속도 경계 |
| `tests/test_audio_camera.py` | WAV/모의 길이/재생 실패, YUY2/공통 디코더 |
| `tests/test_escape_tracker.py` | AND 판정·회전/후퇴·시간 정규화와 실제 설정의 시나리오 연결 |
| `tests/test_inference.py` | 실제 ByteTrack의 종/torso 분리·빈 프레임·pose 정렬 |
| `tests/test_motor.py` | 부호·목표·초기/운전 고장·지연·양 축 송신 후 완료 |
| `tests/test_runtime.py` | main 단독/대기/dry-run, 실험 분리, 독립 입력/watchdog |
| `tests/test_scenario.py` | 전체 단계·이탈/유실/stop 우선순위·재생 실패·감속/촬영 시각 연동 |
| `tests/test_trajectory.py` | p/v/a 연속·v/a/jerk·반전·지터·제동·범위/입력 오류 |
| `README.md` | 정식 실행과 실험, 운영 기준, 설정·음원·검증 안내를 최종 구조로 갱신 |
| `docs/INTEGRATION_HANDOFF.md` | 역할 구조도·상태 머신·돌발상황·팀별 구현 이력·남은 업무 |
| `docs/CHANGES_FROM_TEAM_MAIN.md` | 이 문서. 팀원 최종 main부터의 전체 전후 비교 |
| `docs/MOTION_CONTROL.md` | 궤적 선택 이유·튜닝·지연과 해상도·실측해야 할 항목 |
| `docs/benchmarks/motion_local.json` | 이 PC의 계산 벤치마크 원본 |

`motor/DM_CAN.py`의 프로토콜 구현, 모델 가중치, 기존 WAV·voice MP3, `audio_test/` 측정 자료,
`tools/eq_filter.py`, `tools/pitch_shift.py`, `animal/animal_escape_tracking_combined.py`,
`legacy/`는 이번 통합에서 수정하지 않았다. 특히 오프라인 분석 파일이 정식 런타임에서
호출되는 것으로 오해하지 않도록 한다.

## 7. 동료가 이어서 할 일과 검증 범위

전체 회귀 테스트 **75개 통과**, Python 정적 오류 검사·셸 구문·uv lock 일치 확인.
검증 Python 3.10.20 / Ultralytics 8.4.72 / NumPy 1.26.4 / lap 0.5.12 / Ruckig 0.19.4.
테스트 환경의 OpenCV는 4.11.0이고 프로젝트 잠금은 4.10.0.84라서 완전히 동일한 환경의
검증으로 표현하지 않는다. 실제 카메라·CAN·스피커는 열지 않았다.

| 우선 작업 | 담당 영역 | 완료 증거 |
| --- | --- | --- |
| 경계음 업로드 | 음향 | 두 종 경계음 원본/EQ 준비, 선택한 버전 정상 재생·종료 확인 |
| 기구 검증 | 제어 | 부호·영점·가동범위·0°/+30° default, q/dq 추종·토크/진동·제동거리 기록 |
| 전체 지연 측정 | 제어/통합 | YOLO 부하에서 제어 틱/CAN 지연, 검출부터 실측 응답까지의 로그 |
| 검출/이탈 검증 | 디텍션 | 몸 돌림·가림·눕기·후퇴·급속 이동의 오검출/미검출·지연 기록 |
| 통합 리허설 | 통합 | 사람 대역 이후 실제 영상/현장, 단계별 조준·재생·stop·유실 동작 일치 |
| 남은 정책 확정 | 팀장/통합 | 보호 구역 정의, 퇴치 성공 판단, 위협음 추가 반복 여부 |

경계음이 없는 현재 저장소만으로는 자동 시나리오가 경계음 대기까지 진행하는 것이 정상이다.
이 상태를 전체 현장 퇴치 검증 완료로 인계하지 않는다.
