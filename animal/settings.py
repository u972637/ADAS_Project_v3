"""Runtime settings for the wildlife deterrent."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Local wildlife YOLO weights.
MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "weights.pt"
CAMERA_INDEX = 4  # Provisional; verify /dev/video* after connecting the camera.
ENABLE_MOTOR = False
DRY_RUN = False
HEADLESS = False
AUDIO_PLAYER_COMMAND = None  # Defaults to afplay on macOS, aplay elsewhere.

# weights.pt: 0=boar, 1=deer, 2=torso. Only boar/deer trigger tracking and audio.
# Update both mappings when replacing the model.
CLASS_TO_AUDIO = {0: "boar", 1: "water_deer"}
EXPECTED_MODEL_NAMES = {0: "boar", 1: "deer"}
TORSO_CLASS_ID = 2
TORSO_CLASS_NAME = "torso"

MIN_SELECTION_CONFIDENCE = 0.5
MIN_PLAYBACK_CONFIDENCE = 0.5
CONSECUTIVE_FRAMES = 3
RELEASE_FRAMES = 3
COOLDOWN_SECONDS = 10.0

# 음원은 <AUDIO_DIR>/<CLASS_TO_AUDIO 값>/*.wav 를 그대로 읽는다 — 파일을 넣고
# 빼기만 하면 되고 목록 파일은 없다.
# AUDIO_VARIANT 로 재생할 음원 버전을 고른다 — main.py, start.sh 1·2번,
# tools/play_audio.py 전부 이 값을 따름:
#   "original" : 원본
#   "eq"       : 1.5~2kHz 강조 EQ (tools/eq_filter.py 가 생성)
#   "pitch"    : 1.5~2kHz 쪽으로 피치를 올림 (tools/pitch_shift.py 가 생성)
#   "pitch_eq" : 피치를 올린 뒤 EQ 까지 (tools/pitch_shift.py 가 생성)
AUDIO_VARIANT = "original"
AUDIO_VARIANT_DIRS = {
    "original": PROJECT_ROOT / "assets/audio",
    "eq": PROJECT_ROOT / "assets/audio_eq",
    "pitch": PROJECT_ROOT / "assets/audio_pitch",
    "pitch_eq": PROJECT_ROOT / "assets/audio_pitch_eq",
}
AUDIO_DIR = AUDIO_VARIANT_DIRS[AUDIO_VARIANT]
MIN_AUDIO_FILES_PER_CLASS = 10

# ──────────────────────────────────────────────
# 도주(escape) 감지 기반 타겟 고정 추적
# ──────────────────────────────────────────────
# 매 프레임 신뢰도 최고 탐지를 그때그때 조준하는 대신, model.track()의 한
# track ID를 "lock"해서 계속 같은 개체만 조준하다가, 아래 두 조건 중 하나
# 라도 걸리면 "도주"로 보고 lock을 풀고(=더 이상 조준 안 함) 새 타겟을
# 다시 고른다. animal/wildlife.py의 EscapeTracker가 이 상수들을 쓴다.
# (원본 로직: animal/animal_escape_tracking_combined.py, 오프라인 영상
# 분석용으로 작성됐던 걸 라이브 파이프라인에 맞게 옮긴 것)
TORSO_TRACK_CONFIDENCE = 0.2   # model.track() 이후 "torso"로 인정할 최소 신뢰도
TRACKER_CONFIG = "bytetrack.yaml"

TRACK_LOST_SECONDS = 2.0        # 이 시간(초) 동안 lock된 ID가 안 보이면 lock 해제
                                 # (AI 탐지가 몇 프레임 빠지는 정도는 이 안에서
                                 # 버틴다 — Tracker 가 마지막 목표 위치를 계속
                                 # 토크로 유지하므로 별도 "마지막 좌표 기억" 코드는
                                 # 필요 없음. 2초를 넘기면 실제로 놓친 것으로 보고
                                 # lock 해제 — 테스트 메뉴에서는 r 을 누른 것과 동일)
RESELECT_GRACE_SECONDS = 1.0    # 도주로 lock 해제된 뒤 새 타겟을 고르기까지 대기(초)

# 값은 3D 시뮬레이션 튜닝(무작위 상황 540개) 결과 — 기존 값은 주석의 "원본".
#
# 조건 1: bbox 크기가 기준치보다 많이 줄면(= 뒤로 물러나는 것으로 추정) 도주로 판단
#   면적만 보면 몸을 돌리기만 해도(옆모습 -> 앞·뒷모습) 면적이 절반 이하로 줄어
#   잘못 종료된다(시뮬레이션에서 돌아다니기의 최대 27%). bbox 높이는 몸을 돌려도
#   거의 그대로라서, 면적 AND 높이가 둘 다 줄어야 도주로 본다.
DISTANCE_METRIC = "and"         # "and"(면적·높이 둘 다) | "area"(원본) | "height"
AREA_RATIO_THRESHOLD = 0.50     # 기준 면적 대비 이 비율 이하 (같은 방향이면 거리 약 1.41배)
HEIGHT_RATIO_THRESHOLD = 0.80   # 기준 높이 대비 이 비율 이하 (거리 약 1.25배)
AREA_BASELINE_SECONDS = 0.4     # 새로 lock된 뒤 기준 크기를 모으는 시간(초) (원본 1.0)
AREA_SMOOTH_SECONDS = 0.35      # 크기 스무딩(중앙값) 윈도우(초) (원본 0.5)
AREA_HOLD_SECONDS = 0.5         # 조건이 이 시간(초) 이상 유지돼야 확정 (원본 1.0)
EDGE_MARGIN_PX = 2              # 화면 가장자리에 걸친(잘린) bbox는 샘플에서 제외

# 조건 2: torso 중심의 급속 이동(박스 너비로 정규화)도 도주로 판단
#   속도 = 이동 거리 / (경과 프레임 수 x 평균 박스 너비). 경과 프레임 수는 실제
#   경과 시간 x TORSO_NOMINAL_FPS 로 계산해서, 처리 속도가 25fps보다 느려져도
#   같은 실제 속도 기준이 유지된다(25fps에서는 원본과 동일).
TORSO_WINDOW_SIZE = 5
TORSO_SPEED_THRESHOLD = 0.10    # 25fps 기준 프레임당 박스 너비의 0.10배 = 초당 2.5배 (원본 0.15)
TORSO_NOMINAL_FPS = 25.0

# ──────────────────────────────────────────────
# 사람 대역(person proxy) 테스트 — start.sh 2번
# ──────────────────────────────────────────────
# 실제 멧돼지/고라니 없이 위 EscapeTracker(타겟 ID 고정·도주감지·torso 조준)
# 를 시험하려고, 사람을 동물 한 종처럼 취급해 같은 파이프라인에 넣는다.
# 동물 모델(weights.pt)엔 torso 클래스가 있지만 COCO YOLO 엔 없으므로, pose
# 모델의 어깨·골반 키포인트로 torso 박스를 만들어 대신 쓴다
# (animal/person_proxy.py). 도주 판정 임계값·재생 정책 등 나머지 값은 위
# 동물 설정을 그대로 공유한다.
PERSON_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "yolov8n-pose.pt"
PERSON_CLASS_TO_AUDIO = {0: "boar"}          # 사람이 잡히면 멧돼지 WAV 재생
PERSON_EXPECTED_MODEL_NAMES = {0: "person"}
PERSON_KEYPOINT_CONFIDENCE = 0.5             # torso 를 만들 어깨·골반 키포인트 최소 신뢰도
