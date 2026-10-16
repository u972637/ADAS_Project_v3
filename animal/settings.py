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

AUDIO_MANIFEST = PROJECT_ROOT / "assets/audio/manifest.json"
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

# 조건 1: bbox 면적이 기준치보다 많이 줄면(= 뒤로 물러나는 것으로 추정) 도주로 판단
AREA_RATIO_THRESHOLD = 0.50     # 기준 면적 대비 이 비율 이하로 줄면
AREA_BASELINE_SECONDS = 1.0     # 새로 lock된 뒤 기준 면적을 모으는 시간(초)
AREA_SMOOTH_SECONDS = 0.5       # 면적 스무딩(중앙값) 윈도우(초)
AREA_HOLD_SECONDS = 1.0         # 비율이 이 시간(초) 이상 유지돼야 확정
EDGE_MARGIN_PX = 2              # 화면 가장자리에 걸친(잘린) bbox는 샘플에서 제외

# 조건 2: torso 중심의 급속 이동(박스 너비로 정규화)도 도주로 판단
TORSO_WINDOW_SIZE = 5
TORSO_SPEED_THRESHOLD = 0.15
