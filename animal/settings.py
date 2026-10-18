"""Runtime configuration. Rig coordinates/gains live in motor/config.py."""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "assets/models/weights.pt"
CAMERA_INDEX = 4
ENABLE_MOTOR = False
DRY_RUN = False
HEADLESS = False
AUDIO_PLAYER_COMMAND = None

CLASS_TO_AUDIO = {0: "boar", 1: "water_deer"}
EXPECTED_MODEL_NAMES = {0: "boar", 1: "deer"}
TORSO_CLASS_ID = 2
TORSO_CLASS_NAME = "torso"
MIN_SELECTION_CONFIDENCE = 0.5
MIN_PLAYBACK_CONFIDENCE = 0.5
CONSECUTIVE_FRAMES = 3
TORSO_TRACK_CONFIDENCE = 0.2  # Raw detection threshold; torsos do NOT enter ByteTrack.
TRACKER_CONFIG = "bytetrack.yaml"
TRACK_LOST_SECONDS = 2.0
RESELECT_GRACE_SECONDS = 1.0
CAMERA_TIMEOUT_SECONDS = 2.0
PERCEPTION_TIMEOUT_SECONDS = 2.0
MAX_RESULT_AGE_SECONDS = 1.0

# Provisional exit heuristic. Evaluate only while the motor goal is frozen.
AREA_RATIO_THRESHOLD = 0.50
AREA_BASELINE_SECONDS = 1.0
AREA_SMOOTH_SECONDS = 0.5
AREA_HOLD_SECONDS = 1.0
EDGE_MARGIN_PX = 2
TORSO_WINDOW_SIZE = 5
TORSO_SPEED_THRESHOLD = 0.15  # widths/frame; preserves the existing FPS dependence.

AIM_AVERAGE_FRAMES = 3
AIM_DEADBAND_FRACTION = 0.003
AIM_MAX_JUMP_FRACTION = 0.15  # normalized image distance; isolated jumps rejected
AIM_TOLERANCE_FRACTION = 0.015  # each axis, fraction of full frame
AIM_MAX_VELOCITY_RAD_S = 0.05
AIM_SETTLE_SECONDS = 0.5
WARNING_OBSERVATION_SECONDS = 5.0  # Starts after successful warning playback completes.

# Existing 10 clips/species are THREATS. Original/EQ/pitch choices are preserved.
AUDIO_VARIANT = "original"
AUDIO_VARIANT_DIRS = {
    "original": PROJECT_ROOT / "assets/audio",
    "eq": PROJECT_ROOT / "assets/audio_eq",
    "pitch": PROJECT_ROOT / "assets/audio_pitch",
    "pitch_eq": PROJECT_ROOT / "assets/audio_pitch_eq",
}
AUDIO_DIR = AUDIO_VARIANT_DIRS[AUDIO_VARIANT]
MIN_AUDIO_FILES_PER_CLASS = 10

# Warning assets are pending; no fallback to threat clips.
WARNING_AUDIO_VARIANT = "original"
WARNING_AUDIO_VARIANT_DIRS = {
    "original": PROJECT_ROOT / "assets/warnings",
    "eq": PROJECT_ROOT / "assets/warnings_eq",
}
WARNING_AUDIO_DIR = WARNING_AUDIO_VARIANT_DIRS[WARNING_AUDIO_VARIANT]

PERSON_MODEL_PATH = PROJECT_ROOT / "assets/models/yolov8n-pose.pt"
PERSON_CLASS_TO_AUDIO = {0: "boar"}
PERSON_EXPECTED_MODEL_NAMES = {0: "person"}
PERSON_KEYPOINT_CONFIDENCE = 0.5
