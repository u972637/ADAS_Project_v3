"""Runtime settings for the wildlife deterrent."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent

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

MIN_SELECTION_CONFIDENCE = 0.5
MIN_PLAYBACK_CONFIDENCE = 0.5
CONSECUTIVE_FRAMES = 3
RELEASE_FRAMES = 3
COOLDOWN_SECONDS = 10.0

AUDIO_MANIFEST = PROJECT_ROOT / "assets/audio/manifest.json"
MIN_AUDIO_FILES_PER_CLASS = 10
