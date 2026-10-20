"""Runtime configuration. Rig coordinates/gains live in motor/config.py."""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "assets/models/weights.pt"
CAMERA_INDEX = 4
ENABLE_MOTOR = True  # Production main.py runs detection, control and audio together.
START_PAUSED = False  # main.py --start-paused enables W/resume gating when needed.
DRY_RUN = False
# False: Tracker's 500 Hz control thread shares this process (original).
# True: the control loop runs in a separate OS process (motor/ipc_tracker.py),
# immune to this process's own GIL load. Flip here to switch, no env var needed.
MOTOR_PROCESS = True
HEADLESS = False
AUDIO_PLAYER_COMMAND = None
# Escape/lost termination while tracking: return to the shared default pose
# (motor/config.py DEFAULT_YAW_DEG/DEFAULT_PITCH_DEG) but much slower than a
# normal stop, so the animal isn't startled by a fast motion right after it
# was detected backing away or lost.
TERMINATION_RETURN_SECONDS = 8.0
# Program exit (q/quit) returns here instead of the shared default pose.
QUIT_YAW_DEG = 0.0
QUIT_PITCH_DEG = -4.0

CLASS_TO_AUDIO = {0: "boar", 1: "water_deer"}
EXPECTED_MODEL_NAMES = {0: "boar", 1: "deer"}
TORSO_CLASS_ID = 2
TORSO_CLASS_NAME = "torso"
MIN_SELECTION_CONFIDENCE = 0.5
TORSO_TRACK_CONFIDENCE = 0.3  # Raw detection threshold; torsos do NOT enter ByteTrack.
TRACKER_CONFIG = "bytetrack.yaml"
TRACK_LOST_SECONDS = 2.0
RESELECT_GRACE_SECONDS = 1.0
CAMERA_TIMEOUT_SECONDS = 2.0
PERCEPTION_TIMEOUT_SECONDS = 2.0
MAX_RESULT_AGE_SECONDS = 1.0

# Exit heuristic: bbox area/height shrinking relative to a collected baseline.
DISTANCE_METRIC = "and"  # area AND height; alternatives: "area", "height"
AREA_RATIO_THRESHOLD = 0.50
HEIGHT_RATIO_THRESHOLD = 0.80
AREA_BASELINE_SECONDS = 0.4
AREA_SMOOTH_SECONDS = 0.35
AREA_HOLD_SECONDS = 0.5
EDGE_MARGIN_PX = 2
# Absolute floor on the animal bbox's pixel height, independent of the
# baseline-ratio check above: too far away to track reliably regardless of
# how large it started out. 0 disables this check.
MIN_BBOX_HEIGHT_PX = 100.0

AIM_AVERAGE_FRAMES = 3
AIM_DEADBAND_FRACTION = 0.003
AIM_MAX_JUMP_FRACTION = 0.15  # normalized image distance; isolated jumps rejected

# Playback starts once the aimed (matched_torso) point is within
# PLAYBACK_ENTER_FRACTION of screen center -- evaluated purely on the red-dot
# (motor-signal) aim point, independent of whether the animal bbox itself
# (`target`) is live this frame. Once playing, it keeps going (motor keeps
# tracking regardless) until the aim point drifts past PLAYBACK_EXIT_FRACTION,
# and only while the animal bbox is actually live (see wildlife.py).
PLAYBACK_ENTER_FRACTION = 0.05
PLAYBACK_EXIT_FRACTION = 0.15
PLAYBACK_GAP_SECONDS = 0.3  # Silence between consecutive clips while playing.

# Only the pitch_eq variant is kept on disk; the old single-stage "threat"
# clips (assets/audio*) and the other warning variants (original/eq/pitch)
# were removed since nothing in the running code referenced them anymore.
WARNING_AUDIO_DIR = PROJECT_ROOT / "assets/warnings_pitch_eq"
