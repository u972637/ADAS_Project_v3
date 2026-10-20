"""Runtime configuration. Rig coordinates/gains live in motor/config.py."""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "assets/models/weights.pt"
# None = use the model's own training imgsz (currently 1088, baked into
# weights.pt). Raising this (e.g. 1280/1600, must stay a multiple of the
# model stride, 32) trades inference latency for more pixels on small/far
# detections; whether it actually helps is empirical, not guessed here --
# change it and compare, don't assume bigger is better.
INFERENCE_IMGSZ = 1088 # (32의 배수: …1088, 1152, 1216, 1280, 1344…)
LOG_DIR = PROJECT_ROOT / "log"
# Menu 1/2 'u' key: append the current red-dot torso's pixel width/height here.
TORSO_SIZE_LOG_PATH = LOG_DIR / "torso_size.csv"
# Camera = /dev/video<CAMERA_INDEX>. The number is assigned in USB enumeration
# order, so it moves when the camera is replugged or another camera (the
# laptop's built-in webcam exposes /dev/video2..5) enumerates first -- it was
# 0 at times and 4 at others. If the picture is black or the wrong camera, run
#   ls -l /dev/v4l/by-id/        (the Arducam "-video-index0" -> ../../videoN)
# and set CAMERA_INDEX to that N.
CAMERA_INDEX = 4
# Optional: a /dev/v4l/by-id/...-video-index0 path (never "-video-index1", the
# metadata node) pins the camera independent of its number. Overrides
# CAMERA_INDEX when set; None = use CAMERA_INDEX.
CAMERA_DEVICE = None
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
MIN_SELECTION_CONFIDENCE = 0.6
TORSO_TRACK_CONFIDENCE = 0.2  # Raw detection threshold; torsos do NOT enter ByteTrack.
TRACKER_CONFIG = "bytetrack.yaml"
TRACK_LOST_SECONDS = 3.0
# Once the locked individual's own bbox (not just its torso) has been
# missing this long, ID-switch reidentify stops requiring the candidate to
# be near the last torso position -- it searches the whole frame for any
# same-species, bbox-confirmed torso, biggest wins. Below this, reidentify
# stays close to the last known spot (quick same-individual recovery).
BBOX_RETARGET_GRACE_SECONDS = 2.0
# While no torso is matched (orange dot), the held aim point resent to the
# motor is clamped to within this fraction of screen center, per axis.
# The motor has already been steering that point toward center, and every
# update() is relative to the motor's CURRENT position, so resending a stale
# off-center pixel offset would keep pushing it further each tick.
HELD_AIM_MAX_OFFSET_FRACTION = 0.05
RESELECT_GRACE_SECONDS = 1.0
CAMERA_TIMEOUT_SECONDS = 5.0
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
MIN_BBOX_HEIGHT_PX = 20.0
# Position-based torso bridge (bbox-gap fallback / ByteTrack ID-switch
# reidentify) searches within this many torso-WIDTHS of the last confirmed
# torso position. 1.0 only tolerates near-zero movement during the gap; a
# real animal can easily drift more than its own body-width during even a
# short (sub-second) detection gap, which otherwise leaves it stuck
# unmatched (yellow, never red) once it reappears until the full
# TRACK_LOST_SECONDS timeout releases and re-selects from scratch.
TORSO_BRIDGE_RADIUS_MULTIPLIER = 1.8

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
