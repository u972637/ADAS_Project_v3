"""Shared rig calibration. Public angles: right/up positive, radians.

Motor encoder/command angles have the opposite sign on BOTH axes.
Gains, effective FOV and feed-forward retain the latest hardware tuning.
"""

import math

AXES = ("yaw", "pitch")
DEVICENAME = "/dev/ttyACM0"
BAUDRATE = 921600
CAN_ID_YAW, CAN_ID_PITCH = 0x01, 0x02
MASTER_ID_YAW, MASTER_ID_PITCH = 0x11, 0x12
FRAME_W, FRAME_H = 2592, 1944
CENTER_X, CENTER_Y = FRAME_W / 2, FRAME_H / 2
HFOV_DEG, VFOV_DEG = 28.0, 20.0
RAD_PER_PIXEL_X = math.radians(HFOV_DEG) / FRAME_W
RAD_PER_PIXEL_Y = math.radians(VFOV_DEG) / FRAME_H
MOTOR_SIGN = {"yaw": -1.0, "pitch": -1.0}
YAW_RANGE_RAD, PITCH_RANGE_RAD = 1.2, 0.6
RANGE_RAD = {"yaw": YAW_RANGE_RAD, "pitch": PITCH_RANGE_RAD}
KP_YAW = KP_PITCH = 30.0
KD_YAW = KD_PITCH = 1.8
KP = {"yaw": KP_YAW, "pitch": KP_PITCH}
KD = {"yaw": KD_YAW, "pitch": KD_PITCH}
# Raw motor torque calibration; do not invert this with the angle convention.
TAU_FF_YAW, TAU_FF_PITCH = 0.0, 2.3
RAW_TAU_FF = {"yaw": TAU_FF_YAW, "pitch": TAU_FF_PITCH}
CONTROL_FREQ = 500
# Initial commissioning limits, deliberately above the 0.2 rad/s settle test.
# These bound reference motion, not measured physical acceleration or torque.
MOTION_MAX_VELOCITY = {"yaw": 1.5, "pitch": 1.2}  # rad/s
MOTION_MAX_ACCELERATION = {"yaw": 8.0, "pitch": 6.0}  # rad/s²
MOTION_MAX_JERK = {"yaw": 100.0, "pitch": 80.0}  # rad/s³
MAX_CONTROL_GAP_SECONDS = 0.040  # Fault instead of jumping through a long stall.
MOTION_FINISH_TIMEOUT_SECONDS = 5.0
MAX_STEP_RAD = 0.6
DEADZONE_PX = 8.0
# Shared operating default pose, in logical coordinates (not encoder zero).
# stop/normal exit call Tracker.go_default(); change the pose here only.
DEFAULT_YAW_DEG, DEFAULT_PITCH_DEG = 0.0, 30.0
GO_CENTER_DURATION = 2.0  # Minimum return duration; limits may require longer.
# J4310 V1.0 docs list faults 0x8..0xE but do not define the normal-state nibble.
# Accept 0/1 for firmware compatibility; report the raw value in health().
# On firmware confirmed to report 0=disabled / 1=enabled, set this to (1,).
HEALTHY_STATUS_CODES = (0, 1)
FEEDBACK_TIMEOUT_SECONDS = 0.5
STARTUP_TIMEOUT_SECONDS = 2.0


def to_motor(axis: str, logical: float) -> float:
    return MOTOR_SIGN[axis] * logical


def from_motor(axis: str, raw: float) -> float:
    return raw / MOTOR_SIGN[axis]


def clamp_angle(axis: str, angle: float) -> float:
    if not math.isfinite(angle):
        raise ValueError("motor angle must be finite")
    return max(-RANGE_RAD[axis], min(RANGE_RAD[axis], angle))
