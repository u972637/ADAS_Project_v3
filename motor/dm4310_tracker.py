"""DM4310 control and feedback diagnostics; all public angles are right/up +.

Only this adapter converts logical coordinates to motor encoder coordinates.
The 500 Hz PD stream continues at a fixed goal during audio/observation.
"""

from __future__ import annotations

import logging
import math
import threading
import time

import serial

from motor.DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type
from motor.config import (
    AXES as AXES,
    DEVICENAME as DEVICENAME,
    BAUDRATE as BAUDRATE,
    CAN_ID_YAW as CAN_ID_YAW,
    CAN_ID_PITCH as CAN_ID_PITCH,
    MASTER_ID_YAW as MASTER_ID_YAW,
    MASTER_ID_PITCH as MASTER_ID_PITCH,
    FRAME_W as FRAME_W,
    FRAME_H as FRAME_H,
    CENTER_X as CENTER_X,
    CENTER_Y as CENTER_Y,
    HFOV_DEG as HFOV_DEG,
    VFOV_DEG as VFOV_DEG,
    RAD_PER_PIXEL_X as RAD_PER_PIXEL_X,
    RAD_PER_PIXEL_Y as RAD_PER_PIXEL_Y,
    MOTOR_SIGN as MOTOR_SIGN,
    YAW_RANGE_RAD as YAW_RANGE_RAD,
    PITCH_RANGE_RAD as PITCH_RANGE_RAD,
    RANGE_RAD as RANGE_RAD,
    KP_YAW as KP_YAW,
    KP_PITCH as KP_PITCH,
    KD_YAW as KD_YAW,
    KD_PITCH as KD_PITCH,
    KP as KP,
    KD as KD,
    TAU_FF_YAW as TAU_FF_YAW,
    TAU_FF_PITCH as TAU_FF_PITCH,
    RAW_TAU_FF as RAW_TAU_FF,
    CONTROL_FREQ as CONTROL_FREQ,
    MAX_STEP_RAD as MAX_STEP_RAD,
    DEADZONE_PX as DEADZONE_PX,
    DEFAULT_YAW_DEG as DEFAULT_YAW_DEG,
    DEFAULT_PITCH_DEG as DEFAULT_PITCH_DEG,
    GO_CENTER_DURATION as GO_CENTER_DURATION,
    HEALTHY_STATUS_CODES as HEALTHY_STATUS_CODES,
    FEEDBACK_TIMEOUT_SECONDS as FEEDBACK_TIMEOUT_SECONDS,
    STARTUP_TIMEOUT_SECONDS as STARTUP_TIMEOUT_SECONDS,
    to_motor as to_motor,
    from_motor as from_motor,
    clamp_angle as clamp_angle,
)

logger = logging.getLogger(__name__)
MOTOR_TYPE = DM_Motor_Type.DM4310


class MotorFault(RuntimeError):
    pass


class FeedbackMotor(Motor):
    """Timestamp actual feedback, not requests or cached default positions."""

    def __init__(self, *args):
        super().__init__(*args)
        self._feedback_lock = threading.Lock()
        self._feedback = None

    def recv_data(self, q, dq, tau, err=0):
        super().recv_data(q, dq, tau, err)
        with self._feedback_lock:
            self._feedback = (time.monotonic(), q, dq, tau, err)

    def snapshot(self):
        with self._feedback_lock:
            return self._feedback


def checked_feedback(axis, sample, now):
    if sample is None:
        raise MotorFault(f"{axis}: no feedback")
    received_at, q, dq, torque, status = sample
    if now - received_at > FEEDBACK_TIMEOUT_SECONDS:
        raise MotorFault(f"{axis}: stale feedback ({now - received_at:.3f}s)")
    # Firmware-specific normal states are configured separately from fault codes.
    if status not in HEALTHY_STATUS_CODES:
        raise MotorFault(f"{axis}: motor status 0x{status:X}")
    if not all(math.isfinite(v) for v in (q, dq, torque)):
        raise MotorFault(f"{axis}: non-finite feedback")
    return tuple(from_motor(axis, v) for v in (q, dq, torque))


class Tracker:
    def __init__(self, devicename=DEVICENAME, baudrate=BAUDRATE):
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._serial = self._ctrl = None
        self._closed = False
        self._fault = None
        self._motor = {}
        self._ramp = None
        self._actual_freq = self._update_actual_freq = 0.0
        self._update_count = 0
        self._update_window_t0 = time.monotonic()
        try:
            self._serial = serial.Serial(
                devicename, baudrate, timeout=0.001, write_timeout=0.1
            )
            self._ctrl = MotorControl(self._serial)
            for axis, can_id, master in zip(
                AXES, (CAN_ID_YAW, CAN_ID_PITCH), (MASTER_ID_YAW, MASTER_ID_PITCH)
            ):
                motor = FeedbackMotor(MOTOR_TYPE, can_id, master)
                self._motor[axis] = motor
                self._ctrl.addMotor(motor)
                if not self._ctrl.switchControlMode(motor, Control_Type.MIT):
                    raise MotorFault(f"{axis}: MIT mode not confirmed")
            for motor in self._motor.values():
                self._ctrl.enable(motor)
            self._current = self._wait_for_feedback()
            for axis in AXES:
                if abs(self._current[axis][0]) > RANGE_RAD[axis]:
                    raise MotorFault(f"{axis}: startup angle outside configured range")
            self._target = {axis: self._current[axis][0] for axis in AXES}
            self._thread = threading.Thread(target=self._control_thread, daemon=True)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def _wait_for_feedback(self):
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        last_error = None
        while time.monotonic() < deadline:
            for motor in self._motor.values():
                self._ctrl.refresh_motor_status(motor)
            try:
                return {
                    axis: checked_feedback(axis, motor.snapshot(), time.monotonic())
                    for axis, motor in self._motor.items()
                }
            except MotorFault as error:
                last_error = error
                time.sleep(0.01)
        raise MotorFault(f"startup feedback timeout: {last_error}")

    def _disable(self):
        if self._ctrl is not None:
            for axis, motor in self._motor.items():
                try:
                    self._ctrl.disable(motor)
                except Exception:
                    logger.exception("%s: failed to send disable", axis)

    def _control_thread(self):
        next_t = window_t0 = time.monotonic()
        ticks = 0
        try:
            while not self._stop_event.is_set():
                now = time.monotonic()
                current = {
                    axis: checked_feedback(axis, m.snapshot(), now)
                    for axis, m in self._motor.items()
                }
                with self._lock:
                    self._current = current
                    if self._ramp is not None:
                        r = self._ramp
                        fraction = min((now - r["t0"]) / r["duration"], 1.0)
                        self._target = {
                            a: r["start"][a] + fraction * (r["goal"][a] - r["start"][a])
                            for a in AXES
                        }
                        if fraction >= 1:
                            self._ramp = None
                    goals = dict(self._target)
                for axis in AXES:
                    self._ctrl.controlMIT(
                        self._motor[axis],
                        KP[axis],
                        KD[axis],
                        to_motor(axis, clamp_angle(axis, goals[axis])),
                        0.0,
                        RAW_TAU_FF[axis],
                    )
                ticks += 1
                if now - window_t0 >= 1:
                    self._actual_freq = ticks / (now - window_t0)
                    ticks, window_t0 = 0, now
                next_t += 1.0 / CONTROL_FREQ
                self._stop_event.wait(max(0, next_t - time.monotonic()))
                next_t = max(next_t, time.monotonic())
        except Exception as error:
            self._fault = f"{type(error).__name__}: {error}"
            logger.exception("motor control fault")
            self._stop_event.set()
            self._disable()

    def health(self):
        now = time.monotonic()
        detail = {a: m.snapshot() for a, m in self._motor.items()}
        fault = self._fault
        if not fault:
            try:
                for a, sample in detail.items():
                    checked_feedback(a, sample, now)
                if self._thread is None or not self._thread.is_alive():
                    raise MotorFault("control thread is not running")
            except MotorFault as error:
                fault = str(error)
        return {
            "ok": not fault,
            "fault": fault,
            "feedback_age": {
                a: None if s is None else now - s[0] for a, s in detail.items()
            },
            "status": {a: None if s is None else s[4] for a, s in detail.items()},
        }

    def assert_healthy(self):
        health = self.health()
        if not health["ok"]:
            raise MotorFault(health["fault"])

    def update(self, target_x, target_y):
        if target_x is None or target_y is None:
            return
        self.assert_healthy()
        if not all(math.isfinite(v) for v in (target_x, target_y)):
            raise ValueError("target coordinates must be finite")
        err_x, err_y = target_x - CENTER_X, CENTER_Y - target_y
        if abs(err_x) <= DEADZONE_PX:
            err_x = 0.0
        if abs(err_y) <= DEADZONE_PX:
            err_y = 0.0
        steps = {"yaw": err_x * RAD_PER_PIXEL_X, "pitch": err_y * RAD_PER_PIXEL_Y}
        with self._lock:
            self._ramp = None
            # Even at zero image error replace the previous goal with current feedback.
            for axis in AXES:
                step = max(-MAX_STEP_RAD, min(MAX_STEP_RAD, steps[axis]))
                self._target[axis] = clamp_angle(axis, self._current[axis][0] + step)
            self._update_count += 1
            now = time.monotonic()
            if now - self._update_window_t0 >= 1:
                self._update_actual_freq = self._update_count / (
                    now - self._update_window_t0
                )
                self._update_count, self._update_window_t0 = 0, now

    def hold(self):
        self.assert_healthy()
        with self._lock:
            self._ramp = None
            self._target = {a: self._current[a][0] for a in AXES}

    def go_to(self, yaw_rad=0.0, pitch_rad=0.0, duration=GO_CENTER_DURATION):
        self.assert_healthy()
        if not math.isfinite(duration) or duration < 0:
            raise ValueError("duration must be finite and nonnegative")
        goal = {
            "yaw": clamp_angle("yaw", yaw_rad),
            "pitch": clamp_angle("pitch", pitch_rad),
        }
        with self._lock:
            if duration == 0:
                self._target, self._ramp = goal, None
            else:
                self._ramp = dict(
                    t0=time.monotonic(),
                    duration=duration,
                    start={a: self._current[a][0] for a in AXES},
                    goal=goal,
                )

    def go_center(self):
        self.go_to()

    def go_default(self):
        self.go_to(math.radians(DEFAULT_YAW_DEG), math.radians(DEFAULT_PITCH_DEG))

    def get_state(self, axis):
        with self._lock:
            return self._current[axis]

    def get_yaw_angle_rad(self):
        return self.get_state("yaw")[0]

    def get_actual_freq(self):
        return self._actual_freq

    def get_can_tx_freq(self):
        return self._ctrl.get_tx_freq()

    def get_rx_freq(self, axis):
        return self._motor[axis].get_rx_freq()

    def get_update_freq(self):
        return self._update_actual_freq

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._disable()
        if self._serial is not None:
            self._serial.close()
