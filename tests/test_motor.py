import math
import time
import unittest
from unittest.mock import patch

from motor import dm4310_tracker as module
from motor.dm4310_tracker import Tracker, MotorFault, checked_feedback
from motor.config import (
    AXES,
    CENTER_X,
    CENTER_Y,
    HEALTHY_STATUS_CODES,
    to_motor,
    from_motor,
)


class Serial:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def close(self):
        self.closed = True


class Control:
    mode = True
    feedback = True

    def __init__(self, serial):
        self.serial = serial
        self.motors = []
        self.sent = []
        self.disabled = []

    def addMotor(self, motor):
        self.motors.append(motor)

    def switchControlMode(self, *args):
        return self.mode

    def enable(self, motor):
        pass

    def refresh_motor_status(self, motor):
        if self.feedback:
            motor.recv_data(-0.1, 0, 0, 1)

    def controlMIT(self, *args):
        self.sent.append(args)
        args[0].recv_data(args[3], 0, 0, 1)

    def disable(self, motor):
        self.disabled.append(motor)

    def get_tx_freq(self):
        return 0


class Thread:
    def __init__(self, **kwargs):
        self.alive = False

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def join(self, **kwargs):
        self.alive = False


class MotorTests(unittest.TestCase):
    def setUp(self):
        self.patches = [
            patch.object(module.serial, "Serial", Serial),
            patch.object(module, "MotorControl", Control),
            patch.object(module.threading, "Thread", Thread),
        ]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)

    def tracker(self):
        t = Tracker()
        self.addCleanup(t.close)
        return t

    def test_logical_angles_and_feedback_both_axes_invert(self):
        t = self.tracker()
        self.assertAlmostEqual(t.get_state("yaw")[0], 0.1)
        self.assertAlmostEqual(t.get_state("pitch")[0], 0.1)
        for axis in AXES:
            self.assertAlmostEqual(to_motor(axis, math.radians(30)), -math.radians(30))
            self.assertAlmostEqual(from_motor(axis, -0.3), 0.3)

    def test_right_up_image_errors_make_positive_logical_goals(self):
        t = self.tracker()
        t.update(CENTER_X + 200, CENTER_Y - 200)
        self.assertGreater(t._target["yaw"], 0.1)
        self.assertGreater(t._target["pitch"], 0.1)
        self.assertLess(to_motor("yaw", t._target["yaw"]), -0.1)
        self.assertLess(to_motor("pitch", t._target["pitch"]), -0.1)

    def test_centered_target_cancels_previous_off_center_goal(self):
        t = self.tracker()
        t.update(CENTER_X + 200, CENTER_Y - 200)
        t.update(CENTER_X, CENTER_Y)
        self.assertEqual(t._target, {"yaw": 0.1, "pitch": 0.1})

    def test_hold_brakes_and_default_uses_minimum_duration(self):
        t = self.tracker()
        t.go_to(0.2, 0.3)
        t.hold()
        self.assertIsNone(t._command.position)
        self.assertEqual(t._target, {"yaw": 0.1, "pitch": 0.1})
        t.go_default()
        self.assertAlmostEqual(t._target["pitch"], math.radians(30))
        self.assertEqual(t._command.minimum_duration, 2.0)

    def test_motion_complete_waits_for_both_axis_packets(self):
        t = self.tracker()
        original = t._ctrl.controlMIT

        def send(*args):
            self.assertFalse(t.motion_complete())
            original(*args)
            if len(t._ctrl.sent) == 2:
                t._stop_event.set()

        t._ctrl.controlMIT = send
        t._control_thread()
        self.assertEqual(len(t._ctrl.sent), 2)
        self.assertTrue(t.motion_complete())

    def test_default_return_uses_configured_pose_not_hardcoded_angles(self):
        t = self.tracker()
        with (
            patch.object(module, "DEFAULT_YAW_DEG", 5),
            patch.object(module, "DEFAULT_PITCH_DEG", 20),
        ):
            t.go_default()
        self.assertAlmostEqual(t._target["yaw"], math.radians(5))
        self.assertAlmostEqual(t._target["pitch"], math.radians(20))

    def test_mode_not_confirmed_fails_startup(self):
        with patch.object(Control, "mode", False):
            with self.assertRaisesRegex(MotorFault, "MIT mode"):
                Tracker()

    def test_no_feedback_fails_startup(self):
        with (
            patch.object(Control, "feedback", False),
            patch.object(module, "STARTUP_TIMEOUT_SECONDS", 0.02),
        ):
            with self.assertRaisesRegex(MotorFault, "no feedback"):
                Tracker()

    def test_stale_and_fault_feedback_rejected_normal_status_codes_accepted(self):
        now = time.monotonic()
        for sample in (None, (now - 1, 0, 0, 0, 1), (now, 0, 0, 0, 0xB)):
            with self.assertRaises(MotorFault):
                checked_feedback("yaw", sample, now)
        for status in HEALTHY_STATUS_CODES:
            self.assertEqual(
                checked_feedback("yaw", (now, -0.2, -0.3, -0.4, status), now),
                (0.2, 0.3, 0.4),
            )

    def test_control_exception_latches_fault_and_disables_both_axes(self):
        t = self.tracker()
        with patch.object(
            t._ctrl, "controlMIT", side_effect=OSError("injected CAN failure")
        ):
            with self.assertLogs(module.logger, level="ERROR"):
                t._control_thread()
        self.assertFalse(t.health()["ok"])
        self.assertIn("injected CAN failure", t.health()["fault"])
        self.assertEqual(len(t._ctrl.disabled), 2)

    def test_stream_converts_goal_once_and_preserves_raw_feedforward(self):
        t = self.tracker()
        t.go_to(0.2, 0.3, duration=0)
        now = time.monotonic()
        initial, _, _ = t._next_reference(now)
        self.assertEqual(initial["yaw"][0], 0.1)  # No instantaneous position step.
        sent = []

        def send(*args):
            sent.append(args)
            if len(sent) == 2:
                t._stop_event.set()

        with (
            patch.object(t._ctrl, "controlMIT", side_effect=send),
            patch.object(module.time, "monotonic", return_value=now + 0.02),
        ):
            t._control_thread()
        self.assertLess(sent[0][3], -0.1)
        self.assertGreater(sent[0][3], -0.2)
        self.assertLess(sent[1][3], -0.1)
        self.assertGreater(sent[1][3], -0.3)
        self.assertLess(sent[0][4], 0)  # dq_ref is converted exactly once too.
        self.assertLess(sent[1][4], 0)
        self.assertEqual(sent[1][5], 2.3)

    def test_latest_destination_replaces_pending_frames(self):
        t = self.tracker()
        for x in (CENTER_X + 100, CENTER_X - 150, CENTER_X + 200.5):
            t.update(x, CENTER_Y)
        self.assertEqual(t._planner.replans, 0)
        t._next_reference(time.monotonic())
        self.assertEqual(t._planner.replans, 1)
        self.assertEqual(t._command.position, tuple(t._target[a] for a in AXES))

    def test_overdue_motor_io_latches_fault(self):
        t = self.tracker()
        start = time.monotonic()
        clock = [start]

        def send(*args):
            clock[0] += 0.025  # 50 ms for two axes, with no actual sleeping.

        with (
            patch.object(module.time, "monotonic", side_effect=lambda: clock[0]),
            patch.object(t._ctrl, "controlMIT", side_effect=send),
            self.assertLogs(module.logger, level="ERROR"),
        ):
            t._control_thread()
        self.assertFalse(t.health()["ok"])
        self.assertEqual(len(t._ctrl.disabled), 2)
        self.assertGreater(t.health()["motion"]["max_cycle_work_ms"], 40)

    def test_nan_goal_and_negative_duration_rejected(self):
        t = self.tracker()
        with self.assertRaises(ValueError):
            t.go_to(float("nan"), 0)
        with self.assertRaises(ValueError):
            t.go_to(0, 0, duration=-1)
