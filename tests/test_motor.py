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

    def test_hold_and_default_ramp(self):
        t = self.tracker()
        t.go_to(0.2, 0.3)
        t.hold()
        self.assertIsNone(t._ramp)
        self.assertEqual(t._target, {"yaw": 0.1, "pitch": 0.1})
        t.go_default()
        self.assertAlmostEqual(t._ramp["goal"]["pitch"], math.radians(30))

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
        sent = []

        def send(*args):
            sent.append(args)
            if len(sent) == 2:
                t._stop_event.set()

        with patch.object(t._ctrl, "controlMIT", side_effect=send):
            t._control_thread()
        self.assertAlmostEqual(sent[0][3], -0.2)
        self.assertAlmostEqual(sent[1][3], -0.3)
        self.assertEqual(sent[1][5], 2.3)

    def test_nan_goal_and_negative_duration_rejected(self):
        t = self.tracker()
        with self.assertRaises(ValueError):
            t.go_to(float("nan"), 0)
        with self.assertRaises(ValueError):
            t.go_to(0, 0, duration=-1)
