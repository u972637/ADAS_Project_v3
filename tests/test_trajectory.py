import math
import random
import unittest

from motor.config import (
    AXES,
    MOTION_MAX_VELOCITY,
    MOTION_MAX_ACCELERATION,
    MOTION_MAX_JERK,
)
from motor.trajectory import MotionCommand, MotionError, MotionPlanner


class TrajectoryTests(unittest.TestCase):
    def assert_continuous(self, before, after):
        for old, new in zip(before, after):
            for x, y in zip(old, new):
                self.assertAlmostEqual(x, y, places=9)

    def test_replan_preserves_nonzero_position_velocity_acceleration(self):
        planner = MotionPlanner((0, 0))
        goal = MotionCommand((0.7, 0.4))
        planner.step(0, goal)
        before = planner.step(0.137, goal)
        self.assertGreater(abs(before[1][0]), 0)
        self.assertGreater(abs(before[2][0]), 0)
        after = planner.step(0.137, MotionCommand((-0.5, -0.3)))
        self.assert_continuous(before, after)

    def test_25hz_destinations_respect_all_limits_at_500hz(self):
        planner = MotionPlanner((0, 0))
        previous_acceleration = [0, 0]
        goal = MotionCommand((0, 0))
        for k in range(10000):
            now = k * 0.002
            if k % 20 == 0:
                goal = MotionCommand(
                    (0.7 * math.sin(now * 1.7), 0.3 * math.sin(now * 2.2))
                )
            q, v, a = planner.step(now, goal)
            for i, axis in enumerate(AXES):
                self.assertTrue(math.isfinite(q[i]))
                self.assertLessEqual(abs(v[i]), MOTION_MAX_VELOCITY[axis] + 1e-8)
                self.assertLessEqual(abs(a[i]), MOTION_MAX_ACCELERATION[axis] + 1e-8)
                self.assertLessEqual(
                    abs(a[i] - previous_acceleration[i]) / 0.002,
                    MOTION_MAX_JERK[axis] + 1e-6,
                )
            previous_acceleration = a

    def test_reversal_stream_and_jittered_clock_keep_limits(self):
        rng = random.Random(438)
        planner = MotionPlanner((0, 0))
        now = 0
        before = planner.step(now, MotionCommand((0.7, 0.35)))
        goal = MotionCommand((0.7, 0.35))
        for k in range(2000):
            dt = rng.uniform(0.0015, 0.004)
            now += dt
            if k % 17 == 0:
                goal = MotionCommand((rng.uniform(-0.7, 0.7), rng.uniform(-0.35, 0.35)))
            after = planner.step(now, goal)
            for i, axis in enumerate(AXES):
                self.assertLessEqual(abs(after[1][i]), MOTION_MAX_VELOCITY[axis] + 1e-8)
                self.assertLessEqual(
                    abs(after[2][i]), MOTION_MAX_ACCELERATION[axis] + 1e-8
                )
                self.assertLessEqual(
                    abs(after[2][i] - before[2][i]) / dt, MOTION_MAX_JERK[axis] + 1e-6
                )
            before = after

    def test_hold_brakes_without_reset_and_keeps_final_position(self):
        planner = MotionPlanner((0, 0))
        goal = MotionCommand((0.7, 0.4))
        planner.step(0, goal)
        before = planner.step(0.15, goal)
        stop = MotionCommand(None)
        self.assert_continuous(before, planner.step(0.15, stop))
        self.assertFalse(planner.finished)
        final = planner.step(2.0, stop)
        self.assertTrue(planner.finished)
        self.assertGreater(final[0][0], before[0][0])  # Finite braking distance.
        for axis in (1, 2):
            self.assertTrue(all(abs(x) < 1e-9 for x in final[axis]))
        self.assert_continuous(final, planner.step(3, stop))

    def test_minimum_duration_is_not_a_forced_deadline(self):
        for duration in (0.0, 0.04, 2.0):
            planner = MotionPlanner((0, 0))
            goal = MotionCommand((0.8, 0.4), duration)
            self.assert_continuous(([0, 0], [0, 0], [0, 0]), planner.step(0, goal))
            planner.step(0.04, goal)
            self.assertFalse(planner.finished)
            if duration == 2:
                planner.step(1.99, goal)
                self.assertFalse(planner.finished)
            q, v, a = planner.step(3, goal)
            self.assertTrue(planner.finished)
            self.assertAlmostEqual(q[0], 0.8)
            self.assertAlmostEqual(q[1], 0.4)

    def test_same_destination_does_not_recalculate(self):
        planner = MotionPlanner((0, 0))
        for k in range(100):
            planner.step(k * 0.002, MotionCommand((0.5, 0.2)))
        self.assertEqual(planner.replans, 1)

    def test_initial_measured_velocity_is_preserved(self):
        planner = MotionPlanner((0.1, 0.2), (0.3, -0.2))
        q, v, a = planner.step(0, MotionCommand((0.4, 0.1)))
        self.assertEqual(q, [0.1, 0.2])
        self.assertEqual(v, [0.3, -0.2])

    def test_braking_overshoot_outside_range_is_rejected_before_output(self):
        planner = MotionPlanner((1.199, 0), (1.0, 0))
        with self.assertRaisesRegex(MotionError, "cross angle limit"):
            planner.step(0, MotionCommand(None))

    def test_invalid_destination_and_clock_are_rejected(self):
        for goal in ((math.nan, 0), (2, 0), (0, 0.7)):
            with self.assertRaises(MotionError):
                MotionPlanner((0, 0)).step(0, MotionCommand(goal))
        planner = MotionPlanner((0, 0))
        goal = MotionCommand((0.1, 0.1))
        planner.step(1, goal)
        with self.assertRaises(MotionError):
            planner.step(0.9, goal)
