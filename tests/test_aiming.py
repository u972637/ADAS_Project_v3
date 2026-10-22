import unittest
from unittest.mock import patch

from animal import settings
from animal.aiming import TrackerCameraController
from animal.perception import Detection


class Motor:
    def __init__(self):
        self.commands = []
        self.velocity = 0

    def update(self, x, y):
        self.commands.append((x, y))

    def get_state(self, axis):
        return (0, self.velocity, 0)

    def hold(self):
        pass

    def go_default(self, duration=None, yaw_deg=None, pitch_deg=None):
        pass


def torso(x, y=500):
    return Detection(2, 0.9, (x - 20, y - 20, x + 20, y + 20))


def uniform_decay(n):
    """Same pull n in all three orange zones (zone-independent tests)."""
    return patch.multiple(
        settings,
        ORANGE_DECAY_OUTER=n,
        ORANGE_DECAY_MIDDLE=n,
        ORANGE_DECAY_INNER=n,
    )


class AimingTests(unittest.TestCase):
    def setUp(self):
        self.motor = Motor()
        self.camera = TrackerCameraController(self.motor, (1000, 1000))

    def test_three_frame_average(self):
        for i, x in enumerate((600, 610, 620)):
            self.camera.aim(torso(x), (1000, 1000), i * 0.1)
        self.assertAlmostEqual(self.motor.commands[-1][0], 610)

    def test_isolated_jump_rejected_real_relocation_accepted(self):
        self.camera.aim(torso(500), (1000, 1000), 0)
        self.camera.aim(torso(800), (1000, 1000), 0.1)
        self.assertEqual(len(self.motor.commands), 1)
        self.camera.aim(torso(810), (1000, 1000), 0.2)
        self.assertEqual(len(self.motor.commands), 2)
        self.assertAlmostEqual(self.motor.commands[-1][0], 810)

    def test_centered_measurement_overrides_stale_average(self):
        self.camera.aim(torso(600), (1000, 1000), 0)
        self.camera.aim(torso(500), (1000, 1000), 0.1)
        self.assertEqual(self.motor.commands[-1], (500, 500))

    def test_offset_is_none_until_enough_samples_then_numeric(self):
        # Fewer than AIM_AVERAGE_FRAMES samples: no usable reading yet.
        self.assertIsNone(self.camera.aim(torso(500), (1000, 1000), 0))
        self.assertIsNone(self.camera.aim(torso(500), (1000, 1000), 0.1))
        # Third sample fills the window: dead center -> offset 0.0.
        self.assertAlmostEqual(self.camera.aim(torso(500), (1000, 1000), 0.2), 0.0)
        # Off-center: offset is the max per-axis fraction from center.
        offset = self.camera.aim(torso(600), (1000, 1000), 0.3)
        self.assertGreater(offset, 0.0)

    def test_first_orange_point_extrapolates_the_last_two_filtered_points(self):
        # Filtered (3-frame average) points: 0.60, then avg(0.60, 0.62) = 0.61.
        # First orange = last + (last - previous) = 0.62.
        self.camera.aim(torso(600), (1000, 1000), 0)
        self.camera.aim(torso(620), (1000, 1000), 0.1)
        before = len(self.motor.commands)
        self.camera.missing()
        self.assertEqual(len(self.motor.commands), before + 1)
        x, y = self.motor.commands[-1]
        self.assertAlmostEqual(x, 620)
        self.assertAlmostEqual(y, 500)
        self.assertEqual(self.camera.orange_point_px, (x, y))

    def test_later_orange_points_decay_toward_the_screen_center(self):
        from unittest.mock import patch

        from animal import settings

        self.camera.aim(torso(600), (1000, 1000), 0)
        self.camera.aim(torso(620), (1000, 1000), 0.1)
        with uniform_decay(0.5):
            xs = []
            for _ in range(3):
                self.camera.missing()
                xs.append(self.motor.commands[-1][0])
        # 0.62, then pulled halfway to the CENTER (0.5, not the 0 corner): 0.56, 0.53.
        self.assertAlmostEqual(xs[0], 620)
        self.assertAlmostEqual(xs[1], 560)
        self.assertAlmostEqual(xs[2], 530)
        self.assertAlmostEqual(self.motor.commands[-1][1], 500)

    def test_each_zone_uses_its_own_decay(self):
        # Distance from center (larger axis, 0 = center, 1 = edge): the point is
        # pulled by the zone it is IN at the start of each orange frame.
        zones = dict(
            ORANGE_ZONE_OUTER_FRACTION=0.6,
            ORANGE_ZONE_INNER_FRACTION=0.3,
            ORANGE_DECAY_OUTER=0.2,
            ORANGE_DECAY_MIDDLE=0.5,
            ORANGE_DECAY_INNER=0.9,
        )
        with patch.multiple(settings, **zones):
            self.camera.aim(torso(900), (1000, 1000), 0)     # 0.8 from center: outer
            xs = []
            for _ in range(4):
                self.camera.missing()
                xs.append(self.motor.commands[-1][0])
        # first point stays (no velocity); then 900 -> 820 (outer, n=0.2) -> 756
        # (0.64 from center: still outer) -> 628 (0.512: now middle, n=0.5)
        self.assertAlmostEqual(xs[0], 900)
        self.assertAlmostEqual(xs[1], 500 + 0.8 * 400)
        self.assertAlmostEqual(xs[2], 500 + 0.8 * 0.8 * 400)
        self.assertAlmostEqual(xs[3], 628)

    def test_zone_boundaries(self):
        with patch.multiple(
            settings,
            ORANGE_ZONE_OUTER_FRACTION=0.6,
            ORANGE_ZONE_INNER_FRACTION=0.3,
            ORANGE_DECAY_OUTER=0.1,
            ORANGE_DECAY_MIDDLE=0.2,
            ORANGE_DECAY_INNER=0.3,
        ):
            d = TrackerCameraController._decay_for
            self.assertEqual(d(0.5, 0.5), 0.3)
            self.assertEqual(d(0.5 + 0.5 * 0.29, 0.5), 0.3)
            self.assertEqual(d(0.5 + 0.5 * 0.3, 0.5), 0.2)     # inner edge: middle
            self.assertEqual(d(0.5, 0.5 - 0.5 * 0.59), 0.2)
            self.assertEqual(d(0.5, 0.5 - 0.5 * 0.6), 0.1)     # outer edge: outer
            self.assertEqual(d(1.0, 0.5), 0.1)
            self.assertEqual(d(0.5, 0.5 + 0.5 * 0.5), 0.2)     # larger axis wins:
            self.assertEqual(d(0.95, 0.52), 0.1)               # x decides

    def test_decay_works_on_both_axes_around_the_center(self):
        from unittest.mock import patch

        from animal import settings

        self.camera.aim(torso(300, y=800), (1000, 1000), 0)  # lower left
        with uniform_decay(0.5):
            self.camera.missing()   # no velocity yet: stays at the last point
            self.camera.missing()
        x, y = self.motor.commands[-1]
        self.assertAlmostEqual(x, 400)   # 500 + 0.5 * (300 - 500)
        self.assertAlmostEqual(y, 650)   # 500 + 0.5 * (800 - 500)

    def test_first_orange_point_is_clamped_inside_the_frame(self):
        self.camera._last_point, self.camera._prev_point = (0.99, 0.01), (0.90, 0.10)
        self.camera.missing()
        x, y = self.motor.commands[-1]
        self.assertAlmostEqual(x, 1000)   # 2*0.99 - 0.90 = 1.08 -> 1.0
        self.assertAlmostEqual(y, 0)      # 2*0.01 - 0.10 = -0.08 -> 0.0

    def test_a_single_known_point_has_no_velocity_to_extrapolate(self):
        self.camera.aim(torso(700), (1000, 1000), 0)
        self.camera.missing()
        self.assertEqual(self.motor.commands[-1], (700, 500))

    def test_real_torso_after_orange_restarts_the_filter(self):
        for t in (0, 0.1, 0.2):
            self.camera.aim(torso(600), (1000, 1000), t)
        self.camera.missing()
        self.camera.missing()
        # Far jump (0.3 > the 0.15 outlier gate) that would have been rejected
        # against the pre-loss samples -- accepted now, and not averaged in.
        self.assertIsNone(self.camera.aim(torso(900), (1000, 1000), 1.0))
        self.assertEqual(self.motor.commands[-1], (900, 500))
        self.assertIsNone(self.camera.orange_point_px)
        # The pre-loss point must not give a velocity across the gap.
        self.camera.missing()
        self.assertEqual(self.motor.commands[-1], (900, 500))

    def test_switching_between_torso_and_bbox_restarts_the_filter(self):
        for t in (0, 0.1, 0.2):
            self.camera.aim(torso(600), (1000, 1000), t)
        # The bbox center is a different kind of point, 0.3 away: without a
        # restart it would be rejected by the 0.15 outlier gate / averaged in.
        self.assertIsNone(self.camera.aim(torso(900), (1000, 1000), 0.3, source="bbox"))
        self.assertEqual(self.motor.commands[-1], (900, 500))
        # Filtering continues WITHIN the new source ...
        self.camera.aim(torso(920), (1000, 1000), 0.4, source="bbox")
        self.assertAlmostEqual(self.motor.commands[-1][0], 910)   # avg(900, 920)
        # ... and a switch back restarts it again.
        self.camera.aim(torso(600), (1000, 1000), 0.5, source="torso")
        self.assertEqual(self.motor.commands[-1], (600, 500))

    def test_no_velocity_is_taken_across_a_source_switch(self):
        self.camera.aim(torso(600), (1000, 1000), 0)
        self.camera.aim(torso(640), (1000, 1000), 0.1)            # moving right
        self.camera.aim(torso(900), (1000, 1000), 0.2, source="bbox")
        self.camera.missing()                                      # first orange
        self.assertEqual(self.motor.commands[-1], (900, 500))      # no extrapolation

    def test_no_velocity_is_taken_across_an_accepted_relocation(self):
        for t, x in enumerate((600, 600, 600)):
            self.camera.aim(torso(x), (1000, 1000), t * 0.1)
        self.camera.aim(torso(900), (1000, 1000), 0.3)   # isolated jump: rejected
        self.camera.aim(torso(905), (1000, 1000), 0.4)   # confirmed relocation
        self.camera.missing()
        self.assertEqual(self.motor.commands[-1], (905, 500))

    def test_reset_clears_the_orange_state(self):
        self.camera.aim(torso(600), (1000, 1000), 0)
        self.camera.missing()
        self.assertIsNotNone(self.camera.orange_point_px)
        self.camera.reset()
        self.assertIsNone(self.camera.orange_point_px)
        before = len(self.motor.commands)
        self.camera.missing()
        self.assertEqual(len(self.motor.commands), before)

    def test_missing_frame_with_nothing_ever_aimed_is_a_plain_reset(self):
        self.camera.missing()
        self.assertEqual(self.motor.commands, [])
        self.assertIsNone(self.camera.orange_point_px)
        self.assertIsNone(self.camera.aim(torso(500), (1000, 1000), 0))
