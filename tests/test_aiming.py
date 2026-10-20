import unittest
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

    def test_missing_frame_resends_last_point_and_keeps_averaging(self):
        for t in (0, 0.1, 0.2):
            self.camera.aim(torso(500), (1000, 1000), t)
        before = list(self.motor.commands)
        self.camera.missing()
        # A held gap actively resends the last smoothed point -- the motor
        # keeps a fresh command instead of going quiet.
        self.assertEqual(len(self.motor.commands), len(before) + 1)
        self.assertEqual(self.motor.commands[-1], before[-1])
        # Averaging window was NOT cleared: a real detection resuming right
        # where it left off doesn't need to refill from scratch.
        self.assertAlmostEqual(self.camera.aim(torso(500), (1000, 1000), 2), 0.0)

    def test_missing_frame_with_nothing_ever_aimed_is_a_plain_reset(self):
        self.camera.missing()
        self.assertEqual(self.motor.commands, [])
        self.assertIsNone(self.camera.aim(torso(500), (1000, 1000), 0))

    def test_held_point_is_clamped_near_screen_center(self):
        from animal import settings

        limit = settings.HELD_AIM_MAX_OFFSET_FRACTION
        for t in (0, 0.1, 0.2):
            self.camera.aim(torso(900, y=100), (1000, 1000), t)
        # The live aim itself was far off-center and sent unclamped.
        self.assertAlmostEqual(self.motor.commands[-1][0], 900)
        before = len(self.motor.commands)
        self.camera.missing()
        # ...but the resent hold is pulled to within `limit` of center per
        # axis, and the orange-dot accessor reports exactly what was sent.
        sent = self.motor.commands[-1]
        self.assertEqual(len(self.motor.commands), before + 1)
        self.assertAlmostEqual(sent[0], 1000 * (0.5 + limit))
        self.assertAlmostEqual(sent[1], 1000 * (0.5 - limit))
        self.assertEqual(self.camera.held_aim_point_px, sent)

    def test_held_aim_point_px_tracks_last_sent_point(self):
        self.assertIsNone(self.camera.held_aim_point_px)
        for t in (0, 0.1, 0.2):
            self.camera.aim(torso(500), (1000, 1000), t)
        self.assertEqual(self.camera.held_aim_point_px, (500, 500))
        self.camera.missing()  # Held point survives a missing frame.
        self.assertEqual(self.camera.held_aim_point_px, (500, 500))
        self.camera.reset()
        self.assertIsNone(self.camera.held_aim_point_px)
