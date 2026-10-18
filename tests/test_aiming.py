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

    def go_default(self):
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

    def test_convergence_requires_three_samples_dwell_and_motor_speed(self):
        for t in (0, 0.1, 0.2, 0.3):
            self.assertFalse(self.camera.aim(torso(500), (1000, 1000), t))
        self.assertTrue(self.camera.aim(torso(500), (1000, 1000), 0.8))
        self.motor.velocity = 0.1
        self.assertFalse(self.camera.aim(torso(500), (1000, 1000), 0.9))

    def test_missing_frame_keeps_goal_and_resets_settle(self):
        for t in (0, 0.1, 0.2):
            self.camera.aim(torso(500), (1000, 1000), t)
        before = list(self.motor.commands)
        self.camera.missing()
        self.assertEqual(self.motor.commands, before)
        self.assertFalse(self.camera.aim(torso(500), (1000, 1000), 2))
