import unittest

from animal.aiming import TrackerCameraController
from animal.eventlog import EventLogger
from tests.test_orange_state import Motor, SIZE, torso_only
from tests.test_scenario import detections, feed, make_service


class EventLogTests(unittest.TestCase):
    def setUp(self):
        self.s = make_service()
        self.s.camera = TrackerCameraController(Motor(), SIZE)
        self.lines = []
        self.log = EventLogger(self.lines.append)
        self.t = 0.0

    def feed(self, ds, dt=0.1):
        self.t += dt
        feed(self.s, self.t, ds)
        self.log.observe(self.s, self.t)
        self.log.observe(self.s, self.t)       # repeated look: no duplicates

    def tags(self):
        return [l.split("]")[0] + "]" for l in self.lines]

    def test_lock_orange_recover_by_torso_then_flee(self):
        for _ in range(3):
            self.feed(detections(shift=100))
        self.feed([])
        self.feed([])
        self.feed(torso_only(shift=100))
        self.assertIn("[추적 시작]", self.tags())
        self.assertEqual(self.tags().count("[추적 실패(주황)]"), 1)
        recovered = [l for l in self.lines if l.startswith("[주황 해제]")]
        self.assertEqual(len(recovered), 1)
        self.assertIn("torso", recovered[0])
        self.feed([], dt=0.1)
        self.feed([], dt=2.5)
        self.assertIn("[추적 종료]", self.tags())

    def test_bbox_first_is_reported_and_quiet_frames_log_nothing(self):
        for _ in range(3):
            self.feed(detections(shift=100))
        self.feed([])
        n = len(self.lines)
        self.feed([])
        self.assertEqual(len(self.lines), n)
        self.feed(detections(shift=100, torso=False))
        self.assertIn("bbox", [l for l in self.lines if l.startswith("[주황 해제]")][0])


if __name__ == "__main__":
    unittest.main()
