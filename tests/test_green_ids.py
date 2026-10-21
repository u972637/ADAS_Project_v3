import unittest

from animal.perception import Detection
from animal.wildlife import Phase
from tests.test_scenario import make_service

SIZE = (1200, 1000)
BOX = (400.0, 400.0, 600.0, 600.0)


def animal(conf, track_id=1, cls=0):
    return Detection(cls, conf, BOX, track_id)


TORSO = Detection(2, 0.9, (450.0, 430.0, 550.0, 480.0))


class GreenIdMemoryTests(unittest.TestCase):
    def setUp(self):
        self.s = make_service()  # green threshold pinned to 0.5, forget 5 s

    def feed(self, t, *dets):
        return self.s.process_detections(SIZE, list(dets), t)

    def test_confident_box_is_green_unconfident_one_is_not(self):
        self.feed(0.0, animal(0.9, 1), animal(0.3, 2))
        self.assertTrue(self.s.is_green(animal(0.9, 1)))
        self.assertFalse(self.s.is_green(animal(0.3, 2)))

    def test_id_stays_green_at_any_confidence_after_being_confident_once(self):
        self.feed(0.0, animal(0.9, 1))
        self.feed(0.1, animal(0.05, 1))
        self.assertTrue(self.s.is_green(animal(0.05, 1)))
        # ...but only that id; a different id at the same low confidence is gray.
        self.assertFalse(self.s.is_green(animal(0.05, 2)))

    def test_forgotten_after_unseen_for_the_configured_time(self):
        self.feed(0.0, animal(0.9, 1))
        self.feed(4.0, animal(0.1, 2))          # id 1 unseen for 4 s: still green
        self.assertTrue(self.s.is_green(animal(0.1, 1)))
        self.feed(5.5, animal(0.1, 2))          # unseen for 5.5 s: forgotten
        self.assertFalse(self.s.is_green(animal(0.1, 1)))

    def test_returning_at_low_confidence_after_forgetting_is_not_revived(self):
        self.feed(0.0, animal(0.9, 1))
        self.feed(6.0, animal(0.1, 1))          # back after 6 s, only 0.1
        self.assertFalse(self.s.is_green(animal(0.1, 1)))

    def test_seen_every_frame_never_expires(self):
        for i in range(0, 200):
            self.feed(i * 0.1, animal(0.9 if i == 0 else 0.1, 1))
        self.assertTrue(self.s.is_green(animal(0.1, 1)))

    def test_tracks_a_lock_through_a_confidence_dip(self):
        self.feed(0.0, animal(0.9, 1), TORSO)
        self.assertEqual(self.s.escape_tracker.target_id, 1)
        # Confidence collapses below the threshold; the id is still green, so
        # the lock keeps its bbox instead of treating it as a gap.
        step = self.feed(0.1, animal(0.2, 1), TORSO)
        self.assertEqual(self.s.escape_tracker.target_id, 1)
        self.assertIsNotNone(self.s.last_escape_step.selected)
        self.assertIsNotNone(self.s.last_escape_step.matched_torso)
        self.assertNotEqual(step.reason, "target_temporarily_missing")

    def test_never_green_box_can_not_be_locked(self):
        self.feed(0.0, animal(0.3, 1), TORSO)
        self.assertIsNone(self.s.escape_tracker.target_id)

    def test_green_status_is_updated_even_while_paused(self):
        self.s.pause()
        self.assertEqual(self.s.phase, Phase.STOPPED)
        self.feed(0.0, animal(0.9, 1))
        self.feed(0.1, animal(0.2, 1))
        self.assertTrue(self.s.is_green(animal(0.2, 1)))

    def test_stale_result_does_not_touch_the_memory(self):
        # now - captured_at beyond MAX_RESULT_AGE_SECONDS -> ignored entirely.
        self.s.process_detections(SIZE, [animal(0.9, 1)], 0.0, now=10.0)
        self.assertFalse(self.s.is_green(animal(0.1, 1)))

    def test_box_without_an_id_is_green_only_by_confidence(self):
        no_id_high = Detection(0, 0.9, BOX, None)
        no_id_low = Detection(0, 0.1, BOX, None)
        self.assertTrue(self.s.is_green(no_id_high))
        self.assertFalse(self.s.is_green(no_id_low))

    def test_torsos_never_enter_the_green_memory(self):
        self.feed(0.0, animal(0.9, 1), Detection(2, 0.95, BOX, 7))
        self.assertNotIn(7, self.s._green_ids)


if __name__ == "__main__":
    unittest.main()
