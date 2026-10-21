"""Lock -> torso lost (orange) -> torso back / timeout, through the whole service
with the REAL TrackerCameraController (only the motor itself is a spy)."""

import unittest
from unittest.mock import patch

from animal import settings
from animal.aiming import TrackerCameraController
from animal.perception import Detection
from animal.wildlife import Phase
from tests.test_scenario import detections, feed, make_service

SIZE = (1200, 1000)


class Motor:
    def __init__(self):
        self.commands = []

    def update(self, x, y):
        self.commands.append((x, y))

    def hold(self):
        pass

    def go_default(self, duration=None, yaw_deg=None, pitch_deg=None):
        pass

    def health(self):
        return {"ok": True, "fault": None}


def torso_only(shift=0):
    cx = 500 + shift
    return [Detection(2, 0.9, (cx - 50, 430, cx + 50, 480))]


def make():
    s = make_service()
    s.camera = TrackerCameraController(Motor(), SIZE)
    return s


class OrangeStateTests(unittest.TestCase):
    def lock_while_moving_right(self, s):
        for i, shift in enumerate((0, 20, 40)):
            feed(s, i * 0.1, detections(shift=shift))
        self.assertEqual(s.escape_tracker.target_id, 1)
        self.assertIsNone(s.camera.orange_point_px)

    def test_losing_everything_starts_orange_and_the_motor_follows_it(self):
        s = make()
        self.lock_while_moving_right(s)
        before = len(s.camera.tracker.commands)
        with patch.object(settings, "ORANGE_DECAY_PER_FRAME", 0.5):
            event = feed(s, 0.5, [])
            first = s.camera.orange_point_px
            self.assertEqual(event.reason, "target_temporarily_missing")
            # Still locked; the motor got a command toward the virtual target.
            self.assertEqual(s.escape_tracker.target_id, 1)
            self.assertEqual(len(s.camera.tracker.commands), before + 1)
            self.assertEqual(s.camera.tracker.commands[-1], first)
            feed(s, 0.54, [])
            second = s.camera.orange_point_px
        cx = 0.5 * SIZE[0]
        # Pulled halfway toward the horizontal center.
        self.assertAlmostEqual(second[0] - cx, 0.5 * (first[0] - cx))
        self.assertEqual(s.phase, Phase.TRACKING)

    def test_a_torso_back_within_reach_ends_orange_and_tracks_it_again(self):
        s = make()
        self.lock_while_moving_right(s)
        feed(s, 0.5, [])
        self.assertIsNotNone(s.camera.orange_point_px)
        feed(s, 0.54, torso_only(shift=45))  # near where it was last seen
        self.assertIsNone(s.camera.orange_point_px)
        self.assertEqual(s.escape_tracker.target_id, 1)
        self.assertIsNotNone(s.last_escape_step.matched_torso)

    def test_service_hands_the_orange_point_to_the_tracker_as_the_reference(self):
        s = make()
        self.lock_while_moving_right(s)
        seen = []
        real_step = s.escape_tracker.step

        def spy(*args, **kwargs):
            seen.append(kwargs.get("reference"))
            return real_step(*args, **kwargs)

        s.escape_tracker.step = spy
        feed(s, 0.5, [])               # first lost frame: no orange point yet
        orange = s.camera.orange_point
        feed(s, 0.54, [])              # now there is one: it becomes the reference
        self.assertIsNone(seen[0])
        self.assertAlmostEqual(seen[1][0], orange[0] * SIZE[0])
        self.assertAlmostEqual(seen[1][1], orange[1] * SIZE[1])

    def test_a_torso_near_the_orange_point_is_found_though_far_from_where_it_was(self):
        from unittest.mock import PropertyMock

        def run(with_reference):
            s = make()
            self.lock_while_moving_right(s)       # last torso center ~ (540, 455)
            with patch.object(settings, "ORANGE_DECAY_PER_FRAME", 0.5):
                feed(s, 0.5, [])
                feed(s, 0.54, [])                 # orange now ~ (565, 477)
                if with_reference:
                    feed(s, 0.58, torso_only(shift=160))   # center (660, 455)
                else:
                    with patch.object(
                        TrackerCameraController, "orange_point",
                        new_callable=PropertyMock, return_value=None,
                    ):
                        feed(s, 0.58, torso_only(shift=160))
            return s.last_escape_step.matched_torso

        # 120 px from the last torso (radius is only 100 here) but ~98 px from
        # the orange point the motor is chasing.
        self.assertIsNotNone(run(with_reference=True))
        self.assertIsNone(run(with_reference=False))

    def test_orange_lasting_past_the_timeout_releases_the_lock(self):
        s = make()  # make_service uses a 2 s timeout
        self.lock_while_moving_right(s)
        feed(s, 0.5, [])
        self.assertEqual(s.phase, Phase.TRACKING)
        event = feed(s, 2.6, [])
        self.assertEqual(event.reason, "tracking_lost")
        self.assertEqual(s.phase, Phase.SEARCH)
        self.assertIsNone(s.escape_tracker.target_id)
        self.assertIsNone(s.camera.orange_point_px)

    def test_playback_continues_but_never_starts_during_orange(self):
        s = make()
        for i in range(3):                         # centered: x 0.5, y 0.455
            feed(s, i * 0.1, detections(shift=100))
        self.assertTrue(s.playing)                 # started while centered
        feed(s, 0.5, [])
        self.assertTrue(s.playing)                 # kept through orange
        self.assertEqual(len(s.player.requests), 1)

        s2 = make()
        # Locked but far off-center: playback has not started yet.
        for i in range(3):
            feed(s2, i * 0.1, detections(shift=300))
        self.assertFalse(s2.playing)
        feed(s2, 0.5, [])
        self.assertFalse(s2.playing)               # orange must not start it
        self.assertEqual(s2.player.requests, [])



class BboxFollowServiceTests(unittest.TestCase):
    def lock(self, s):
        for i in range(3):
            feed(s, i * 0.1, detections(shift=100))            # bbox center (600, 500)
        self.assertEqual(s.escape_tracker.target_id, 1)

    def test_torso_lost_but_bbox_visible_the_motor_follows_the_bbox_center(self):
        s = make()
        self.lock(s)
        feed(s, 0.5, detections(shift=100, torso=False))
        step = s.last_escape_step
        self.assertIsNone(step.matched_torso)
        self.assertIsNotNone(step.followed_bbox)
        self.assertEqual(s.camera.tracker.commands[-1], (600, 500))
        self.assertIsNone(s.camera.orange_point)               # not orange: a bbox is followed
        self.assertEqual(s.escape_tracker.target_id, 1)

    def test_bbox_then_gone_becomes_orange_from_the_bbox_points(self):
        s = make()
        self.lock(s)
        for i, shift in enumerate((100, 110, 120)):
            feed(s, 0.5 + i * 0.04, detections(shift=shift, torso=False))
        feed(s, 0.7, [])
        self.assertIsNotNone(s.camera.orange_point)            # orange now
        self.assertEqual(s.escape_tracker.target_id, 1)        # lock still alive

    def test_following_a_bbox_does_not_start_the_timeout_but_orange_afterwards_does(self):
        s = make()
        self.lock(s)
        for i in range(120):                                   # 6 s of bbox only
            feed(s, 0.5 + i * 0.05, detections(shift=100, torso=False))
        self.assertEqual(s.escape_tracker.target_id, 1)
        feed(s, 7.0, [])
        event = feed(s, 9.1, [])                               # > 2 s without anything
        self.assertEqual(event.reason, "tracking_lost")

    def test_torso_reappearing_inside_the_followed_bbox_is_tracked_again(self):
        s = make()
        self.lock(s)
        feed(s, 0.5, detections(shift=100, torso=False))
        feed(s, 0.54, detections(shift=100))
        self.assertIsNotNone(s.last_escape_step.matched_torso)
        self.assertIsNone(s.last_escape_step.followed_bbox)


if __name__ == "__main__":
    unittest.main()
