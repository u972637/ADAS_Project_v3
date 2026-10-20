"""EscapeTracker: area AND height distance check (bbox shrinking only).

Synthetic single-animal sequences (no camera, model or hardware needed):
  turn       box gets narrower (side view -> rear view), height unchanged
  walk_away  box shrinks in both directions (distance grows)

Run:  python -m unittest tests.test_escape_tracker
"""

from __future__ import annotations

import unittest

from animal import settings
from animal.perception import (
    AREA_REASON,
    Detection,
    EscapeTracker,
)

SIZE = (1296, 972)
BOAR, TORSO = 0, 2


def tracker(**overrides):
    kwargs = dict(
        track_lost_seconds=settings.TRACK_LOST_SECONDS,
        reselect_grace_seconds=settings.RESELECT_GRACE_SECONDS,
        area_baseline_seconds=settings.AREA_BASELINE_SECONDS,
        area_smooth_seconds=settings.AREA_SMOOTH_SECONDS,
        area_hold_seconds=settings.AREA_HOLD_SECONDS,
        area_ratio_threshold=settings.AREA_RATIO_THRESHOLD,
        height_ratio_threshold=settings.HEIGHT_RATIO_THRESHOLD,
        distance_metric=settings.DISTANCE_METRIC,
        edge_margin_px=settings.EDGE_MARGIN_PX,
    )
    kwargs.update(overrides)
    return EscapeTracker(**kwargs)


def boxes(cx, cy, w, h):
    animal = Detection(BOAR, 0.9, (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2), 1)
    torso = Detection(
        TORSO, 0.8, (cx - w * 0.3, cy - h * 0.35, cx + w * 0.3, cy + h * 0.15), 2
    )
    return [animal], [torso]


def sequence(kind, fps=25.0, seconds=8.0):
    for i in range(int(seconds * fps)):
        t = i / fps
        cx, w, h = 640.0, 300.0, 160.0
        if t >= 2.0:
            u = min(1.0, (t - 2.0) / 1.5)
            if kind == "turn":
                w *= 1.0 - 0.6 * u  # area -> 40 %, height unchanged
            elif kind == "walk_away":
                w *= 1.0 - 0.45 * u  # distance x1.8: both sides -> 55 %
                h *= 1.0 - 0.45 * u
        yield t, boxes(cx, 480.0, w, h)


def first_end(trk, frames):
    for t, (animals, torsos) in frames:
        step = trk.step(t, animals, torsos, *SIZE)
        if step.terminated:
            return t, step.termination_reason
    return None


class EscapeTrackerTest(unittest.TestCase):
    def test_settings_defaults(self):
        self.assertEqual(settings.DISTANCE_METRIC, "and")
        self.assertEqual(
            (settings.AREA_RATIO_THRESHOLD, settings.HEIGHT_RATIO_THRESHOLD),
            (0.50, 0.80),
        )
        self.assertEqual(
            (
                settings.AREA_BASELINE_SECONDS,
                settings.AREA_SMOOTH_SECONDS,
                settings.AREA_HOLD_SECONDS,
            ),
            (0.4, 0.35, 0.5),
        )

    def test_turning_alone_does_not_end(self):
        self.assertIsNone(first_end(tracker(), sequence("turn")))
        self.assertIsNone(
            first_end(tracker(distance_metric="height"), sequence("turn"))
        )
        original = first_end(tracker(distance_metric="area"), sequence("turn"))
        self.assertEqual(original[1], AREA_REASON)

    def test_walking_away_ends(self):
        t, reason = first_end(tracker(), sequence("walk_away"))
        self.assertEqual(reason, AREA_REASON)
        # Both dimensions shrink by the same scale. Area reaches 0.50 at
        # scale=sqrt(0.50), LATER than height reaching 0.80; both are required.
        scale = min(settings.HEIGHT_RATIO_THRESHOLD, settings.AREA_RATIO_THRESHOLD**0.5)
        reach = 2.0 + 1.5 * (1.0 - scale) / 0.45
        self.assertGreater(t, reach)
        self.assertLess(
            t, reach + settings.AREA_SMOOTH_SECONDS + settings.AREA_HOLD_SECONDS + 0.2
        )

    def test_and_needs_both(self):
        self.assertIsNone(
            first_end(tracker(height_ratio_threshold=0.3), sequence("walk_away"))
        )

    def test_original_settings_still_available(self):
        trk = tracker(
            distance_metric="area",
            area_baseline_seconds=1.0,
            area_smooth_seconds=0.5,
            area_hold_seconds=1.0,
        )
        self.assertEqual(first_end(trk, sequence("turn"))[1], AREA_REASON)

    def test_invalid_metric_rejected(self):
        with self.assertRaises(ValueError):
            tracker(distance_metric="volume")


class UnifiedLostTimerTest(unittest.TestCase):
    def test_torso_never_matching_eventually_times_out_even_with_healthy_bbox(self):
        """The animal bbox keeps being tracked every single frame (never a
        gap), but its torso is never detected at all -- used to stall
        forever in STATE_TORSO_MISSING with no timeout, since the old
        lost-timer only ever advanced when the BBOX was absent. Now the
        timer is driven by matched_torso alone, so this still releases on
        track_lost_seconds."""
        trk = tracker(track_lost_seconds=1.0)
        animal, _ = boxes(640.0, 480.0, 300.0, 160.0)
        animal = animal[0]
        trk.step(0.0, [animal], [], *SIZE)
        self.assertEqual(trk.target_id, 1)

        step = None
        for i in range(1, 60):
            step = trk.step(i * 0.04, [animal], [], *SIZE)
            if step.terminated:
                break
        self.assertTrue(step.terminated)
        self.assertEqual(step.termination_reason, "tracking_lost")
        self.assertIsNone(trk.target_id)

    def test_resets_when_torso_matches_again_before_the_timeout(self):
        trk = tracker(track_lost_seconds=1.0)
        animal, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        animal = animal[0]
        trk.step(0.0, [animal], torsos, *SIZE)

        # Torso missing for a while, but a real match lands before 1.0s.
        trk.step(0.2, [animal], [], *SIZE)
        trk.step(0.4, [animal], [], *SIZE)
        step = trk.step(0.6, [animal], torsos, *SIZE)
        self.assertFalse(step.terminated)
        self.assertIsNotNone(step.matched_torso)

        # Timer should have reset at 0.6s -- still alive at 0.6 + 0.9s.
        step = trk.step(1.5, [animal], [], *SIZE)
        self.assertFalse(step.terminated)


class ReidentifyAreaPriorityTest(unittest.TestCase):
    def test_bigger_torso_wins_over_nearer_smaller_one(self):
        trk = tracker()  # defaults: radius_multiplier=1.0, bbox grace=1.0s
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)  # last_torso width=180
        trk.step(0.0, animals, torsos, *SIZE)
        trk.step(0.04, [], [], *SIZE)  # full gap

        # Candidate A: closer (dist=90) but a small torso.
        animal_a = Detection(BOAR, 0.9, (500.0, 400.0, 600.0, 560.0), 10)
        torso_a = Detection(TORSO, 0.8, (520.0, 434.0, 580.0, 494.0))  # 60x60
        # Candidate B: farther (dist=120, still within radius=180) but a
        # much bigger torso -- should win despite being less close.
        animal_b = Detection(BOAR, 0.9, (650.0, 400.0, 850.0, 560.0), 11)
        torso_b = Detection(TORSO, 0.8, (690.0, 434.0, 830.0, 494.0))  # 140x60

        step = trk.step(0.08, [animal_a, animal_b], [torso_a, torso_b], *SIZE)
        self.assertIs(step.selected, animal_b)
        self.assertEqual(trk.target_id, 11)


class BboxRetargetGraceTest(unittest.TestCase):
    def test_far_candidate_rejected_before_grace_then_adopted_after(self):
        trk = tracker(bbox_retarget_grace_seconds=0.5)
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)
        trk.step(0.04, [], [], *SIZE)  # bbox (and torso) gap starts here

        far_animal = Detection(BOAR, 0.9, (1000.0, 800.0, 1200.0, 900.0), 99)
        far_torso = Detection(TORSO, 0.8, (1020.0, 820.0, 1180.0, 880.0))

        # Still within grace (0.1 - 0.04 = 0.06s < 0.5s): too far, rejected.
        step = trk.step(0.1, [far_animal], [far_torso], *SIZE)
        self.assertIsNone(step.selected)
        self.assertEqual(trk.target_id, 1)

        # Grace expired (0.6 - 0.04 = 0.56s >= 0.5s): same candidate, same
        # distance -- now adopted, radius no longer applies.
        step = trk.step(0.6, [far_animal], [far_torso], *SIZE)
        self.assertIs(step.selected, far_animal)
        self.assertEqual(trk.target_id, 99)


class TorsoBridgeRadiusTest(unittest.TestCase):
    def test_default_radius_misses_a_torso_that_drifted_past_one_width(self):
        trk = tracker()  # torso_bridge_radius_multiplier defaults to 1.0
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)  # torso width here is 180px
        trk.step(0.04, [], [], *SIZE)  # full gap

        # Reappears 1.5 torso-widths away (270px) -- plausible after even a
        # short real-world gap, but past the default 1x-width radius.
        far_torso = Detection(TORSO, 0.8, (640.0 + 270 - 90, 424.0, 640.0 + 270 + 90, 504.0))
        step = trk.step(0.08, [], [far_torso], *SIZE)
        self.assertIsNone(step.matched_torso)

    def test_wider_radius_setting_recovers_the_same_case(self):
        trk = tracker(torso_bridge_radius_multiplier=2.5)
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)
        trk.step(0.04, [], [], *SIZE)

        far_torso = Detection(TORSO, 0.8, (640.0 + 270 - 90, 424.0, 640.0 + 270 + 90, 504.0))
        step = trk.step(0.08, [], [far_torso], *SIZE)
        self.assertIsNotNone(step.matched_torso)
        self.assertEqual(step.matched_torso.xyxy, far_torso.xyxy)


class TorsoContainmentMissFallbackTest(unittest.TestCase):
    def test_bbox_tracked_but_torso_just_outside_containment_still_matches(self):
        """Animal bbox is tracked fine (no gap at all) but this frame's torso
        center lands just outside the bbox edge -- torso_matches()'s strict
        containment check fails even though the torso clearly belongs to
        this individual. Should bridge via the same position fallback used
        for a full bbox gap, not drop to STATE_TORSO_MISSING (red -> yellow)
        for one frame of detector jitter."""
        trk = tracker()
        animal, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        animal = animal[0]  # Detection, (490,400,790,560)
        step = trk.step(0.0, [animal], torsos, *SIZE)
        self.assertIsNotNone(step.matched_torso)

        # Same animal bbox again (same track_id, direct match would
        # succeed for the bbox itself), but the torso this frame sits just
        # below the bbox's bottom edge (560) -- fails containment.
        stray_torso = Detection(TORSO, 0.8, (620.0, 550.0, 660.0, 590.0))
        step = trk.step(0.04, [animal], [stray_torso], *SIZE)
        self.assertIs(step.selected, animal)
        self.assertIsNotNone(step.matched_torso)
        self.assertEqual(step.matched_torso.xyxy, stray_torso.xyxy)
        self.assertFalse(step.terminated)


class ReidentificationTest(unittest.TestCase):
    def test_new_track_id_at_same_torso_position_is_adopted(self):
        trk = tracker()
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)
        self.assertEqual(trk.target_id, 1)

        # One frame with nothing detected at all (a gap in both animal and
        # torso detection, not just the animal bbox).
        step = trk.step(0.04, [], [], *SIZE)
        self.assertIsNone(step.selected)
        self.assertFalse(step.terminated)

        # ByteTrack hands the SAME real animal a brand new ID, right where
        # its torso last was -- should be adopted immediately, not treated
        # as a stranger that has to wait out track_lost_seconds.
        animal2 = Detection(BOAR, 0.9, (490.0, 400.0, 790.0, 560.0), 99)
        torso2 = Detection(TORSO, 0.8, (550.0, 424.0, 730.0, 504.0), 100)
        step = trk.step(0.08, [animal2], [torso2], *SIZE)
        self.assertIs(step.selected, animal2)
        # matched_torso is re-tagged with the locked individual's
        # track_id/species (see animal/perception.py), so it's a new object
        # with the same position as torso2, not torso2 itself.
        self.assertEqual(step.matched_torso.xyxy, torso2.xyxy)
        self.assertEqual(step.matched_torso.track_id, 99)
        self.assertEqual(step.matched_torso.class_id, BOAR)
        self.assertEqual(trk.target_id, 99)
        self.assertEqual(trk.target_class, BOAR)
        self.assertFalse(step.terminated)

    def test_new_track_id_of_a_different_species_is_not_adopted(self):
        trk = tracker()
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)
        trk.step(0.04, [], [], *SIZE)

        DEER = 1
        animal2 = Detection(DEER, 0.9, (490.0, 400.0, 790.0, 560.0), 99)
        torso2 = Detection(TORSO, 0.8, (550.0, 424.0, 730.0, 504.0), 100)
        step = trk.step(0.08, [animal2], [torso2], *SIZE)
        self.assertIsNone(step.selected)
        self.assertEqual(trk.target_id, 1)  # lock untouched, still waiting

    def test_far_away_new_track_id_is_not_adopted(self):
        trk = tracker()
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)
        trk.step(0.04, [], [], *SIZE)

        # Same species, but nowhere near the last confirmed torso position.
        animal2 = Detection(BOAR, 0.9, (1000.0, 800.0, 1200.0, 900.0), 99)
        torso2 = Detection(TORSO, 0.8, (1020.0, 820.0, 1180.0, 880.0), 100)
        step = trk.step(0.08, [animal2], [torso2], *SIZE)
        self.assertIsNone(step.selected)
        self.assertEqual(trk.target_id, 1)


class IntegratedEscapeTest(unittest.TestCase):
    def test_runtime_defaults_reject_turn_then_stop_audio_on_retreat(self):
        # Exercise the actual settings → scenario factory integration path
        # (real AREA_RATIO_THRESHOLD/HEIGHT_RATIO_THRESHOLD/etc, not test stubs).
        from animal.wildlife import WildlifeDeterrent, Phase
        from tests.test_scenario import Camera, Player

        service = WildlifeDeterrent.from_settings(player=Player(), camera=Camera())
        service.min_selection_confidence = 0.5  # synthetic animals are 0.9

        def sample(t, width=200, height=200):
            animal = Detection(
                0,
                0.9,
                (500 - width / 2, 500 - height / 2, 500 + width / 2, 500 + height / 2),
                1,
            )
            torso = Detection(
                2,
                0.9,
                (500 - width / 4, 500 - height / 4, 500 + width / 4, 500 + height / 4),
            )
            return service.process_detections((1200, 1000), [animal, torso], t)

        for i in range(50):
            sample(i / 25)
        self.assertEqual(service.phase, Phase.TRACKING)
        self.assertTrue(service.playing)
        for i in range(50, 100):
            sample(i / 25, width=80, height=200)
        # Area=40%, height=100% (turning, not retreating): "and" metric needs both.
        self.assertEqual(service.phase, Phase.TRACKING)
        self.assertTrue(service.playing)
        event = None
        for i in range(100, 150):
            event = sample(i / 25, width=110, height=110)
            if event.reason == AREA_REASON:
                break
        self.assertEqual(event.reason, AREA_REASON)
        self.assertEqual(service.phase, Phase.SEARCH)
        self.assertFalse(service.playing)
        self.assertEqual(service.player.state.state, "stopped")
        self.assertEqual(len(service.player.requests), 1)


if __name__ == "__main__":
    unittest.main()
