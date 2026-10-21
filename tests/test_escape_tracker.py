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
        track_lost_seconds=settings.ORANGE_TIMEOUT_SECONDS,
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


class LockStartTest(unittest.TestCase):
    def test_a_bbox_without_a_matched_torso_is_not_locked(self):
        trk = tracker()
        animals, _ = boxes(640.0, 480.0, 300.0, 160.0)
        step = trk.step(0.0, animals, [], *SIZE)
        self.assertIsNone(trk.target_id)
        self.assertIsNone(step.selected)

    def test_a_torso_outside_every_bbox_does_not_start_a_lock(self):
        trk = tracker()
        animals, _ = boxes(640.0, 480.0, 300.0, 160.0)
        stray = Detection(TORSO, 0.9, (900.0, 700.0, 960.0, 760.0))
        trk.step(0.0, animals, [stray], *SIZE)
        self.assertIsNone(trk.target_id)

    def test_the_largest_bbox_that_has_a_torso_wins_not_the_largest_overall(self):
        trk = tracker()
        big_no_torso = Detection(BOAR, 0.9, (100.0, 100.0, 700.0, 700.0), 1)
        small = Detection(BOAR, 0.9, (800.0, 200.0, 1000.0, 400.0), 2)
        small_torso = Detection(TORSO, 0.8, (850.0, 250.0, 950.0, 320.0))
        step = trk.step(0.0, [big_no_torso, small], [small_torso], *SIZE)
        self.assertEqual(trk.target_id, 2)
        self.assertIs(step.selected, small)

    def test_two_candidates_with_torsos_pick_the_larger_bbox(self):
        trk = tracker()
        big = Detection(BOAR, 0.9, (100.0, 100.0, 700.0, 700.0), 1)
        big_torso = Detection(TORSO, 0.8, (300.0, 300.0, 500.0, 450.0))
        small = Detection(BOAR, 0.9, (800.0, 200.0, 1000.0, 400.0), 2)
        small_torso = Detection(TORSO, 0.8, (850.0, 250.0, 950.0, 320.0))
        trk.step(0.0, [big, small], [big_torso, small_torso], *SIZE)
        self.assertEqual(trk.target_id, 1)


class UnifiedLostTimerTest(unittest.TestCase):
    def test_nothing_to_track_for_the_timeout_releases_the_lock(self):
        """No torso AND no bbox to follow for track_lost_seconds -> released."""
        trk = tracker(track_lost_seconds=1.0)
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)  # a lock needs a torso to start
        self.assertEqual(trk.target_id, 1)

        step = None
        for i in range(1, 60):
            step = trk.step(i * 0.04, [], [], *SIZE)
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


class NearestTorsoTest(unittest.TestCase):
    def test_nearest_torso_wins_even_when_a_farther_one_is_bigger(self):
        trk = tracker()  # radius = 1.0 x last width (180) here
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)  # last torso center (640, 464)
        trk.step(0.0, animals, torsos, *SIZE)
        trk.step(0.04, [], [], *SIZE)

        # A: nearer (dist 90) but small. B: farther (dist 120, still inside) and bigger.
        animal_a = Detection(BOAR, 0.9, (500.0, 400.0, 600.0, 560.0), 10)
        torso_a = Detection(TORSO, 0.8, (520.0, 434.0, 580.0, 494.0))
        animal_b = Detection(BOAR, 0.9, (650.0, 400.0, 850.0, 560.0), 11)
        torso_b = Detection(TORSO, 0.8, (690.0, 434.0, 830.0, 494.0))

        step = trk.step(0.08, [animal_a, animal_b], [torso_a, torso_b], *SIZE)
        self.assertIs(step.selected, animal_a)
        self.assertEqual(trk.target_id, 10)


def lock_small(trk):
    """Lock on an animal with a small (40 px wide) torso centered at (700, 500)."""
    animal = Detection(BOAR, 0.9, (600.0, 400.0, 800.0, 600.0), 1)
    torso = Detection(TORSO, 0.8, (680.0, 480.0, 720.0, 520.0))
    trk.step(0.0, [animal], [torso], *SIZE)
    assert trk.target_id == 1
    return animal, torso


def torso_at(cx, cy, w=40.0):
    return Detection(TORSO, 0.8, (cx - w / 2, cy - w / 2, cx + w / 2, cy + w / 2))


class PositionMatchTest(unittest.TestCase):
    def test_radius_is_a_circle_with_a_fixed_pixel_floor(self):
        # Width-based radius alone (1.8 x 40 = 72 px) is tiny for a far target.
        no_floor = tracker(torso_bridge_radius_multiplier=1.8)
        lock_small(no_floor)
        no_floor.step(0.04, [], [], *SIZE)
        self.assertIsNone(no_floor.step(0.08, [], [torso_at(900, 500)], *SIZE).matched_torso)

        floor = tracker(torso_bridge_radius_multiplier=1.8, torso_bridge_min_radius_px=250.0)
        lock_small(floor)
        floor.step(0.04, [], [], *SIZE)
        got = floor.step(0.08, [], [torso_at(900, 500)], *SIZE)  # 200 px away
        self.assertIsNotNone(got.matched_torso)

    def test_the_floor_is_a_circle_not_a_square(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        trk.step(0.04, [], [], *SIZE)
        # dx = dy = 180 -> inside a +-250 square, but 254.6 px from the center.
        self.assertIsNone(trk.step(0.08, [], [torso_at(880, 680)], *SIZE).matched_torso)
        # dx = dy = 170 -> 240.4 px: inside the circle.
        trk2 = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk2)
        trk2.step(0.04, [], [], *SIZE)
        self.assertIsNotNone(trk2.step(0.08, [], [torso_at(870, 670)], *SIZE).matched_torso)

    def test_the_nearest_candidate_inside_the_circle_is_chosen(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        trk.step(0.04, [], [], *SIZE)
        near, far = torso_at(780, 500), torso_at(900, 500)
        step = trk.step(0.08, [], [far, near], *SIZE)
        self.assertEqual(step.matched_torso.xyxy, near.xyxy)

    def test_reference_point_replaces_the_last_torso_position(self):
        # The orange point moved toward center: the torso is expected there,
        # not where it used to be.
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        trk.step(0.04, [], [], *SIZE)
        far = torso_at(1100, 500)  # 400 px from the last torso position
        self.assertIsNone(trk.step(0.08, [], [far], *SIZE).matched_torso)
        step = trk.step(0.12, [], [far], *SIZE, reference=(1050.0, 500.0))
        self.assertIsNotNone(step.matched_torso)

    def test_same_id_bbox_with_its_torso_matches_at_any_distance(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        # The same bbox id reappears far away (e.g. after a big camera move)
        # with a torso inside it: no distance check applies.
        moved = Detection(BOAR, 0.9, (1500.0, 400.0, 1700.0, 600.0), 1)
        step = trk.step(0.04, [moved], [torso_at(1600, 500)], 2592, 1944)
        self.assertIs(step.selected, moved)
        self.assertIsNotNone(step.matched_torso)

    def test_target_id_follows_the_bbox_that_holds_the_matched_torso(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        trk.step(0.04, [], [], *SIZE)
        other_id = Detection(BOAR, 0.9, (650.0, 400.0, 850.0, 600.0), 77)
        step = trk.step(0.08, [other_id], [torso_at(760, 500)], *SIZE)
        self.assertEqual(trk.target_id, 77)
        self.assertIs(step.selected, other_id)

    def test_a_torso_inside_another_species_bbox_is_skipped_for_the_next_one(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        trk.step(0.04, [], [], *SIZE)
        deer = Detection(1, 0.9, (700.0, 400.0, 860.0, 600.0), 5)
        nearer_in_deer = torso_at(780, 500)       # inside the deer bbox: nearest
        farther_bare = torso_at(850, 640)         # no bbox around it
        step = trk.step(0.08, [deer], [nearer_in_deer, farther_bare], *SIZE)
        self.assertEqual(step.matched_torso.xyxy, farther_bare.xyxy)
        self.assertEqual(trk.target_class, BOAR)


class BboxFollowTest(unittest.TestCase):
    def test_torso_gone_but_same_id_bbox_visible_follows_the_bbox(self):
        trk = tracker(track_lost_seconds=1.0, torso_bridge_min_radius_px=250.0)
        animal, _ = lock_small(trk)
        step = trk.step(0.04, [animal], [], *SIZE)
        self.assertIsNone(step.matched_torso)
        self.assertIs(step.followed_bbox, animal)
        self.assertIs(step.selected, animal)
        self.assertFalse(step.terminated)

    def test_following_a_bbox_keeps_the_lock_alive_indefinitely(self):
        trk = tracker(track_lost_seconds=1.0, torso_bridge_min_radius_px=250.0)
        animal, _ = lock_small(trk)
        for i in range(1, 100):  # ~4 s, far past the 1 s timeout
            step = trk.step(i * 0.04, [animal], [], *SIZE)
            self.assertFalse(step.terminated)
        # ...until the bbox is gone too: then the timer runs and releases.
        released = None
        for i in range(100, 160):
            step = trk.step(i * 0.04, [], [], *SIZE)
            if step.terminated:
                released = step
                break
        self.assertIsNotNone(released)
        self.assertEqual(released.termination_reason, "tracking_lost")

    def test_a_returning_bbox_with_a_new_id_near_the_old_spot_is_followed(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        trk.step(0.04, [], [], *SIZE)
        new_id = Detection(BOAR, 0.9, (680.0, 400.0, 880.0, 600.0), 42)
        step = trk.step(0.08, [new_id], [], *SIZE)
        self.assertIs(step.followed_bbox, new_id)
        self.assertEqual(trk.target_id, 42)

    def test_the_reference_is_the_rectangle_not_only_the_center(self):
        # A big bbox: the old torso spot is inside it but 380 px from its center.
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)                          # last torso center (700, 500)
        trk.step(0.04, [], [], *SIZE)
        big = Detection(BOAR, 0.9, (700.0, 100.0, 1500.0, 900.0), 43)  # center (1100, 500)
        step = trk.step(0.08, [big], [], *SIZE)
        self.assertIs(step.followed_bbox, big)

    def test_a_far_bbox_or_another_species_is_not_followed(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        lock_small(trk)
        trk.step(0.04, [], [], *SIZE)
        far = Detection(BOAR, 0.9, (1400.0, 100.0, 1600.0, 300.0), 44)
        deer_here = Detection(1, 0.9, (600.0, 400.0, 800.0, 600.0), 45)
        step = trk.step(0.08, [far, deer_here], [], *SIZE)
        self.assertIsNone(step.followed_bbox)
        self.assertIsNone(step.selected)

    def test_a_torso_always_beats_following_the_bbox(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        animal, _ = lock_small(trk)
        # Same-id bbox, but its torso moved outside the bbox: matched by position.
        stray = torso_at(700, 640)
        step = trk.step(0.04, [animal], [stray], *SIZE)
        self.assertIsNotNone(step.matched_torso)
        self.assertIsNone(step.followed_bbox)

    def test_the_torso_taken_over_once_it_shows_up_inside_the_followed_bbox(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        animal, _ = lock_small(trk)
        trk.step(0.04, [animal], [], *SIZE)                 # bbox only
        step = trk.step(0.08, [animal], [torso_at(690, 520)], *SIZE)
        self.assertIsNotNone(step.matched_torso)
        self.assertIsNone(step.followed_bbox)

    def test_followed_bbox_center_becomes_the_next_reference(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        animal, _ = lock_small(trk)                         # bbox center (700, 500)
        moved = Detection(BOAR, 0.9, (900.0, 400.0, 1100.0, 600.0), 1)  # center (1000, 500)
        trk.step(0.04, [moved], [], *SIZE)                  # followed: reference -> (1000, 500)
        # Same bbox vanishes; a torso at (1150, 500) is 150 px from the followed
        # center but 450 px from the original torso.
        step = trk.step(0.08, [], [torso_at(1150, 500)], *SIZE)
        self.assertIsNotNone(step.matched_torso)


class ReleaseAndCooldownTest(unittest.TestCase):
    def _lost(self, trk):
        """Lock, then lose everything until the lock is released; returns the release time."""
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(0.0, animals, torsos, *SIZE)
        t = 0.0
        while True:
            t += 0.1
            if trk.step(t, [], [], *SIZE).terminated:
                return t

    def test_no_new_lock_for_the_whole_cooldown_then_any_animal_is_allowed(self):
        trk = tracker(track_lost_seconds=1.0, reselect_grace_seconds=5.0)
        released = self._lost(trk)
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(released + 4.9, animals, torsos, *SIZE)
        self.assertIsNone(trk.target_id)                    # still banned
        trk.step(released + 5.1, animals, torsos, *SIZE)
        self.assertEqual(trk.target_id, 1)                  # the SAME id may be locked again

    def test_area_escape_also_starts_the_same_cooldown(self):
        trk = tracker(reselect_grace_seconds=5.0)
        end = first_end(trk, sequence("walk_away"))
        self.assertEqual(end[1], AREA_REASON)
        animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
        trk.step(end[0] + 4.0, animals, torsos, *SIZE)
        self.assertIsNone(trk.target_id)

    def test_baseline_is_remeasured_when_the_target_bbox_id_changes(self):
        trk = tracker(torso_bridge_min_radius_px=250.0)
        for i in range(0, 30):                              # 1.2 s on a big animal
            animals, torsos = boxes(640.0, 480.0, 300.0, 160.0)
            trk.step(i * 0.04, animals, torsos, *SIZE)
        self.assertTrue(trk.baseline_ready)
        # A much smaller animal (new id) takes over the torso spot: its size is
        # not "the first animal shrinking", so no baseline carries over.
        small = Detection(BOAR, 0.9, (590.0, 440.0, 690.0, 520.0), 9)
        small_torso = Detection(TORSO, 0.8, (620.0, 460.0, 660.0, 500.0))
        step = trk.step(1.2, [small], [small_torso], *SIZE)
        self.assertEqual(trk.target_id, 9)
        self.assertFalse(trk.baseline_ready)
        for i in range(1, 60):                              # 2.4 s as the small animal
            step = trk.step(1.2 + i * 0.04, [small], [small_torso], *SIZE)
            self.assertFalse(step.terminated)


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
