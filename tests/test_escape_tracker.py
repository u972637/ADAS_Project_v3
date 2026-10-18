"""EscapeTracker: area AND height distance check, time-based movement speed.

Synthetic single-animal sequences (no camera, model or hardware needed):
  turn       box gets narrower (side view -> rear view), height unchanged
  walk_away  box shrinks in both directions (distance grows)
  run        torso centre moves sideways at a constant pixel speed

Run:  python -m unittest tests.test_escape_tracker
"""

from __future__ import annotations

import unittest

from animal import settings
from animal.perception import (
    AREA_REASON,
    RAPID_MOVEMENT_REASON,
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
        torso_window_size=settings.TORSO_WINDOW_SIZE,
        torso_speed_threshold=settings.TORSO_SPEED_THRESHOLD,
        nominal_fps=settings.TORSO_NOMINAL_FPS,
    )
    kwargs.update(overrides)
    return EscapeTracker(**kwargs)


def boxes(cx, cy, w, h):
    animal = Detection(BOAR, 0.9, (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2), 1)
    torso = Detection(
        TORSO, 0.8, (cx - w * 0.3, cy - h * 0.35, cx + w * 0.3, cy + h * 0.15), 2
    )
    return [animal], [torso]


def sequence(kind, fps=25.0, seconds=8.0, px_per_s=0.0):
    for i in range(int(seconds * fps)):
        t = i / fps
        cx, w, h = (300.0 if kind == "run" else 640.0), 300.0, 160.0
        if t >= 2.0:
            u = min(1.0, (t - 2.0) / 1.5)
            if kind == "turn":
                w *= 1.0 - 0.6 * u  # area -> 40 %, height unchanged
            elif kind == "walk_away":
                w *= 1.0 - 0.45 * u  # distance x1.8: both sides -> 55 %
                h *= 1.0 - 0.45 * u
            elif kind == "run":
                cx = 300.0 + px_per_s * (t - 2.0)
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
        self.assertEqual(settings.TORSO_SPEED_THRESHOLD, 0.10)

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

    def test_speed_threshold_is_the_same_at_lower_fps(self):
        # 0.10 bbox widths per frame at 25 fps = 2.5 widths/s = 750 px/s for a 300 px box.
        for fps in (25.0, 12.5, 10.0, 5.0):
            fast = first_end(tracker(), sequence("run", fps=fps, px_per_s=900.0))
            slow = first_end(
                tracker(), sequence("run", fps=fps, px_per_s=600.0, seconds=2.6)
            )
            self.assertIsNotNone(fast, f"{fps} fps")
            self.assertEqual(fast[1], RAPID_MOVEMENT_REASON)
            self.assertIsNone(slow, f"{fps} fps")

    def test_original_settings_still_available(self):
        trk = tracker(
            distance_metric="area",
            area_baseline_seconds=1.0,
            area_smooth_seconds=0.5,
            area_hold_seconds=1.0,
            torso_speed_threshold=0.15,
        )
        self.assertEqual(first_end(trk, sequence("turn"))[1], AREA_REASON)

    def test_invalid_metric_rejected(self):
        with self.assertRaises(ValueError):
            tracker(distance_metric="volume")


class IntegratedEscapeTest(unittest.TestCase):
    def test_runtime_defaults_reject_turn_then_stop_audio_on_retreat(self):
        # Exercise the actual settings → scenario factory → frozen observation path.
        from animal.wildlife import WildlifeDeterrent, Phase
        from tests.test_scenario import Camera, Catalog, Player

        with self.assertLogs("animal.wildlife", level="WARNING"):
            service = WildlifeDeterrent.from_settings(player=Player(), camera=Camera())
        service.warning_catalogs = {"boar": Catalog("warning")}

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
        self.assertEqual(service.phase, Phase.PLAY_WARNING)
        aims = len(service.camera.aims)
        for i in range(50, 100):
            sample(i / 25, width=80, height=200)
        self.assertEqual(service.phase, Phase.PLAY_WARNING)  # Area=40%, height=100%.
        self.assertEqual(len(service.camera.aims), aims)
        for i in range(100, 150):
            event = sample(i / 25, width=110, height=110)
            if event.reason == AREA_REASON:
                break
        self.assertEqual(event.reason, AREA_REASON)
        self.assertEqual(service.phase, Phase.SEARCH)
        self.assertEqual(service.player.state.state, "stopped")
        self.assertEqual(len(service.player.requests), 1)
        self.assertEqual(len(service.camera.aims), aims)


if __name__ == "__main__":
    unittest.main()
