import unittest
from pathlib import Path

from animal.audio import PlaybackStatus
from animal.perception import Detection, EscapeTracker
from animal.wildlife import WildlifeDeterrent, Phase


class Player:
    def __init__(self):
        self.requests = []
        self.state = PlaybackStatus("idle")
        self.stops = 0

    def play(self, file):
        self.requests.append(file)
        self.state = PlaybackStatus("playing", file)

    def status(self):
        return self.state

    def finish(self):
        self.state = PlaybackStatus("completed", self.state.file, 0)

    def stop(self):
        self.stops += 1
        self.state = PlaybackStatus("stopped", self.state.file)


class Catalog:
    def __init__(self, stage):
        self.stage = stage

    def next(self, label):
        return Path(f"{self.stage}/{label}.wav")


class Camera:
    """Stands in for the whole TrackerCameraController: aim() reports a
    settable offset directly (0.0 = dead center), bypassing the real
    averaging/outlier logic — tests control it explicitly via self.offset."""

    def __init__(self):
        self.aims = []
        self.holds = self.homes = 0
        self.offset = 0.0
        self.ok = True

    def aim(self, target, size, timestamp):
        self.aims.append((target, timestamp))
        return self.offset

    def reset(self):
        pass

    def missing(self):
        pass

    def hold(self):
        self.holds += 1

    def home(self, duration=None, yaw_deg=None, pitch_deg=None):
        self.homes += 1

    def health(self):
        return {"ok": self.ok, "fault": None if self.ok else "test fault"}


def make_service(alarm=True):
    escape = EscapeTracker(
        track_lost_seconds=2,
        reselect_grace_seconds=1,
        area_baseline_seconds=0.1,
        area_smooth_seconds=0.1,
        area_hold_seconds=0.2,
        area_ratio_threshold=0.5,
        edge_margin_px=2,
    )
    camera, player = Camera(), Player()
    return WildlifeDeterrent(
        class_map={0: "boar", 1: "water_deer"},
        expected_names={0: "boar", 1: "deer"},
        catalogs={"boar": Catalog("alarm")} if alarm else {},
        player=player,
        escape_tracker=escape,
        camera=camera,
    )


def detections(shift=0, track_id=1, cls=0, torso=True, scale=1):
    width = 200 * scale
    cx = 500 + shift
    result = [
        Detection(
            cls, 0.9, (cx - width / 2, 400, cx + width / 2, 400 + width), track_id
        )
    ]
    if torso:
        result.append(Detection(2, 0.9, (cx - width / 4, 430, cx + width / 4, 480)))
    return result


def feed(s, t, ds=None, **kwargs):
    return s.process_detections(
        (1200, 1000), detections() if ds is None else ds, t, **kwargs
    )


def start_tracking(s):
    """One confident, centered frame is enough: selection is one-frame and
    playback starts as soon as a confident target is picked (no center
    requirement), so this starts it on the very first feed."""
    feed(s, 0.0)
    return s


class ScenarioTests(unittest.TestCase):
    def test_enters_tracking_and_starts_playback_once_centered(self):
        s = make_service()
        event = feed(s, 0.0)
        self.assertEqual(s.phase, Phase.TRACKING)
        self.assertTrue(s.playing)
        self.assertEqual(s.player.requests, [Path("alarm/boar.wav")])
        self.assertEqual(event.reason, "playing")

    def test_playback_needs_centered_aim_to_start_but_exit_band_to_stop(self):
        s = make_service()
        s.camera.offset = 0.40  # far from center -- must not start yet
        feed(s, 0.0)
        self.assertFalse(s.playing, "should NOT start outside the enter band")
        s.camera.offset = 0.03  # inside PLAYBACK_ENTER_FRACTION (0.05)
        feed(s, 0.1)
        self.assertTrue(s.playing, "should START once centered within the enter band")
        s.camera.offset = 0.10
        feed(s, 0.2)
        self.assertTrue(s.playing, "should KEEP playing inside the exit band")
        s.camera.offset = 0.20
        feed(s, 0.3)
        self.assertFalse(s.playing, "should STOP only once past the exit band")
        self.assertEqual(s.player.state.state, "stopped")

    def test_motor_keeps_tracking_continuously_while_playing(self):
        s = start_tracking(make_service())
        aims = len(s.camera.aims)
        feed(s, 0.1, detections(shift=5))
        feed(s, 0.2, detections(shift=10))
        self.assertGreater(len(s.camera.aims), aims, "aim() must keep being called")

    def test_continuous_playback_advances_clips_after_a_gap(self):
        s = start_tracking(make_service())
        self.assertEqual(len(s.player.requests), 1)
        s.player.finish()
        feed(s, 0.1)  # clip completed this frame: gap timer starts, no new clip yet
        self.assertEqual(len(s.player.requests), 1)
        feed(s, 0.1 + 0.31)  # past PLAYBACK_GAP_SECONDS=0.3
        self.assertEqual(len(s.player.requests), 2)
        self.assertTrue(s.playing)

    def test_missing_species_catalog_never_plays_but_keeps_tracking(self):
        s = make_service(alarm=False)
        event = feed(s, 0.0)
        self.assertEqual(s.phase, Phase.TRACKING)
        self.assertFalse(s.playing)
        self.assertEqual(s.player.requests, [])
        self.assertEqual(event.reason, "tracking")

    def test_area_reduction_terminates_while_tracking(self):
        s = start_tracking(make_service())
        event = None
        for t in (0.5, 0.6, 0.7, 0.8, 0.9, 1.0):
            event = feed(s, t, detections(scale=0.5))
            if "bbox_area_reduced" in event.reason:
                break
        self.assertIn("bbox_area_reduced", event.reason)
        self.assertEqual(s.player.state.state, "stopped")

    def test_short_missing_holds_then_confirmed_loss_stops_audio(self):
        s = start_tracking(make_service())
        aims = len(s.camera.aims)
        feed(s, 0.5, [])
        feed(s, 1.5, [])
        self.assertEqual(s.player.state.state, "playing")
        self.assertEqual(len(s.camera.aims), aims)
        event = feed(s, 2.5, [])
        self.assertEqual(event.reason, "tracking_lost")
        self.assertEqual(s.player.state.state, "stopped")
        self.assertEqual(s.phase, Phase.SEARCH)
        feed(s, 2.6, detections(track_id=5))
        self.assertEqual(s.phase, Phase.TRACKING)
        self.assertTrue(s.playing)

    def test_torso_missing_has_no_animal_center_fallback(self):
        s = make_service()
        event = None
        for i in range(6):
            event = feed(s, i * 0.1, detections(torso=False))
        self.assertEqual(s.camera.aims, [])
        self.assertEqual(event.reason, "torso_missing_hold")
        self.assertEqual(s.player.requests, [])

    def test_stop_latches_until_resume_and_rejects_inflight_results(self):
        s = start_tracking(make_service())
        old = s.generation
        s.stop()
        aims = len(s.camera.aims)
        feed(s, 0.6)
        self.assertEqual(s.phase, Phase.STOPPED)
        self.assertEqual(s.camera.homes, 1)
        self.assertEqual(len(s.camera.aims), aims)
        self.assertEqual(s.player.state.state, "stopped")
        s.resume()
        feed(s, 0.7, generation=old)
        self.assertIsNone(s.escape_tracker.target_id)
        feed(s, 0.8, generation=s.generation)
        self.assertEqual(s.phase, Phase.TRACKING)
        self.assertTrue(s.playing)

    def test_no_camera_frame_watchdog_stops_audio_without_process_call(self):
        s = start_tracking(make_service())
        event = s.tick(4, latest_camera_at=0.4)
        self.assertEqual(event.reason, "camera_timeout")
        self.assertEqual(s.player.state.state, "stopped")
        self.assertIsNone(s.escape_tracker.target_id)

    def test_blocked_inference_with_fresh_camera_is_distinguished(self):
        s = start_tracking(make_service())
        self.assertEqual(s.tick(4, latest_camera_at=4).reason, "perception_timeout")
        self.assertEqual(s.player.state.state, "stopped")

    def test_stale_result_cannot_aim(self):
        s = make_service()
        event = feed(s, 0, now=3)
        self.assertEqual(event.reason, "stale_detection_ignored")
        self.assertEqual(s.camera.aims, [])

    def test_audio_failure_does_not_escalate_and_allows_explicit_retry(self):
        s = start_tracking(make_service())
        s.player.state = PlaybackStatus(
            "failed", Path("alarm/boar.wav"), 1, "device unavailable"
        )
        s.tick(0.5)
        self.assertEqual(s.phase, Phase.FAULT)
        self.assertIn("device unavailable", s.last_error)
        feed(s, 30)
        self.assertEqual(len(s.player.requests), 1)
        s.resume()
        for t in (31, 31.1, 31.2, 31.3, 31.4):
            feed(s, t)
        self.assertEqual(len(s.player.requests), 2)
        self.assertEqual(s.player.requests[-1], Path("alarm/boar.wav"))

    def test_motor_fault_stops_audio_and_blocks_resume(self):
        s = start_tracking(make_service())
        s.camera.ok = False
        s.tick(0.5)
        self.assertEqual(s.phase, Phase.FAULT)
        self.assertEqual(s.player.state.state, "stopped")
        s.resume()
        self.assertEqual(s.phase, Phase.FAULT)

    def test_species_change_cannot_inherit_locked_target(self):
        s = start_tracking(make_service())
        event = None
        for t in (0.5, 2.5):
            event = feed(s, t, detections(cls=1))
        self.assertEqual(event.reason, "tracking_lost")

    def test_no_motor_does_not_claim_convergence_or_play(self):
        s = make_service()
        s.camera = None
        event = None
        for t in (0, 0.1, 0.2, 1, 2):
            event = feed(s, t)
        self.assertEqual(event.reason, "motor_disabled")
        self.assertEqual(s.player.requests, [])


if __name__ == "__main__":
    unittest.main()
