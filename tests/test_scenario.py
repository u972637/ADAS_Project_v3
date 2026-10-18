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
    def __init__(self):
        self.aims = []
        self.holds = self.homes = 0
        self.converged = True
        self.ok = True
        self.braking_done = True

    def aim(self, target, size, timestamp):
        self.aims.append((target, timestamp))
        return self.converged

    def reset(self):
        pass

    def missing(self):
        pass

    def hold(self):
        self.holds += 1

    def home(self):
        self.homes += 1

    def frozen_ready(self):
        return self.braking_done

    def health(self):
        return {"ok": self.ok, "fault": None if self.ok else "test fault"}


def make_service(warnings=True):
    escape = EscapeTracker(
        track_lost_seconds=2,
        reselect_grace_seconds=1,
        area_baseline_seconds=0.1,
        area_smooth_seconds=0.1,
        area_hold_seconds=0.2,
        area_ratio_threshold=0.5,
        edge_margin_px=2,
        torso_window_size=3,
        torso_speed_threshold=0.15,
    )
    camera, player = Camera(), Player()
    return WildlifeDeterrent(
        class_map={0: "boar", 1: "water_deer"},
        expected_names={0: "boar", 1: "deer"},
        catalog=Catalog("threat"),
        warning_catalogs={"boar": Catalog("warning")} if warnings else {},
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


def start_warning(s):
    for t in (0, 0.1, 0.2, 0.3, 0.4):
        feed(s, t)
    return s


class ScenarioTests(unittest.TestCase):
    def test_frames_captured_before_braking_finished_cannot_seed_baseline(self):
        service = make_service()
        service.camera.braking_done = False
        for t in (0, 0.1, 0.2):
            feed(service, t)
        service.camera.braking_done = True
        # Inference completes at 1.0, but this image predates stationary feedback.
        feed(service, 0.3, now=1.0)
        feed(service, 0.5, now=1.1)
        self.assertFalse(service.escape_tracker.baseline_ready)
        self.assertEqual(service.player.requests, [])
        for t in (1.2, 1.4):
            feed(service, t, now=t + 0.05)
        self.assertEqual(service.phase, Phase.PLAY_WARNING)

    def test_braking_delays_escape_baseline_and_audio_until_stationary(self):
        service = make_service()
        service.camera.braking_done = False
        for i in range(15):
            feed(service, i / 10)
        self.assertEqual(service.phase, Phase.BASELINE_WARNING)
        self.assertFalse(service.escape_tracker.baseline_ready)
        self.assertEqual(service.player.requests, [])
        service.camera.braking_done = True
        for i in range(15, 18):
            feed(service, i / 10)
        self.assertEqual(service.phase, Phase.PLAY_WARNING)

    def test_warning_observe_five_seconds_then_reaim_before_threat(self):
        s = start_warning(make_service())
        self.assertEqual(s.phase, Phase.PLAY_WARNING)
        self.assertEqual(s.player.requests, [Path("warning/boar.wav")])
        aims = len(s.camera.aims)
        feed(s, 0.5, detections(shift=10))
        self.assertEqual(len(s.camera.aims), aims)
        s.player.finish()
        feed(s, 1)
        self.assertEqual(s.phase, Phase.OBSERVE_WARNING)
        feed(s, 5.99)
        self.assertEqual(s.phase, Phase.OBSERVE_WARNING)
        self.assertEqual(len(s.camera.aims), aims)
        feed(s, 6)
        self.assertEqual(s.phase, Phase.AIM_THREAT)
        self.assertEqual(len(s.player.requests), 1)
        feed(s, 6.1)
        self.assertEqual(len(s.camera.aims), aims + 1)
        for t in (6.2, 6.3):
            feed(s, t)
        self.assertEqual(s.phase, Phase.PLAY_THREAT)
        self.assertEqual(s.player.requests[-1], Path("threat/boar.wav"))
        s.player.finish()
        feed(s, 7)
        feed(s, 30)
        self.assertEqual(s.phase, Phase.OBSERVE_THREAT)
        self.assertEqual(len(s.player.requests), 2)
        self.assertEqual(len(s.camera.aims), aims + 1)

    def test_missing_warning_never_substitutes_threat(self):
        s = start_warning(make_service(warnings=False))
        self.assertEqual(s.phase, Phase.WAIT_WARNING_ASSET)
        feed(s, 60)
        self.assertEqual(s.player.requests, [])

    def test_rapid_exit_stops_sound_and_never_aims_or_plays_on_exit_frame(self):
        s = start_warning(make_service())
        count = len(s.camera.aims)
        event = feed(s, 0.5, detections(shift=200))
        self.assertIn("rapid_movement_detected", event.reason)
        self.assertEqual(s.phase, Phase.SEARCH)
        self.assertEqual(s.player.state.state, "stopped")
        self.assertEqual(len(s.camera.aims), count)
        self.assertEqual(len(s.player.requests), 1)
        feed(s, 0.6, detections(shift=200))
        self.assertIsNone(s.escape_tracker.target_id)

    def test_moving_camera_during_aim_does_not_trigger_escape(self):
        s = make_service()
        s.camera.converged = False
        for i, shift in enumerate((0, 100, 200, 300, 400, 300, 200)):
            event = feed(s, i * 0.1, detections(shift=shift))
        self.assertEqual(s.phase, Phase.AIM_WARNING)
        self.assertNotIn("rapid", event.reason)

    def test_area_reduction_in_frozen_stage_terminates(self):
        s = start_warning(make_service())
        for t in (0.5, 0.6, 0.7, 0.8, 0.9, 1.0):
            event = feed(s, t, detections(scale=0.5))
            if "bbox_area_reduced" in event.reason:
                break
        self.assertIn("bbox_area_reduced", event.reason)
        self.assertEqual(s.player.state.state, "stopped")

    def test_short_missing_holds_then_confirmed_loss_stops_audio(self):
        s = start_warning(make_service())
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
        self.assertEqual(s.phase, Phase.CONFIRM)

    def test_torso_missing_has_no_animal_center_fallback(self):
        s = make_service()
        for i in range(6):
            event = feed(s, i * 0.1, detections(torso=False))
        self.assertEqual(s.camera.aims, [])
        self.assertEqual(event.reason, "torso_missing_hold")
        self.assertEqual(s.player.requests, [])

    def test_stop_latches_until_resume_and_rejects_inflight_results(self):
        s = start_warning(make_service())
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
        self.assertEqual(s.phase, Phase.CONFIRM)

    def test_no_camera_frame_watchdog_stops_audio_without_process_call(self):
        s = start_warning(make_service())
        event = s.tick(4, latest_camera_at=0.4)
        self.assertEqual(event.reason, "camera_timeout")
        self.assertEqual(s.player.state.state, "stopped")
        self.assertIsNone(s.escape_tracker.target_id)

    def test_blocked_inference_with_fresh_camera_is_distinguished(self):
        s = start_warning(make_service())
        self.assertEqual(s.tick(4, latest_camera_at=4).reason, "perception_timeout")
        self.assertEqual(s.player.state.state, "stopped")

    def test_stale_result_cannot_aim(self):
        s = make_service()
        event = feed(s, 0, now=3)
        self.assertEqual(event.reason, "stale_detection_ignored")
        self.assertEqual(s.camera.aims, [])

    def test_audio_failure_does_not_escalate_and_allows_explicit_retry(self):
        s = start_warning(make_service())
        s.player.state = PlaybackStatus(
            "failed", Path("warning/boar.wav"), 1, "device unavailable"
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
        self.assertEqual(s.player.requests[-1], Path("warning/boar.wav"))

    def test_motor_fault_stops_audio_and_blocks_resume(self):
        s = start_warning(make_service())
        s.camera.ok = False
        s.tick(0.5)
        self.assertEqual(s.phase, Phase.FAULT)
        self.assertEqual(s.player.state.state, "stopped")
        s.resume()
        self.assertEqual(s.phase, Phase.FAULT)

    def test_species_change_cannot_inherit_locked_target(self):
        s = start_warning(make_service())
        for t in (0.5, 2.5):
            event = feed(s, t, detections(cls=1))
        self.assertEqual(event.reason, "tracking_lost")

    def test_no_motor_does_not_claim_convergence_or_play(self):
        s = make_service()
        s.camera = None
        for t in (0, 0.1, 0.2, 1, 2):
            event = feed(s, t)
        self.assertEqual(event.reason, "motor_disabled")
        self.assertEqual(s.player.requests, [])


if __name__ == "__main__":
    unittest.main()
