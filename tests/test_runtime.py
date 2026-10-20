import os
import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from animal.runtime import RuntimeGuard
from animal.wildlife import Phase
from tests.test_scenario import make_service, start_tracking


class RuntimeTests(unittest.TestCase):
    def test_watchdog_runs_without_inference_thread_progress(self):
        service = start_tracking(make_service())
        capture = SimpleNamespace(latest_capture_at=0.4)
        guard = RuntimeGuard(service, capture, read_stdin=False)
        try:
            deadline = time.monotonic() + 1
            while service.phase != Phase.SEARCH and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(service.last_event.reason, "camera_timeout")
            self.assertEqual(service.player.state.state, "stopped")
        finally:
            guard.close()

    def test_partial_terminal_line_cannot_block_watchdog(self):
        reader, writer = os.pipe()
        with os.fdopen(reader) as stdin:
            os.write(writer, b"par")
            service = start_tracking(make_service())
            with patch("animal.runtime.sys.stdin", stdin):
                guard = RuntimeGuard(service, SimpleNamespace(latest_capture_at=0.4))
                try:
                    deadline = time.monotonic() + 1
                    while service.phase != Phase.SEARCH and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertEqual(service.last_event.reason, "camera_timeout")
                finally:
                    guard.close()
        os.close(writer)

    def test_terminal_stop_latches_while_main_thread_is_idle(self):
        reader, writer = os.pipe()
        with os.fdopen(reader) as stdin:
            service = make_service()
            with patch("animal.runtime.sys.stdin", stdin):
                guard = RuntimeGuard(
                    service, SimpleNamespace(latest_capture_at=time.monotonic())
                )
                try:
                    os.write(writer, b"stop\n")
                    deadline = time.monotonic() + 1
                    while (
                        service.phase != Phase.STOPPED and time.monotonic() < deadline
                    ):
                        time.sleep(0.01)
                    self.assertEqual(service.phase, Phase.STOPPED)
                    self.assertEqual(service.camera.homes, 1)
                finally:
                    guard.close()
        os.close(writer)


class EntrypointTests(unittest.TestCase):
    def test_experiments_reuse_core_with_separate_person_adapter(self):
        from tools.motor_manual_control import run_wildlife_track

        with patch("animal.runtime.run") as run:
            run_wildlife_track()
        run.assert_called_once_with(enable_motor=True, start_paused=True)
        with patch("tools.person_proxy.run_person_test") as run_person:
            run_wildlife_track(person_proxy=True)
        run_person.assert_called_once_with()

    def run_main(self, *args):
        """One real runtime iteration, substituting physical I/O and YOLO only."""
        import main
        from animal.wildlife import WildlifeDeterrent
        from tests.test_scenario import detections

        created = []
        factory = WildlifeDeterrent.from_settings

        def make_service(**kwargs):
            service = factory(**kwargs)
            created.append(service)
            return service

        now = time.monotonic()
        with (
            patch("ultralytics.YOLO") as model,
            patch("animal.camera.open_camera", return_value=Mock()),
            patch("animal.camera.CameraCapture") as capture,
            patch("motor.dm4310_tracker.Tracker") as tracker,
            # Independent of the repo's MOTOR_PROCESS/CONTROL_MODE defaults:
            # this test mocks motor.dm4310_tracker.Tracker specifically and
            # cannot pickle a Mock across RemoteTracker's child process
            # boundary, nor does it mock StepTracker.
            patch("animal.settings.MOTOR_PROCESS", False),
            patch("motor.config.CONTROL_MODE", "trajectory"),
            patch("motor.config.TRAJECTORY_GO_CENTER_DURATION", 0),
            patch("animal.runtime.RuntimeGuard") as guard,
            patch(
                "animal.wildlife.WildlifeDeterrent.from_settings",
                side_effect=make_service,
            ),
            patch(
                "animal.inference.infer_detections",
                return_value=((1000, 1000), detections()),
            ) as infer,
            patch("builtins.print"),
        ):
            model.return_value.names = {0: "boar", 1: "deer", 2: "torso"}
            capture.return_value.read_new.return_value = (True, object(), 1, now)
            guard.return_value.quit.is_set.side_effect = [False, True]
            tracker.return_value.health.return_value = {"ok": True, "fault": None}
            main.main(["--headless", *args])
            infer.assert_called_once()
            capture.return_value.release.assert_called_once()
            return created[0], tracker

    def test_main_alone_enables_motor_and_complete_scenario(self):
        from animal.audio import WavPlayer

        service, tracker = self.run_main()
        tracker.assert_called_once_with()
        tracker.return_value.go_default.assert_called_once()
        tracker.return_value.close.assert_called_once()
        self.assertIsInstance(service.player, WavPlayer)
        # A production frame was consumed before final cleanup.
        self.assertIsNotNone(service._last_result_at)

    def test_main_start_paused_does_not_consume_detection_for_aiming(self):
        service, tracker = self.run_main("--start-paused")
        tracker.assert_called_once_with()
        tracker.return_value.update.assert_not_called()
        self.assertIsNone(service._last_result_at)

    def test_main_dry_run_disables_real_motor_and_audio(self):
        from animal.audio import DryRunPlayer
        from animal.aiming import SimulatedCameraController

        service, tracker = self.run_main("--dry-run")
        tracker.assert_not_called()
        self.assertIsInstance(service.camera, SimulatedCameraController)
        self.assertIsInstance(service.player, DryRunPlayer)

    def test_menu_exit_without_selection_never_opens_motor(self):
        from tools import motor_manual_control as menu

        with (
            patch("motor.dm4310_tracker.Tracker") as tracker,
            patch("builtins.input", return_value="0"),
            patch("builtins.print"),
        ):
            menu.main()
        tracker.assert_not_called()
