import os
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from animal.runtime import RuntimeGuard
from animal.wildlife import Phase
from tests.test_scenario import make_service, start_warning


class RuntimeTests(unittest.TestCase):
    def test_watchdog_runs_without_inference_thread_progress(self):
        service = start_warning(make_service())
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
            service = start_warning(make_service())
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
    def test_main_and_menu_use_same_runtime(self):
        import main
        import animal.runtime
        from tools.motor_manual_control import run_wildlife_track

        self.assertIs(main.run, animal.runtime.run)
        with patch("animal.runtime.run") as run:
            run_wildlife_track(person_proxy=True)
        run.assert_called_once_with(
            person_proxy=True, enable_motor=True, start_paused=True
        )

    def test_menu_exit_without_selection_never_opens_motor(self):
        from tools import motor_manual_control as menu

        with (
            patch.object(menu, "Tracker") as tracker,
            patch("builtins.input", return_value="0"),
            patch("builtins.print"),
        ):
            menu.main()
        tracker.assert_not_called()
