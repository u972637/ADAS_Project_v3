from pathlib import Path
import tempfile
import time
import unittest
import wave

import numpy as np
from animal.audio import WavPlayer, DryRunPlayer, AudioCatalog
from animal.camera import read_camera_frame, FRAME_W
from tools.camera_test import read_frame


class AudioTests(unittest.TestCase):
    def test_process_failure_has_exit_code_and_stderr(self):
        with tempfile.TemporaryDirectory() as temp:
            player_path = Path(temp) / "fail-player"
            player_path.write_text(
                '#!/bin/sh\necho "test device unavailable" >&2\nexit 7\n'
            )
            player_path.chmod(0o700)
            player = WavPlayer(str(player_path))
            player.play(Path("dummy.wav"))
            deadline = time.monotonic() + 2
            while player.is_playing() and time.monotonic() < deadline:
                time.sleep(0.005)
            status = player.status()
            self.assertEqual(status.state, "failed")
            self.assertEqual(status.returncode, 7)
            self.assertIn("test device unavailable", status.error)
            player.stop()
            self.assertEqual(player.status(), status)

    def test_zero_exit_is_completed_not_failed(self):
        player = WavPlayer("/bin/true")
        player.play(Path("dummy.wav"))
        deadline = time.monotonic() + 2
        while player.is_playing() and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(player.status().state, "completed")

    def test_spawn_error_is_recorded(self):
        player = WavPlayer("/bin/true")
        player.command = "/nonexistent/adas-player"
        with self.assertRaises(FileNotFoundError):
            player.play(Path("dummy.wav"))
        self.assertEqual(player.status().state, "failed")

    def test_dry_run_preserves_clip_duration(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "sound.wav"
            with wave.open(str(path), "wb") as wav:
                wav.setparams((1, 2, 8000, 0, "NONE", "not compressed"))
                wav.writeframes(b"\0\0" * 16000)
            clock = [10]
            player = DryRunPlayer(lambda: clock[0])
            player.play(path)
            clock[0] = 11.99
            self.assertTrue(player.is_playing())
            clock[0] = 12
            self.assertEqual(player.status().state, "completed")

    def test_existing_warning_catalog_has_no_repeats_within_one_pass(self):
        # assets/audio* (old single-stage "threat" clips) were removed along
        # with the other warning variants once nothing referenced them;
        # assets/warnings_pitch_eq is the only audio source left on disk.
        root = Path(__file__).resolve().parents[1] / "assets/warnings_pitch_eq"
        catalog = AudioCatalog.from_directory(root, {"boar", "water_deer"}, 1)
        for label, count in (("boar", 13), ("water_deer", 24)):
            self.assertEqual(len({catalog.next(label) for _ in range(count)}), count)


class Capture:
    def __init__(self, raw):
        self.raw = raw

    def read(self):
        return self.raw is not None, self.raw


class CameraTests(unittest.TestCase):
    def test_menu_and_runtime_share_exact_decoder(self):
        self.assertIs(read_frame, read_camera_frame)

    def test_packed_yuy2_matches_two_channel_and_correct_width(self):
        yuy = np.empty((6, FRAME_W, 2), np.uint8)
        yuy[:, :, 0] = 100
        yuy[:, :, 1] = 128
        ok, a = read_camera_frame(Capture(yuy))
        ok2, b = read_frame(Capture(yuy.reshape(6, FRAME_W * 2)))
        self.assertTrue(ok and ok2)
        self.assertEqual(b.shape, (6, FRAME_W, 3))
        np.testing.assert_array_equal(a, b)

    def test_gray_bgr_and_absent_frames(self):
        for raw in (
            np.zeros((10, 20), np.uint8),
            np.zeros((10, 20, 1), np.uint8),
            np.zeros((10, 20, 3), np.uint8),
        ):
            ok, frame = read_camera_frame(Capture(raw))
            self.assertTrue(ok)
            self.assertEqual(frame.shape, (10, 20, 3))
        self.assertEqual(read_camera_frame(Capture(None)), (False, None))
