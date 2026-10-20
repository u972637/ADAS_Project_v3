import unittest
from unittest.mock import MagicMock, patch

from animal import camera, settings


class ConfiguredCameraTests(unittest.TestCase):
    def test_pinned_device_path_wins_over_index(self):
        with (
            patch.object(settings, "CAMERA_DEVICE", "/dev/v4l/by-id/x-video-index0"),
            patch.object(settings, "CAMERA_INDEX", 7),
        ):
            self.assertEqual(
                camera.configured_camera(), "/dev/v4l/by-id/x-video-index0"
            )

    def test_falls_back_to_index_when_no_device_is_pinned(self):
        with (
            patch.object(settings, "CAMERA_DEVICE", None),
            patch.object(settings, "CAMERA_INDEX", 7),
        ):
            self.assertEqual(camera.configured_camera(), 7)

    def test_any_pinned_repo_device_is_a_capture_node_not_the_metadata_node(self):
        # "-video-index1" opens fine but never delivers frames, so pinning it
        # would look like a dead camera instead of failing loudly. (Unpinned,
        # CAMERA_DEVICE=None, is the repo default and has nothing to check.)
        if settings.CAMERA_DEVICE is None:
            self.skipTest("CAMERA_DEVICE not pinned; using CAMERA_INDEX")
        self.assertTrue(settings.CAMERA_DEVICE.startswith("/dev/v4l/by-id/"))
        self.assertTrue(settings.CAMERA_DEVICE.endswith("-video-index0"))


class OpenCameraTests(unittest.TestCase):
    def test_path_is_handed_to_opencv_v4l2_as_is(self):
        cap = MagicMock()
        cap.isOpened.return_value = True
        with patch("animal.camera.cv2.VideoCapture", return_value=cap) as opened:
            result = camera.open_camera("/dev/v4l/by-id/x-video-index0")
        self.assertIs(result, cap)
        opened.assert_called_once_with(
            "/dev/v4l/by-id/x-video-index0", camera.cv2.CAP_V4L2
        )

    def test_unopenable_source_returns_none_and_releases(self):
        cap = MagicMock()
        cap.isOpened.return_value = False
        with patch("animal.camera.cv2.VideoCapture", return_value=cap):
            self.assertIsNone(camera.open_camera("/dev/v4l/by-id/missing"))
        cap.release.assert_called_once()


class CameraHintTests(unittest.TestCase):
    def test_lists_connected_capture_nodes(self):
        found = ["/dev/v4l/by-id/b-video-index0", "/dev/v4l/by-id/a-video-index0"]
        with patch("animal.camera.glob.glob", return_value=found):
            hint = camera.camera_hint()
        self.assertIn("/dev/v4l/by-id/a-video-index0", hint)
        self.assertIn("/dev/v4l/by-id/b-video-index0", hint)
        self.assertIn("CAMERA_DEVICE", hint)

    def test_says_so_when_nothing_is_connected(self):
        with patch("animal.camera.glob.glob", return_value=[]):
            self.assertIn("없습니다", camera.camera_hint())


class ReconnectReopensTheSameSourceTests(unittest.TestCase):
    def test_capture_keeps_the_pinned_source_for_reconnects(self):
        cap = MagicMock()
        cap.read.return_value = (False, None)
        with patch("animal.camera.CameraCapture._run"):
            capture = camera.CameraCapture(cap, "/dev/v4l/by-id/x-video-index0")
        self.assertEqual(capture._camera, "/dev/v4l/by-id/x-video-index0")


if __name__ == "__main__":
    unittest.main()
