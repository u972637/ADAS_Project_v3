"""Arducam capture for the wildlife detection loop."""

from __future__ import annotations

import threading
import time

import cv2

FRAME_W, FRAME_H = 2592, 1944
CAMERA_FOURCC = "YUY2"


def open_camera(camera_index: int):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*CAMERA_FOURCC))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 25)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def read_camera_frame(cap):
    """Return a BGR frame from YUY2, grayscale, or an already converted source."""
    try:
        ready, raw = cap.read()
        if not ready or raw is None:
            return False, None
        if raw.ndim == 3 and raw.shape[2] == 2:
            frame = cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)
        elif raw.ndim == 2 and raw.shape[1] == FRAME_W * 2:
            frame = cv2.cvtColor(
                raw.reshape(raw.shape[0], FRAME_W, 2), cv2.COLOR_YUV2BGR_YUY2
            )
        elif raw.ndim == 2:
            frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        elif raw.ndim == 3 and raw.shape[2] == 1:
            frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        elif raw.ndim == 3 and raw.shape[2] == 3:
            frame = raw
        else:
            raise ValueError(f"지원하지 않는 카메라 프레임 형식: {raw.shape}")
        return True, frame
    except (cv2.error, ValueError) as error:
        print(f"[WARN] 카메라 프레임 처리 실패: {error}")
        return False, None


class CameraCapture:
    """Keep only the newest frame while YOLO runs on another thread."""

    def __init__(self, cap, camera_index: int):
        self._cap = cap
        self._camera_index = camera_index
        self._frame = None
        self._sequence = 0
        self._captured_at = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        failures = 0
        while not self._stop.is_set():
            ready, frame = read_camera_frame(self._cap)
            if ready:
                failures = 0
                with self._lock:
                    self._frame = frame
                    self._sequence += 1
                    self._captured_at = time.monotonic()
                continue

            if self._stop.is_set():
                break
            failures += 1
            if failures < 2:
                time.sleep(0.01)
                continue

            print("[WARN] 카메라 읽기 실패 — 재연결 시도")
            self._cap.release()
            while not self._stop.is_set():
                if self._stop.wait(1.5):
                    break
                cap = open_camera(self._camera_index)
                if cap is not None:
                    self._cap = cap
                    for _ in range(10):
                        read_camera_frame(cap)
                    failures = 0
                    print("[OK] 카메라 재연결 성공")
                    break

    def read_new(self, last_sequence: int):
        """Return each captured frame at most once with its monotonic time."""
        with self._lock:
            if self._sequence == last_sequence:
                return False, None, last_sequence, None
            return True, self._frame, self._sequence, self._captured_at

    def release(self):
        self._stop.set()
        self._cap.release()
        self._thread.join(timeout=2.0)
