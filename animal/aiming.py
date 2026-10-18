"""Filter torso jitter and report convergence; never decides when to play audio."""

from collections import deque
import math

from animal import settings


class TrackerCameraController:
    def __init__(self, tracker, tracker_size):
        if min(tracker_size) <= 0:
            raise ValueError("tracker frame dimensions must be positive")
        self.tracker = tracker
        self.tracker_size = tracker_size
        self._samples = deque(maxlen=settings.AIM_AVERAGE_FRAMES)
        self._outlier = None
        self._stable_since = None

    def reset(self):
        self._samples.clear()
        self._outlier = self._stable_since = None

    def missing(self):
        # Hold last motor goal across short detector gaps, but require fresh convergence.
        self.reset()

    def aim(self, target, frame_size, timestamp):
        width, height = frame_size
        if min(width, height) <= 0:
            raise ValueError("frame dimensions must be positive")
        x, y = target.center
        point = (x / width, y / height)
        if (
            self._samples
            and math.dist(point, self._samples[-1]) > settings.AIM_MAX_JUMP_FRACTION
        ):
            # Reject an isolated jump. Two nearby consecutive jumps allow real relocation.
            if (
                self._outlier is None
                or math.dist(point, self._outlier) > settings.AIM_MAX_JUMP_FRACTION / 2
            ):
                self._outlier = point
                self._stable_since = None
                return False
            self._samples.clear()
        self._outlier = None
        self._samples.append(point)
        sx = sum(p[0] for p in self._samples) / len(self._samples)
        sy = sum(p[1] for p in self._samples) / len(self._samples)
        # Raw centered measurements must cancel an older, off-center goal immediately.
        if abs(point[0] - 0.5) <= settings.AIM_DEADBAND_FRACTION:
            sx = 0.5
        if abs(point[1] - 0.5) <= settings.AIM_DEADBAND_FRACTION:
            sy = 0.5
        self.tracker.update(sx * self.tracker_size[0], sy * self.tracker_size[1])
        centered = all(
            abs(v - 0.5) <= settings.AIM_TOLERANCE_FRACTION for v in (*point, sx, sy)
        )
        slow = all(
            abs(self.tracker.get_state(axis)[1]) <= settings.AIM_MAX_VELOCITY_RAD_S
            for axis in ("yaw", "pitch")
        )
        if not centered or not slow or len(self._samples) < settings.AIM_AVERAGE_FRAMES:
            self._stable_since = None
            return False
        if self._stable_since is None:
            self._stable_since = timestamp
        return timestamp - self._stable_since >= settings.AIM_SETTLE_SECONDS

    def hold(self):
        self.reset()
        self.tracker.hold()

    def home(self):
        self.reset()
        self.tracker.go_default()

    def frozen_ready(self):
        """Do not collect escape baselines while the reference is still braking."""
        return self.tracker.motion_complete() and all(
            abs(self.tracker.get_state(axis)[1]) <= settings.AIM_MAX_VELOCITY_RAD_S
            for axis in ("yaw", "pitch")
        )

    def health(self):
        return self.tracker.health()


class SimulatedCameraController(TrackerCameraController):
    """Explicit dry-run adapter: no CAN traffic; the real image must still center."""

    class Motor:
        def update(self, *args):
            pass

        def get_state(self, axis):
            return (0.0, 0.0, 0.0)

        def hold(self):
            pass

        def go_default(self):
            pass

        def motion_complete(self):
            return True

        def health(self):
            return {"ok": True, "fault": None}

    def __init__(self):
        super().__init__(self.Motor(), (2592, 1944))
