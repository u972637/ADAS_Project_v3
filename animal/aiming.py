"""Filter torso jitter and report the smoothed off-center distance; never
decides when to play audio — that's a hysteresis band owned by wildlife.py."""

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
        self._last_point: tuple[float, float] | None = None

    def reset(self):
        self._samples.clear()
        self._outlier = None
        self._last_point = None

    def _held_point(self) -> tuple[float, float] | None:
        """Last smoothed point, clamped per axis to within
        HELD_AIM_MAX_OFFSET_FRACTION of screen center. The motor has been
        steering that point toward center ever since it was sent, and
        update() is relative to the motor's CURRENT position, so resending
        the stale off-center offset as-is would keep pushing the motor
        further every tick instead of holding it."""
        if self._last_point is None:
            return None
        limit = settings.HELD_AIM_MAX_OFFSET_FRACTION
        sx, sy = self._last_point
        return (
            min(max(sx, 0.5 - limit), 0.5 + limit),
            min(max(sy, 0.5 - limit), 0.5 + limit),
        )

    @property
    def held_aim_point_px(self) -> tuple[float, float] | None:
        """The point `missing()` keeps resending when no torso matches this
        frame (center-clamped, see _held_point), in frame pixel coordinates.
        None if nothing has ever been aimed at. Display-only accessor (e.g.
        drawing a dot for "motor target, no torso this frame" distinct from
        a live match)."""
        held = self._held_point()
        if held is None:
            return None
        return held[0] * self.tracker_size[0], held[1] * self.tracker_size[1]

    def missing(self):
        """No torso matched this frame (short detector/fallback gap, same
        locked target). Keep resending a held point near screen center
        instead of going quiet -- the motor keeps an active, fresh command
        through the gap, and the averaging window doesn't need to refill
        from scratch once a real detection resumes. If there's no prior
        point (nothing was ever aimed at, e.g. plain search), there's
        nothing to hold."""
        held = self.held_aim_point_px
        if held is not None:
            self.tracker.update(*held)
        else:
            self.reset()

    def aim(self, target, frame_size, timestamp):
        """Send the smoothed target to the motor every call (motor keeps
        tracking continuously). Returns the max per-axis fraction of the
        smoothed point from center (0 = dead center), or None when there
        isn't yet a usable reading (outlier rejected / still filling the
        averaging window) — the caller should leave its playback state
        unchanged on None rather than treat it as "far off center"."""
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
                return None
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
        self._last_point = (sx, sy)
        if len(self._samples) < settings.AIM_AVERAGE_FRAMES:
            return None
        return max(abs(sx - 0.5), abs(sy - 0.5))

    def hold(self):
        self.reset()
        self.tracker.hold()

    def home(self, duration=None, yaw_deg=None, pitch_deg=None):
        self.reset()
        self.tracker.go_default(duration=duration, yaw_deg=yaw_deg, pitch_deg=pitch_deg)

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

        def go_default(self, duration=None, yaw_deg=None, pitch_deg=None):
            pass

        def motion_complete(self):
            return True

        def health(self):
            return {"ok": True, "fault": None}

    def __init__(self):
        super().__init__(self.Motor(), (2592, 1944))
