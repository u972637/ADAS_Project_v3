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
        # The last two FILTERED (3-frame average) aim points, normalized 0..1
        # (not the raw torso centers): the basis for the orange extrapolation.
        self._last_point: tuple[float, float] | None = None
        self._prev_point: tuple[float, float] | None = None
        # Orange state: number of lost frames so far and the current virtual
        # target (normalized). 0 / None while a real torso is being tracked.
        self._orange_frames = 0
        self._orange_point: tuple[float, float] | None = None
        # What the filtered points come from: "torso" or "bbox" (the center of a
        # bbox followed because no torso matched). Mixing the two in one average
        # would drag the target between them, so a change restarts the filter.
        self._source: str | None = None

    def reset(self):
        self._samples.clear()
        self._outlier = None
        self._last_point = None
        self._prev_point = None
        self._orange_frames = 0
        self._orange_point = None
        self._source = None

    @property
    def orange_point(self) -> tuple[float, float] | None:
        """The current orange (virtual) target, normalized 0..1, or None while a
        real torso is tracked. The tracker uses it as the center to look for the
        torso around while the motor is chasing the virtual target."""
        return self._orange_point

    @property
    def orange_point_px(self) -> tuple[float, float] | None:
        """The current orange (virtual) target in frame pixel coordinates, or
        None while a real torso is being tracked / nothing is being tracked.
        It is exactly the point the motor was last commanded toward during the
        orange state, so the display and the motor always agree."""
        if self._orange_point is None:
            return None
        return (
            self._orange_point[0] * self.tracker_size[0],
            self._orange_point[1] * self.tracker_size[1],
        )

    @staticmethod
    def _decay_for(x: float, y: float) -> float:
        """Per-frame pull n for a normalized point: chosen by its distance from the
        screen center (0 = center, 1 = edge, larger axis), see settings.py."""
        distance = max(abs(x - 0.5), abs(y - 0.5)) / 0.5
        if distance >= settings.ORANGE_ZONE_OUTER_FRACTION:
            return settings.ORANGE_DECAY_OUTER
        if distance < settings.ORANGE_ZONE_INNER_FRACTION:
            return settings.ORANGE_DECAY_INNER
        return settings.ORANGE_DECAY_MIDDLE

    def missing(self):
        """The tracked torso is gone this frame but the lock is alive: advance
        the orange (virtual) target and command the motor toward it.

        First orange frame: extrapolate the last two filtered points one step
        (2*last - previous, i.e. last + velocity), clamped inside the frame; with
        only one point there is no velocity, so it stays put. Every later
        orange frame pulls the previous orange point toward the screen center
        C=(0.5, 0.5) by the zone's ORANGE_DECAY_* (in normalized coordinates; the
        center is NOT the (0,0) corner). If nothing was ever aimed at (plain
        search) there is nothing to extrapolate."""
        if self._last_point is None:
            self.reset()
            return
        if self._orange_frames == 0:
            lx, ly = self._last_point
            px, py = self._prev_point if self._prev_point is not None else (lx, ly)
            point = (
                min(max(2 * lx - px, 0.0), 1.0),
                min(max(2 * ly - py, 0.0), 1.0),
            )
        else:
            ox, oy = self._orange_point
            keep = 1.0 - self._decay_for(ox, oy)
            point = (0.5 + keep * (ox - 0.5), 0.5 + keep * (oy - 0.5))
        self._orange_point = point
        self._orange_frames += 1
        self.tracker.update(point[0] * self.tracker_size[0], point[1] * self.tracker_size[1])

    def aim(self, target, frame_size, timestamp, source="torso"):
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
        if self._source is not None and source != self._source:
            # Torso <-> bbox center: a different kind of point (the bbox center
            # is not where the torso is). Restart the filter from the new one.
            self._samples.clear()
            self._outlier = None
            self._last_point = None
        self._source = source
        if self._orange_frames:
            # A real torso is back after orange. Its position is unrelated to
            # the pre-loss samples, so restart the filter from it: otherwise the
            # old samples would be averaged in, and a jump past the outlier gate
            # would be rejected. Forget the pre-loss points too, so the next
            # extrapolation never uses a velocity measured across the gap.
            self._samples.clear()
            self._outlier = None
            self._last_point = None
            self._orange_frames = 0
            self._orange_point = None
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
            # The jump itself is not motion: never extrapolate across it.
            self._last_point = None
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
        self._prev_point = self._last_point
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
