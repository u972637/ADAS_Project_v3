"""Local, bounded two-axis reference generation; no I/O and no waypoint service.

Only a changed destination triggers a calculation. Every control tick samples
the active trajectory at monotonic wall time. Replanning preserves the previous
command's position, velocity and acceleration, rather than restarting at rest.
"""

from dataclasses import dataclass
import math
import time

from ruckig import ControlInterface, InputParameter, Ruckig, Trajectory

from motor.config import (
    AXES,
    CONTROL_FREQ,
    RANGE_RAD,
    MOTION_MAX_VELOCITY,
    MOTION_MAX_ACCELERATION,
    MOTION_MAX_JERK,
)


class MotionError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotionCommand:
    # None means decelerate to zero velocity, then hold the resulting position.
    position: tuple[float, float] | None
    minimum_duration: float = 0.0


class MotionPlanner:
    def __init__(self, position, velocity=(0.0, 0.0)):
        if (
            len(position) != 2
            or len(velocity) != 2
            or not all(math.isfinite(x) for x in (*position, *velocity))
        ):
            raise MotionError("initial motion state must be finite")
        self._generator = Ruckig(2, 1.0 / CONTROL_FREQ)
        self._input = InputParameter(2)
        self._input.max_velocity = [MOTION_MAX_VELOCITY[a] for a in AXES]
        self._input.max_acceleration = [MOTION_MAX_ACCELERATION[a] for a in AXES]
        self._input.max_jerk = [MOTION_MAX_JERK[a] for a in AXES]
        self._input.target_velocity = [0.0, 0.0]
        self._input.target_acceleration = [0.0, 0.0]
        self._reference = (list(position), list(velocity), [0.0, 0.0])
        self._trajectory = Trajectory(2)
        self._command = None
        self._started_at = None
        self._last_time = None
        self.finished = False
        self.replans = 0
        self.last_compute_seconds = self.max_compute_seconds = 0.0
        self.last_replan_seconds = self.max_replan_seconds = 0.0

    def _sample(self, now):
        if self._started_at is not None:
            elapsed = max(0.0, now - self._started_at)
            self._reference = self._trajectory.at_time(
                min(elapsed, self._trajectory.duration)
            )
            self.finished = elapsed >= self._trajectory.duration
        return self._reference

    def step(self, now, command):
        started = time.perf_counter()
        try:
            if not math.isfinite(now) or (
                self._last_time is not None and now < self._last_time
            ):
                raise MotionError("trajectory clock must be finite and monotonic")
            self._last_time = now
            position, velocity, acceleration = self._sample(now)
            if command != self._command:
                self._plan(now, command, position, velocity, acceleration)
            return self._sample(now)
        finally:
            self.last_compute_seconds = time.perf_counter() - started
            self.max_compute_seconds = max(
                self.max_compute_seconds, self.last_compute_seconds
            )

    def _plan(self, now, command, position, velocity, acceleration):
        started = time.perf_counter()
        if not math.isfinite(command.minimum_duration) or command.minimum_duration < 0:
            raise MotionError("minimum duration must be finite and nonnegative")
        inp = self._input
        inp.current_position = position
        inp.current_velocity = velocity
        inp.current_acceleration = acceleration
        inp.minimum_duration = command.minimum_duration or None
        if command.position is None:
            inp.control_interface = ControlInterface.Velocity
            inp.target_position = position
        else:
            if len(command.position) != 2 or not all(
                math.isfinite(x) and abs(x) <= RANGE_RAD[a]
                for a, x in zip(AXES, command.position)
            ):
                raise MotionError("destination outside configured angle range")
            inp.control_interface = ControlInterface.Position
            inp.target_position = command.position
        try:
            # Ruckig's exact current-state boundary check can reject its own
            # sampled curve by one floating-point ULP at peak velocity. Check
            # feasibility with a 1e-9 numerical tolerance; never reset p/v/a.
            for i, axis in enumerate(AXES):
                v, a = velocity[i], acceleration[i]
                peak = abs(v)
                if v * a > 0:
                    peak += a * a / (2 * MOTION_MAX_JERK[axis])
                if (
                    peak > MOTION_MAX_VELOCITY[axis] + 1e-9
                    or abs(a) > MOTION_MAX_ACCELERATION[axis] + 1e-9
                ):
                    raise MotionError(f"{axis}: infeasible current motion state")
            self._generator.validate_input(inp, False, True)
            result = self._generator.calculate(inp, self._trajectory)
        except Exception as error:
            raise MotionError(f"trajectory calculation failed: {error}") from error
        if result < 0:
            raise MotionError(f"trajectory calculation failed: {result}")
        # Endpoint clamping alone does not protect against a braking overshoot.
        # Inspect the entire curve before sending any of its samples to hardware.
        for axis, bound in zip(AXES, self._trajectory.position_extrema):
            limit = RANGE_RAD[axis]
            if bound.min < -limit - 1e-9 or bound.max > limit + 1e-9:
                raise MotionError(f"{axis}: trajectory would cross angle limit")
        self._command = command
        self._started_at = now
        self.finished = self._trajectory.duration == 0
        self.replans += 1
        self.last_replan_seconds = time.perf_counter() - started
        self.max_replan_seconds = max(self.max_replan_seconds, self.last_replan_seconds)

    def metrics(self):
        return {
            "compute_ms": self.last_compute_seconds * 1000,
            "max_compute_ms": self.max_compute_seconds * 1000,
            "replan_ms": self.last_replan_seconds * 1000,
            "max_replan_ms": self.max_replan_seconds * 1000,
            "replans": self.replans,
        }
