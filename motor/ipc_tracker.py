"""Process-isolated Tracker: the 500 Hz control loop runs in its own process.

CPython's GIL makes "separate thread" an insufficient isolation boundary — a
CPU-heavy main thread (YOLO inference, drawing) can starve the control
thread's turn to run even though it never waits on the main thread's result
(see docs/CHANGES_FROM_TEAM_MAIN.md discussion / MotorFault "control loop
stalled" incidents). A separate OS process has its own GIL, so the parent's
CPU load cannot delay the child's control loop at all.

RemoteTracker (parent side) exposes the subset of Tracker's interface that
animal/aiming.py and animal/runtime.py actually use. Commands are fire-and-
forget (never block on the child); status is broadcast by the child every
STATUS_INTERVAL seconds and cached by a background thread in the parent, so
reads are always "latest known," not round-tripped live.
"""

from __future__ import annotations

import logging
import multiprocessing
import queue as queue_module
import threading
import time

logger = logging.getLogger(__name__)

STATUS_INTERVAL = 0.005  # 5 ms — tighter status freshness; still far looser
                          # than the 500 Hz (2 ms) control tick, so the extra
                          # IPC traffic stays well under the control budget.
STARTUP_TIMEOUT_SECONDS = 5.0
STATUS_STALE_SECONDS = 0.5  # If the child falls silent this long, treat as dead.


class MotorFault(RuntimeError):
    pass


def _motor_worker(cmd_queue, status_conn, ready_event, devicename, baudrate):
    """Runs in the child process. Owns the real Tracker exclusively."""
    from motor.dm4310_tracker import Tracker, MotorFault as _RealMotorFault

    try:
        tracker = Tracker(devicename=devicename, baudrate=baudrate)
    except Exception as error:
        status_conn.send({"startup_error": f"{type(error).__name__}: {error}"})
        ready_event.set()
        return
    ready_event.set()

    def _snapshot():
        health = tracker.health()
        return {
            "startup_error": None,
            "current": {a: tracker.get_state(a) for a in ("yaw", "pitch")},
            "motion_complete": tracker.motion_complete(),
            "actual_freq": tracker.get_actual_freq(),
            "can_tx_freq": tracker.get_can_tx_freq(),
            "rx_freq": {a: tracker.get_rx_freq(a) for a in ("yaw", "pitch")},
            "health": health,
            "sent_at": time.monotonic(),
        }

    try:
        while True:
            try:
                cmd = cmd_queue.get(timeout=STATUS_INTERVAL)
            except queue_module.Empty:
                cmd = None
            # Drain any backlog immediately — only the latest update() matters,
            # matching Tracker._set_command's own "never queue old frames" rule.
            while cmd is not None:
                if cmd[0] == "close":
                    return
                try:
                    _dispatch(tracker, cmd)
                except _RealMotorFault:
                    pass  # Surfaces via the next status snapshot instead.
                try:
                    cmd = cmd_queue.get_nowait()
                except queue_module.Empty:
                    cmd = None
            try:
                status_conn.send(_snapshot())
            except Exception:
                logger.exception("motor worker: failed to send status")
    finally:
        tracker.close()


def _dispatch(tracker, cmd):
    kind = cmd[0]
    if kind == "update":
        _, x, y = cmd
        tracker.update(x, y)
    elif kind == "hold":
        tracker.hold()
    elif kind == "go_to":
        _, yaw_rad, pitch_rad, duration = cmd
        tracker.go_to(yaw_rad=yaw_rad, pitch_rad=pitch_rad, duration=duration)
    elif kind == "go_default":
        tracker.go_default()
    elif kind == "go_center":
        tracker.go_center()
    else:
        raise ValueError(f"unknown motor command: {kind!r}")


class RemoteTracker:
    """Parent-side handle. Matches the subset of Tracker's interface used by
    animal/aiming.py and animal/runtime.py."""

    def __init__(self, devicename=None, baudrate=None):
        from motor.config import DEVICENAME, BAUDRATE

        self._cmd_queue = multiprocessing.Queue()
        parent_conn, child_conn = multiprocessing.Pipe(duplex=False)
        ready_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=_motor_worker,
            args=(
                self._cmd_queue,
                child_conn,
                ready_event,
                devicename or DEVICENAME,
                baudrate or BAUDRATE,
            ),
            daemon=True,
        )
        self._lock = threading.Lock()
        self._status = None
        self._closed = False
        # "cam->target" rate: how often THIS process calls update(), measured
        # here (not by the child) since the real Tracker measures it the same
        # way -- inside update() itself, on the caller's side.
        self._update_count = 0
        self._update_actual_freq = 0.0
        self._update_window_t0 = time.monotonic()
        self._process.start()
        child_conn.close()  # Only the child writes; drop the parent's write end.

        if not ready_event.wait(STARTUP_TIMEOUT_SECONDS):
            self._process.terminate()
            raise MotorFault("motor process startup timed out")

        self._conn = parent_conn
        self._reader = threading.Thread(target=self._read_status, daemon=True)
        self._reader.start()

        # One synchronous wait for the first snapshot so health()/get_state()
        # are valid immediately after construction, matching Tracker's own
        # "wait for feedback before returning" startup contract.
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        while self._status is None and time.monotonic() < deadline:
            time.sleep(0.01)
        if self._status is None:
            self.close()
            raise MotorFault("motor process produced no status before timeout")
        if self._status.get("startup_error"):
            error = self._status["startup_error"]
            self.close()
            raise MotorFault(f"motor process failed to start: {error}")

    def _read_status(self):
        while not self._closed:
            try:
                status = self._conn.recv()
            except (EOFError, OSError):
                return
            with self._lock:
                self._status = status

    def _latest(self):
        with self._lock:
            status = self._status
        if status is None:
            raise MotorFault("no status received from motor process yet")
        if status.get("startup_error"):
            raise MotorFault(f"motor process failed to start: {status['startup_error']}")
        if time.monotonic() - status["sent_at"] > STATUS_STALE_SECONDS:
            raise MotorFault("motor process status is stale — process may be stuck/dead")
        if not self._process.is_alive():
            raise MotorFault("motor process exited")
        return status

    def assert_healthy(self):
        status = self._latest()
        if not status["health"]["ok"]:
            raise MotorFault(status["health"]["fault"])

    def health(self):
        try:
            status = self._latest()
        except MotorFault as error:
            return {"ok": False, "fault": str(error), "feedback_age": {}, "status": {}, "motion": {}}
        return status["health"]

    def update(self, target_x, target_y):
        if target_x is None or target_y is None:
            return
        self.assert_healthy()
        self._cmd_queue.put(("update", float(target_x), float(target_y)))
        self._update_count += 1
        now = time.monotonic()
        if now - self._update_window_t0 >= 1:
            self._update_actual_freq = self._update_count / (now - self._update_window_t0)
            self._update_count, self._update_window_t0 = 0, now

    def hold(self):
        self.assert_healthy()
        self._cmd_queue.put(("hold",))

    def go_to(self, yaw_rad=0.0, pitch_rad=0.0, duration=None):
        self.assert_healthy()
        from motor.config import GO_CENTER_DURATION

        self._cmd_queue.put(
            ("go_to", yaw_rad, pitch_rad, GO_CENTER_DURATION if duration is None else duration)
        )

    def go_center(self):
        self.assert_healthy()
        self._cmd_queue.put(("go_center",))

    def go_default(self):
        self.assert_healthy()
        self._cmd_queue.put(("go_default",))

    def get_state(self, axis):
        return self._latest()["current"][axis]

    def get_yaw_angle_rad(self):
        return self.get_state("yaw")[0]

    def get_actual_freq(self):
        try:
            return self._latest()["actual_freq"]
        except MotorFault:
            return 0.0

    def get_update_freq(self):
        return self._update_actual_freq

    def get_can_tx_freq(self):
        try:
            return self._latest()["can_tx_freq"]
        except MotorFault:
            return 0.0

    def get_rx_freq(self, axis):
        try:
            return self._latest()["rx_freq"][axis]
        except MotorFault:
            return 0.0

    def motion_complete(self):
        return self._latest()["motion_complete"]

    def wait_until_complete(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assert_healthy()
            if self.motion_complete():
                return True
            time.sleep(0.01)
        return False

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._cmd_queue.put(("close",))
        except Exception:
            pass
        if self._process.is_alive():
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._process.terminate()
        self._conn.close()
