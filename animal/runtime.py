"""Production wildlife runner and reusable live-session infrastructure.

Experiment launchers may call this module; this module never imports them.
"""

from __future__ import annotations

from contextlib import ExitStack
import logging
import os
import select
import sys
import threading
import time

from animal import settings
from animal.wildlife import Phase, WildlifeDeterrent
from animal.perception import validate_model_classes
from animal.overlay import put_text_kr

logger = logging.getLogger(__name__)


class RuntimeGuard:
    """Monitor input/feedback and accept terminal stop while YOLO is busy."""

    def __init__(self, service, capture, *, read_stdin=True):
        self.service, self.capture = service, capture
        self.quit = threading.Event()
        self._done = threading.Event()
        self._read_stdin = read_stdin
        self._input_buffer = b""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def command(self, command):
        command = command.strip().lower()
        if command in ("stop", "r", "e"):
            self.service.stop()
        elif command in ("resume", "w"):
            self.service.resume()
        elif command in ("quit", "q"):
            self.service.stop()
            self.quit.set()

    def _run(self):
        while not self._done.wait(0.05):
            try:
                self.service.tick(time.monotonic(), self.capture.latest_capture_at)
                if self._read_stdin and select.select([sys.stdin], [], [], 0)[0]:
                    chunk = os.read(sys.stdin.fileno(), 4096)
                    if not chunk:
                        self._read_stdin = False
                    self._input_buffer += chunk
                    while b"\n" in self._input_buffer:
                        line, self._input_buffer = self._input_buffer.split(b"\n", 1)
                        self.command(line.decode(errors="replace"))
            except Exception as error:
                logger.exception("runtime guard failed")
                self.service.fail(f"watchdog_failed: {error}")
                self.quit.set()
                return

    def close(self):
        self._done.set()
        self._thread.join(timeout=2)


def run(
    *, enable_motor=None, start_paused=None, headless=None, dry_run=None, motor_process=None
):
    """Run the complete production task, independently of start.sh and tools/."""
    from animal.inference import infer_detections

    return run_session(
        model_path=settings.MODEL_PATH,
        service_factory=WildlifeDeterrent.from_settings,
        inference=infer_detections,
        enable_motor=enable_motor,
        start_paused=start_paused,
        headless=headless,
        dry_run=dry_run,
        motor_process=motor_process,
    )


def run_session(
    *,
    model_path,
    service_factory,
    inference,
    enable_motor=None,
    start_paused=None,
    headless=None,
    dry_run=None,
    motor_process=None,
):
    """Shared camera, motor, input and cleanup loop with caller-supplied perception."""
    import cv2
    from ultralytics import YOLO
    from animal.camera import CameraCapture, open_camera
    from animal.aiming import TrackerCameraController, SimulatedCameraController
    from motor.config import FRAME_W, FRAME_H

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    enable_motor = settings.ENABLE_MOTOR if enable_motor is None else enable_motor
    start_paused = settings.START_PAUSED if start_paused is None else start_paused
    headless = settings.HEADLESS if headless is None else headless
    dry_run = settings.DRY_RUN if dry_run is None else dry_run
    motor_process = settings.MOTOR_PROCESS if motor_process is None else motor_process
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    model = YOLO(str(model_path))
    with ExitStack() as stack:
        tracker = None
        adapter = SimulatedCameraController() if dry_run else None
        service = service_factory(camera=adapter, dry_run=dry_run)
        validate_model_classes(
            model.names,
            service.class_map,
            service.expected_names,
            # Negative IDs denote caller-generated boxes, absent from model.names.
            torso_class_id=(
                service.torso_class_id if service.torso_class_id >= 0 else None
            ),
        )
        if enable_motor and not dry_run:
            if motor_process:
                from motor.ipc_tracker import RemoteTracker

                tracker = RemoteTracker()
            else:
                from motor.dm4310_tracker import Tracker

                tracker = Tracker()
            stack.callback(tracker.close)
            service.camera = TrackerCameraController(tracker, (FRAME_W, FRAME_H))
        stack.callback(service.player.stop)
        if start_paused:
            service.pause()
        cap = open_camera(settings.CAMERA_INDEX)
        if cap is None:
            raise RuntimeError(f"Cannot open camera {settings.CAMERA_INDEX}")
        capture = CameraCapture(cap, settings.CAMERA_INDEX)
        stack.callback(capture.release)
        if not headless:
            cv2.namedWindow("Wildlife Deterrent", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Wildlife Deterrent", 1100, 825)
            stack.callback(cv2.destroyAllWindows)
        guard = RuntimeGuard(service, capture)
        stack.callback(guard.close)
        print(
            "Terminal: stop / resume / quit + Enter. Window: R/E/Space stop, W resume, Q exit."
        )
        print(
            f"dry_run={dry_run}, motor={tracker is not None}, start_paused={start_paused}, "
            f"warning_assets={settings.WARNING_AUDIO_DIR}"
        )
        last_sequence = 0
        last_report = None
        try:
            while not guard.quit.is_set():
                ready, frame, last_sequence, captured_at = capture.read_new(
                    last_sequence
                )
                if ready:
                    generation = service.generation
                    started = time.monotonic()
                    size, detections = inference(model, service, frame)
                    event = service.process_detections(
                        size,
                        detections,
                        captured_at,
                        now=time.monotonic(),
                        generation=generation,
                    )
                    if not headless:
                        # Capture, inference and control share unmirrored coordinates.
                        display = frame.copy()
                        for detection in detections:
                            color = (
                                (0, 180, 255)
                                if detection.class_id == service.torso_class_id
                                else (80, 200, 80)
                            )
                            x1, y1, x2, y2 = map(int, detection.xyxy)
                            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                            if detection.track_id is not None:
                                cv2.putText(
                                    display,
                                    f"{service.class_map[detection.class_id]} ID={detection.track_id}",
                                    (x1, max(20, y1 - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    color,
                                    2,
                                )
                            if detection.class_id == service.torso_class_id:
                                # Every detected torso's target coordinate shows
                                # in yellow, always -- not only the locked one.
                                center = tuple(map(int, detection.center))
                                cv2.circle(display, center, 8, (0, 255, 255), -1)
                        selected = service.last_escape_step
                        if selected and not selected.terminated:
                            if selected.selected:
                                x1, y1, x2, y2 = map(int, selected.selected.xyxy)
                                cv2.rectangle(
                                    display, (x1, y1), (x2, y2), (0, 255, 0), 5
                                )
                            if selected.matched_torso and service.phase in (
                                Phase.AIM_WARNING,
                                Phase.AIM_THREAT,
                            ):
                                # Overrides that one point to red while it is
                                # actually being sent to the motor.
                                center = tuple(map(int, selected.matched_torso.center))
                                cv2.circle(display, center, 8, (0, 0, 255), -1)
                        cv2.drawMarker(
                            display, (size[0] // 2, size[1] // 2), (255, 255, 255)
                        )
                        tracking = service.phase not in (
                            Phase.STOPPED,
                            Phase.SEARCH,
                            Phase.FAULT,
                        )
                        status_text, status_color = {
                            Phase.STOPPED: ("정지 (W 로 재개)", (150, 150, 150)),
                            Phase.FAULT: ("고장", (0, 0, 255)),
                        }.get(
                            service.phase,
                            ("추적중" if tracking else "탐색중", (0, 255, 0) if tracking else (0, 255, 255)),
                        )
                        display = put_text_kr(
                            display, status_text, (20, 10), 28, status_color
                        )
                        lines = [
                            f"{service.phase.value} | {event.reason}",
                            f"Inference {(time.monotonic() - started) * 1000:.0f} ms | W resume  R/E/Space stop  Q exit",
                        ]
                        if tracker:
                            health = tracker.health()
                            lines.append(
                                f"Control {tracker.get_actual_freq():.0f} Hz | feedback {health['feedback_age']}"
                            )
                        if service.last_error:
                            lines.append(service.last_error)
                        for i, line in enumerate(lines):
                            cv2.putText(
                                display,
                                line,
                                (20, 45 + 35 * i),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 220, 255),
                                2,
                            )
                        cv2.imshow("Wildlife Deterrent", display)
                else:
                    time.sleep(0.01)
                report = (
                    service.phase.value,
                    service.last_event.reason,
                    service.last_error,
                )
                if report != last_report:
                    logger.info("state=%s event=%s error=%s", *report)
                    last_report = report
                # Poll even when the camera has no frame. Terminal stop also works during inference.
                if not headless:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        guard.command("quit")
                    elif key in (ord("r"), ord("e"), 32):
                        guard.command("stop")
                    elif key == ord("w"):
                        guard.command("resume")
        except KeyboardInterrupt:
            pass
        finally:
            guard.close()  # No resume command may cancel the final return ramp.
            service.stop()
            # Limits may extend the requested return duration; wait for the actual plan.
            if tracker and tracker.health()["ok"]:
                if not tracker.wait_until_complete():
                    logger.error(
                        "Default-pose return timed out; closing motor connection"
                    )
            if tracker:
                logger.info(
                    "Motion timing summary: %s", tracker.health().get("motion", {})
                )
