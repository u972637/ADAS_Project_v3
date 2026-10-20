"""Production wildlife runner and reusable live-session infrastructure.

Experiment launchers may call this module; this module never imports them.
"""

from __future__ import annotations

from contextlib import ExitStack
import csv
from datetime import datetime
import logging
import os
import select
import sys
import threading
import time

from animal import settings
from animal.wildlife import Phase, WildlifeDeterrent
from animal.perception import validate_model_classes
from animal.overlay import put_lines_kr

logger = logging.getLogger(__name__)


def _log_torso_size(matched_torso):
    """Append the red-dot torso's pixel width/height/area to settings.TORSO_SIZE_LOG_PATH."""
    path = settings.TORSO_SIZE_LOG_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not path.exists()
    x1, y1, x2, y2 = matched_torso.xyxy
    width, height = x2 - x1, y2 - y1
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    with path.open("a", newline="") as handle:
        writer = csv.writer(handle)
        if is_new:
            writer.writerow(["timestamp", "width_px", "height_px", "area_px"])
        writer.writerow([stamp, width, height, width * height])


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
        if command in ("stop", "r"):
            self.service.stop()
        elif command in ("resume", "w"):
            self.service.resume()
        elif command in ("quit", "q"):
            self.service.stop(yaw_deg=settings.QUIT_YAW_DEG, pitch_deg=settings.QUIT_PITCH_DEG)
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
    from animal.camera import (
        CameraCapture,
        camera_hint,
        configured_camera,
        open_camera,
    )
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
                from motor.factory import create_tracker

                tracker = create_tracker()
            stack.callback(tracker.close)
            service.camera = TrackerCameraController(tracker, (FRAME_W, FRAME_H))
        stack.callback(service.player.stop)
        if start_paused:
            service.pause()
            if service.camera:
                # Menu 1/2 (and `--start-paused`) land at the shared default
                # pose right away instead of wherever the rig happened to be
                # left -- pause() only holds the current position.
                service.camera.home()
        camera = configured_camera()
        cap = open_camera(camera)
        if cap is None:
            raise RuntimeError(f"Cannot open camera {camera}. {camera_hint()}")
        capture = CameraCapture(cap, camera)
        stack.callback(capture.release)
        if not headless:
            cv2.namedWindow("Wildlife Deterrent", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Wildlife Deterrent", 1100, 825)
            stack.callback(cv2.destroyAllWindows)
        guard = RuntimeGuard(service, capture)
        stack.callback(guard.close)
        print(
            "Terminal: stop / resume / quit + Enter. Window: R stop, W resume, Q exit."
        )
        print(
            f"dry_run={dry_run}, motor={tracker is not None}, start_paused={start_paused}, "
            f"warning_assets={settings.WARNING_AUDIO_DIR}"
        )
        last_sequence = 0
        last_report = None
        last_loop_at = time.monotonic()
        prev_ready = None
        try:
            while not guard.quit.is_set():
                now_loop = time.monotonic()
                gap = now_loop - last_loop_at
                if gap > 0.2:
                    # Pins down whether a freeze is an upstream stall (no new
                    # frame reaching read_new -- camera/driver/USB side) or
                    # something blocking this loop itself despite frames
                    # arriving on time; the per-stage timing below only
                    # covers frames that DID arrive, so a stall before that
                    # point would otherwise go unlogged. prev_ready tells us
                    # whether the PREVIOUS iteration (the one that took this
                    # long) had a frame to process or was just idling.
                    logger.warning(
                        "loop gap %.0f ms (prev_ready=%s)", gap * 1000, prev_ready
                    )
                last_loop_at = now_loop
                ready, frame, last_sequence, captured_at = capture.read_new(
                    last_sequence
                )
                prev_ready = ready
                if ready:
                    generation = service.generation
                    started = time.monotonic()
                    size, detections = inference(model, service, frame)
                    inferred_at = time.monotonic()
                    event = service.process_detections(
                        size,
                        detections,
                        captured_at,
                        now=time.monotonic(),
                        generation=generation,
                    )
                    processed_at = time.monotonic()
                    if not headless:
                        # Capture, inference and control run on the full-res
                        # frame; the preview window only needs to be legible,
                        # so draw on a downscaled copy (matches the window's
                        # own size) instead of a 2592x1944 canvas -- drawing,
                        # the Korean-text PIL round-trip and cv2.imshow's GUI
                        # blit all scale with pixel count and were a real
                        # source of display-side stutter at full resolution.
                        display_w = 1100
                        display_scale = display_w / size[0]
                        display_h = round(size[1] * display_scale)
                        display = cv2.resize(frame, (display_w, display_h))

                        def scaled_xyxy(xyxy):
                            x1, y1, x2, y2 = xyxy
                            return (
                                round(x1 * display_scale),
                                round(y1 * display_scale),
                                round(x2 * display_scale),
                                round(y2 * display_scale),
                            )

                        def scaled_point(point):
                            return (
                                round(point[0] * display_scale),
                                round(point[1] * display_scale),
                            )

                        for detection in detections:
                            is_torso = detection.class_id == service.torso_class_id
                            # Animals under MIN_SELECTION_CONFIDENCE are
                            # still detected and tracked (they get an ID) but
                            # can never be locked onto -- draw them gray so
                            # the threshold is visible instead of looking
                            # ignored.
                            eligible = (
                                is_torso
                                or detection.confidence >= service.min_selection_confidence
                            )
                            color = (
                                (0, 180, 255)
                                if is_torso
                                else (80, 200, 80)
                                if eligible
                                else (140, 140, 140)
                            )
                            x1, y1, x2, y2 = scaled_xyxy(detection.xyxy)
                            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                            if detection.track_id is not None:
                                cv2.putText(
                                    display,
                                    f"{service.class_map[detection.class_id]} ID={detection.track_id} {detection.confidence:.2f}",
                                    (x1, max(20, y1 - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    color,
                                    2,
                                )
                            if detection.class_id == service.torso_class_id:
                                # Every detected torso's target coordinate shows
                                # in yellow, always -- not only the locked one.
                                center = scaled_point(detection.center)
                                cv2.circle(display, center, 8, (0, 255, 255), -1)
                        selected = service.last_escape_step
                        if selected and not selected.terminated:
                            if selected.selected:
                                x1, y1, x2, y2 = scaled_xyxy(selected.selected.xyxy)
                                cv2.rectangle(
                                    display, (x1, y1), (x2, y2), (0, 255, 0), 5
                                )
                            if selected.matched_torso:
                                # Red = motor has an active aim signal on this
                                # torso (tracking), independent of whether
                                # sound happens to be playing right now.
                                center = scaled_point(selected.matched_torso.center)
                                cv2.circle(display, center, 8, (0, 0, 255), -1)
                            elif service.camera is not None and not any(
                                d.class_id == service.torso_class_id
                                for d in detections
                            ):
                                # Orange = NO torso in view at all this frame
                                # (no yellow dot either), but the lock is
                                # still alive (not yet released) and the
                                # motor keeps resending its last aimed point
                                # (animal/aiming.py missing()) -- a held
                                # motor target with no torso. A visible
                                # torso that simply isn't matched to the lock
                                # shows as yellow only, never orange.
                                held = service.camera.held_aim_point_px
                                if held is not None:
                                    center = scaled_point(held)
                                    cv2.circle(display, center, 8, (0, 165, 255), -1)
                        cv2.drawMarker(
                            display,
                            (display_w // 2, display_h // 2),
                            (255, 255, 255),
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
                        lines = [
                            f"{service.phase.value} | {event.reason}",
                            # 추론(모델 판정)과 판정(추적/소리 결정)을 나눠서 표시 --
                            # 끊김의 원인이 모델 추론 쪽인지 판정 로직 쪽인지 바로 구분용.
                            f"추론 {(inferred_at - started) * 1000:.0f}ms | 판정 "
                            f"{(processed_at - inferred_at) * 1000:.0f}ms | "
                            f"그리기 {(time.monotonic() - processed_at) * 1000:.0f}ms",
                        ]
                        if tracker:
                            health = tracker.health()
                            lines.append(
                                f"Control {tracker.get_actual_freq():.0f} Hz | feedback {health['feedback_age']}"
                            )
                        if service.last_error:
                            lines.append(service.last_error)
                        # 키 안내: 한글이 섞인 줄이라 cv2.putText로 그리면
                        # 깨지므로(???로 표시) put_lines_kr 쪽으로 모은다.
                        lines.append(
                            "R=정지  W=재개  Q=종료  "
                            "U=추적 중인 torso 크기 기록"
                        )
                        overlay_lines = [(status_text, (20, 10), 28, status_color)]
                        overlay_lines += [
                            (line, (20, 45 + 35 * i), 22, (0, 220, 255))
                            for i, line in enumerate(lines)
                        ]
                        display = put_lines_kr(display, overlay_lines)
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
                    elif key == ord("r"):
                        guard.command("stop")
                    elif key == ord("w"):
                        guard.command("resume")
                    elif key == ord("u"):
                        selected = service.last_escape_step
                        if (
                            selected
                            and not selected.terminated
                            and selected.matched_torso
                        ):
                            _log_torso_size(selected.matched_torso)
        except KeyboardInterrupt:
            pass
        finally:
            guard.close()  # No resume command may cancel the final return ramp.
            # The program is exiting on every path that reaches here (normal
            # 'q', Ctrl+C, or an uncaught loop error) — always land on the
            # quit pose, not whatever a mid-session stop() last used.
            service.stop(yaw_deg=settings.QUIT_YAW_DEG, pitch_deg=settings.QUIT_PITCH_DEG)
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
