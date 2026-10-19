"""Single owner of target → aim → freeze → warning → observe → threat transitions."""

from __future__ import annotations

import logging
import platform
import threading
from enum import Enum

from animal import settings
from animal.audio import AudioCatalog, DryRunPlayer, WavPlayer
from animal.aiming import TrackerCameraController as TrackerCameraController
from animal.perception import (
    Detection as Detection,
    Event,
    EscapeTracker,
    EscapeStep as EscapeStep,
    read_detections,
    validate_model_classes as validate_model_classes,
    torso_matches as torso_matches,
    AreaMonitor as AreaMonitor,
    _plain as _plain,
)

logger = logging.getLogger(__name__)


class Phase(str, Enum):
    STOPPED = "stopped"
    SEARCH = "search"
    AIM_WARNING = "aim_warning"
    BASELINE_WARNING = "frozen_warning_baseline"
    WAIT_WARNING_ASSET = "missing_warning_asset"
    PLAY_WARNING = "playing_warning"
    OBSERVE_WARNING = "observing_warning"
    AIM_THREAT = "aim_threat"
    BASELINE_THREAT = "frozen_threat_baseline"
    PLAY_THREAT = "playing_threat"
    OBSERVE_THREAT = "observing_threat"
    FAULT = "fault"


FROZEN_PHASES = {
    Phase.BASELINE_WARNING,
    Phase.WAIT_WARNING_ASSET,
    Phase.PLAY_WARNING,
    Phase.OBSERVE_WARNING,
    Phase.BASELINE_THREAT,
    Phase.PLAY_THREAT,
    Phase.OBSERVE_THREAT,
}


class WildlifeDeterrent:
    def __init__(
        self,
        *,
        class_map,
        expected_names,
        catalog,
        warning_catalogs,
        player,
        escape_tracker,
        camera=None,
        torso_class_id=settings.TORSO_CLASS_ID,
        torso_confidence=settings.TORSO_TRACK_CONFIDENCE,
    ):
        self.class_map, self.expected_names = dict(class_map), dict(expected_names)
        self.catalog, self.warning_catalogs = catalog, warning_catalogs
        self.player, self.camera = player, camera
        self.escape_tracker = escape_tracker
        self.torso_class_id, self.torso_confidence = torso_class_id, torso_confidence
        self.min_selection_confidence = settings.MIN_SELECTION_CONFIDENCE
        self.tracker_config = settings.TRACKER_CONFIG
        self.phase = Phase.SEARCH
        self.last_escape_step = None
        self.last_error = None
        self.last_event = Event(None, None, None, "search")
        self._lock = threading.RLock()
        self._last_result_at = None
        self._observation_started = None
        self._frozen_ready_since = None
        self._generation = 0
        self.identity_tracker = (
            None  # Lazy construction; no Ultralytics needed by state tests.
        )

    @classmethod
    def from_settings(
        cls,
        *,
        player=None,
        camera=None,
        class_map=None,
        expected_names=None,
        torso_class_id=None,
        torso_confidence=None,
        dry_run=None,
    ):
        class_map = dict(settings.CLASS_TO_AUDIO if class_map is None else class_map)
        if player is None:
            command = settings.AUDIO_PLAYER_COMMAND or (
                "afplay" if platform.system() == "Darwin" else "aplay"
            )
            dry_run = settings.DRY_RUN if dry_run is None else dry_run
            player = DryRunPlayer() if dry_run else WavPlayer(command)
        warning_catalogs = {}
        for label in set(class_map.values()):
            folder = settings.WARNING_AUDIO_DIR / label
            if list(folder.glob("*.wav")):
                warning_catalogs[label] = AudioCatalog.from_directory(
                    settings.WARNING_AUDIO_DIR, {label}, 1
                )
            else:
                logger.warning(
                    "Warning asset missing: %s/*.wav; threat substitution is disabled",
                    folder,
                )
        return cls(
            class_map=class_map,
            expected_names=(
                settings.EXPECTED_MODEL_NAMES
                if expected_names is None
                else expected_names
            ),
            catalog=AudioCatalog.from_directory(
                settings.AUDIO_DIR,
                set(class_map.values()),
                settings.MIN_AUDIO_FILES_PER_CLASS,
            ),
            warning_catalogs=warning_catalogs,
            player=player,
            camera=camera,
            escape_tracker=EscapeTracker(
                track_lost_seconds=settings.TRACK_LOST_SECONDS,
                reselect_grace_seconds=settings.RESELECT_GRACE_SECONDS,
                area_baseline_seconds=settings.AREA_BASELINE_SECONDS,
                area_smooth_seconds=settings.AREA_SMOOTH_SECONDS,
                area_hold_seconds=settings.AREA_HOLD_SECONDS,
                area_ratio_threshold=settings.AREA_RATIO_THRESHOLD,
                height_ratio_threshold=settings.HEIGHT_RATIO_THRESHOLD,
                distance_metric=settings.DISTANCE_METRIC,
                nominal_fps=settings.TORSO_NOMINAL_FPS,
                edge_margin_px=settings.EDGE_MARGIN_PX,
                torso_window_size=settings.TORSO_WINDOW_SIZE,
                torso_speed_threshold=settings.TORSO_SPEED_THRESHOLD,
            ),
            torso_class_id=(
                settings.TORSO_CLASS_ID if torso_class_id is None else torso_class_id
            ),
            torso_confidence=(
                settings.TORSO_TRACK_CONFIDENCE
                if torso_confidence is None
                else torso_confidence
            ),
        )

    @property
    def generation(self):
        with self._lock:
            return self._generation

    def _event(self, reason, target=None, file=None):
        label = self.class_map.get(target.class_id) if target else None
        self.last_event = Event(target, label, file, reason)
        return self.last_event

    def _clear_tracking_state(self):
        self._observation_started = None
        self.last_escape_step = None
        if self.camera:
            self.camera.reset()

    def _hold(self):
        self._frozen_ready_since = None
        if self.camera:
            self.camera.hold()

    def _fault(self, reason):
        self.phase = Phase.FAULT
        self._generation += 1
        self.last_error = reason
        self.player.stop()
        try:
            self._hold()
        except Exception:
            logger.exception("Cannot hold motor after fault")
        logger.error("%s", reason)
        return self._event(reason)

    def fail(self, reason):
        with self._lock:
            return self._fault(reason)

    def stop(self):
        """Latched software stop + controlled return to default pose; resume is explicit."""
        with self._lock:
            self.phase = Phase.STOPPED
            self._generation += 1
            self.escape_tracker.reset()
            self._clear_tracking_state()
            try:
                self._hold()
                self.player.stop()
                if self.camera:
                    self.camera.home()
            except Exception as error:
                return self._fault(f"stop_failed: {error}")
            return self._event("stopped_returning_to_default")

    def resume(self):
        with self._lock:
            if self.camera and not self.camera.health()["ok"]:
                return self._fault(f"motor_fault: {self.camera.health()['fault']}")
            self.player.stop()
            self._hold()  # Cancel a still-running return ramp before accepting new targets.
            self.escape_tracker.reset()
            self._clear_tracking_state()
            self._generation += 1
            self.last_error = None
            self.phase = Phase.SEARCH
            return self._event("search")

    def pause(self):
        """Initial menu gate: hold current position, without commanding home."""
        with self._lock:
            self.phase = Phase.STOPPED
            self._generation += 1
            self.player.stop()
            self._hold()

    def _play(self, target, warning):
        label = self.class_map[target.class_id]
        catalog = self.warning_catalogs.get(label) if warning else self.catalog
        if catalog is None:
            self.phase = Phase.WAIT_WARNING_ASSET
            return self._event("missing_warning_asset", target)
        try:
            file = catalog.next(label)
            self.player.play(file)
        except Exception as error:
            return self._fault(f"playback_failed: {error}")
        self.phase = Phase.PLAY_WARNING if warning else Phase.PLAY_THREAT
        return self._event(
            "warning_requested" if warning else "threat_requested", target, file
        )

    def _poll_audio(self, now):
        if self.phase not in (Phase.PLAY_WARNING, Phase.PLAY_THREAT):
            return
        status = self.player.status()
        if status.state == "failed":
            self._fault(f"playback_failed: exit={status.returncode}; {status.error}")
        elif status.state == "completed":
            self.phase = (
                Phase.OBSERVE_WARNING
                if self.phase == Phase.PLAY_WARNING
                else Phase.OBSERVE_THREAT
            )
            self._observation_started = now
            self._event("playback_completed", self.last_event.target, status.file)
        elif status.state != "playing":
            self._fault(f"playback_interrupted: {status.state}")

    def tick(self, now, latest_camera_at=None):
        """Independent watchdog; call even during blocked inference or absent frames."""
        with self._lock:
            if self.phase == Phase.FAULT:
                return self.last_event
            if self.camera:
                health = self.camera.health()
                if not health["ok"]:
                    return self._fault(f"motor_fault: {health['fault']}")
            self._poll_audio(now)
            if self.phase in (Phase.STOPPED, Phase.FAULT, Phase.SEARCH):
                return self.last_event
            camera_stale = (
                latest_camera_at is not None
                and now - latest_camera_at > settings.CAMERA_TIMEOUT_SECONDS
            )
            perception_stale = (
                self._last_result_at is not None
                and now - self._last_result_at > settings.PERCEPTION_TIMEOUT_SECONDS
            )
            if camera_stale or perception_stale:
                self.player.stop()
                self._hold()
                self.escape_tracker.reset()
                self._clear_tracking_state()
                self.phase = Phase.SEARCH
                self._generation += 1
                return self._event(
                    "camera_timeout" if camera_stale else "perception_timeout"
                )
            return self.last_event

    def process_result(self, raw, captured_at, **kwargs):
        frame_size, detections = read_detections(raw)
        return self.process_detections(frame_size, detections, captured_at, **kwargs)

    def process_detections(
        self, frame_size, detections, captured_at, *, now=None, generation=None
    ):
        with self._lock:
            now = captured_at if now is None else now
            if generation is not None and generation != self._generation:
                return self.last_event
            if self.phase in (Phase.STOPPED, Phase.FAULT):
                return self.last_event
            if now - captured_at > settings.MAX_RESULT_AGE_SECONDS:
                return self._event("stale_detection_ignored")
            self._last_result_at = captured_at
            try:
                return self._process(frame_size, detections, captured_at, now)
            except Exception as error:
                logger.exception("scenario failure")
                return self._fault(f"scenario_failed: {error}")

    def _process(self, frame_size, detections, captured_at, now):
        self._poll_audio(now)
        if self.phase == Phase.FAULT:
            return self.last_event
        animals = [
            d
            for d in detections
            if d.class_id in self.class_map
            and d.confidence >= self.min_selection_confidence
        ]
        torsos = [
            d
            for d in detections
            if d.class_id == self.torso_class_id
            and d.confidence >= self.torso_confidence
        ]
        observe = self.phase in FROZEN_PHASES
        if self.phase in (Phase.BASELINE_WARNING, Phase.BASELINE_THREAT):
            ready = self.camera is not None and self.camera.frozen_ready()
            if not ready:
                self._frozen_ready_since = None
            elif self._frozen_ready_since is None:
                self._frozen_ready_since = now
            # A result arriving after braking may have been CAPTURED during motion.
            observe = ready and captured_at >= self._frozen_ready_since
        step = self.escape_tracker.step(
            captured_at,
            animals,
            torsos,
            *frame_size,
            observe=observe,
        )
        self.last_escape_step = step
        # Termination is authoritative; never fall through to aim or audio on this frame.
        if step.terminated:
            self.player.stop()
            self._hold()
            self._clear_tracking_state()
            self.phase = Phase.SEARCH
            return self._event(step.termination_reason, step.selected)
        target = step.selected
        if target is None and step.matched_torso is None:
            if self.camera:
                self.camera.missing()
            return self._event(
                "target_temporarily_missing"
                if self.escape_tracker.target_id is not None
                else "search"
            )
        if self.phase == Phase.SEARCH:
            # EscapeTracker already picked this target the moment it became
            # eligible (see its own one-frame selection); no extra multi-frame
            # debounce here — W is the gate, this is the first usable frame.
            self.phase = Phase.AIM_WARNING
        if self.phase in (Phase.AIM_WARNING, Phase.AIM_THREAT):
            if self.camera is None:
                return self._event("motor_disabled", target)
            if step.matched_torso is None:
                self.camera.missing()
                return self._event("torso_missing_hold", target)
            if self.camera.aim(step.matched_torso, frame_size, captured_at):
                self._hold()
                self.escape_tracker.reset_observation()
                self.phase = (
                    Phase.BASELINE_WARNING
                    if self.phase == Phase.AIM_WARNING
                    else Phase.BASELINE_THREAT
                )
            return self._event(self.phase.value, target)
        if self.phase in (Phase.BASELINE_WARNING, Phase.BASELINE_THREAT):
            if (
                target is not None
                and self.escape_tracker.baseline_ready
                and target.confidence >= settings.MIN_PLAYBACK_CONFIDENCE
            ):
                return self._play(target, warning=self.phase == Phase.BASELINE_WARNING)
        elif self.phase == Phase.OBSERVE_WARNING:
            if now - self._observation_started >= settings.WARNING_OBSERVATION_SECONDS:
                self.phase = Phase.AIM_THREAT
                self.escape_tracker.reset_observation()
                if self.camera:
                    self.camera.reset()
        # After one threat clip, stay frozen and observe until exit, loss or stop.
        return self._event(self.phase.value, target)


def process_frame(model, service, frame, captured_at, *, now=None, generation=None):
    from animal.inference import infer_detections

    size, detections = infer_detections(model, service, frame)
    import time

    return service.process_detections(
        size,
        detections,
        captured_at,
        now=time.monotonic() if now is None else now,
        generation=generation,
    )
