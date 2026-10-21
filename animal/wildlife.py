"""Single owner of target -> aim -> continuous playback transitions.

Sound starts once the aimed torso point (the red dot -- a pure motor-signal
condition, independent of the animal bbox) is within PLAYBACK_ENTER_FRACTION
of screen center, then keeps playing (motor keeps tracking the whole time)
until the aim point drifts past PLAYBACK_EXIT_FRACTION.
"""

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
    TRACKING = "tracking"  # aiming continuously; audio on/off via hysteresis
    FAULT = "fault"


class WildlifeDeterrent:
    def __init__(
        self,
        *,
        class_map,
        expected_names,
        catalogs,
        player,
        escape_tracker,
        camera=None,
        torso_class_id=settings.TORSO_CLASS_ID,
        torso_confidence=settings.TORSO_TRACK_CONFIDENCE,
    ):
        self.class_map, self.expected_names = dict(class_map), dict(expected_names)
        self.catalogs = catalogs  # label -> AudioCatalog; missing label = no sound for it
        self.player, self.camera = player, camera
        self.escape_tracker = escape_tracker
        self.torso_class_id, self.torso_confidence = torso_class_id, torso_confidence
        self.min_selection_confidence = settings.MIN_SELECTION_CONFIDENCE
        self.green_forget_seconds = settings.GREEN_ID_FORGET_SECONDS
        # track_id -> last time it was seen while green. A bbox is "green" when
        # its confidence reaches min_selection_confidence, and its id then stays
        # green at any confidence until unseen for green_forget_seconds.
        self._green_ids: dict[int, float] = {}
        self.tracker_config = settings.TRACKER_CONFIG
        self.phase = Phase.SEARCH
        self.last_escape_step = None
        self.last_error = None
        self.last_event = Event(None, None, None, "search")
        self._lock = threading.RLock()
        self._last_result_at = None
        self._playing = False
        self._playback_label = None
        self._next_clip_at = None
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
        catalogs = {}
        for label in set(class_map.values()):
            folder = settings.WARNING_AUDIO_DIR / label
            if list(folder.glob("*.wav")):
                catalogs[label] = AudioCatalog.from_directory(
                    settings.WARNING_AUDIO_DIR, {label}, 1
                )
            else:
                logger.warning(
                    "Warning asset missing: %s/*.wav; playback disabled for this species",
                    folder,
                )
        return cls(
            class_map=class_map,
            expected_names=(
                settings.EXPECTED_MODEL_NAMES
                if expected_names is None
                else expected_names
            ),
            catalogs=catalogs,
            player=player,
            camera=camera,
            escape_tracker=EscapeTracker(
                track_lost_seconds=settings.ORANGE_TIMEOUT_SECONDS,
                reselect_grace_seconds=settings.RESELECT_GRACE_SECONDS,
                area_baseline_seconds=settings.AREA_BASELINE_SECONDS,
                area_smooth_seconds=settings.AREA_SMOOTH_SECONDS,
                area_hold_seconds=settings.AREA_HOLD_SECONDS,
                area_ratio_threshold=settings.AREA_RATIO_THRESHOLD,
                height_ratio_threshold=settings.HEIGHT_RATIO_THRESHOLD,
                distance_metric=settings.DISTANCE_METRIC,
                edge_margin_px=settings.EDGE_MARGIN_PX,
                min_bbox_height_px=settings.MIN_BBOX_HEIGHT_PX,
                torso_bridge_radius_multiplier=settings.TORSO_BRIDGE_RADIUS_MULTIPLIER,
                torso_bridge_min_radius_px=settings.TORSO_BRIDGE_MIN_RADIUS_PX,
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

    @property
    def playing(self):
        with self._lock:
            return self._playing

    def is_green(self, detection):
        """Animal bbox eligible for lock-on / following: confident enough now,
        or an id that was confident earlier and has not been forgotten."""
        if detection.confidence >= self.min_selection_confidence:
            return True
        with self._lock:
            return detection.track_id is not None and detection.track_id in self._green_ids

    def _update_green_ids(self, detections, at):
        # Forget first: an id that was unseen for green_forget_seconds must not
        # be revived just because it shows up again at low confidence.
        for track_id, seen_at in list(self._green_ids.items()):
            if at - seen_at >= self.green_forget_seconds:
                del self._green_ids[track_id]
        for d in detections:
            if d.class_id in self.class_map and d.track_id is not None and self.is_green(d):
                self._green_ids[d.track_id] = at

    def _event(self, reason, target=None, file=None):
        label = self.class_map.get(target.class_id) if target else None
        self.last_event = Event(target, label, file, reason)
        return self.last_event

    def _clear_tracking_state(self):
        self.last_escape_step = None
        if self.camera:
            self.camera.reset()

    def _hold(self):
        if self.camera:
            self.camera.hold()

    def _stop_playback(self):
        self.player.stop()
        self._playing = False
        self._playback_label = None
        self._next_clip_at = None

    def _fault(self, reason):
        self.phase = Phase.FAULT
        self._generation += 1
        self.last_error = reason
        self._stop_playback()
        try:
            self._hold()
        except Exception:
            logger.exception("Cannot hold motor after fault")
        logger.error("%s", reason)
        return self._event(reason)

    def fail(self, reason):
        with self._lock:
            return self._fault(reason)

    def stop(self, yaw_deg=None, pitch_deg=None):
        """Latched software stop + controlled return to default pose; resume is explicit.

        yaw_deg/pitch_deg override the shared default pose (motor/config.py)
        — used for the program-exit pose, distinct from a mid-session stop.
        """
        with self._lock:
            self.phase = Phase.STOPPED
            self._generation += 1
            self.escape_tracker.reset()
            self._clear_tracking_state()
            try:
                self._hold()
                self._stop_playback()
                if self.camera:
                    self.camera.home(yaw_deg=yaw_deg, pitch_deg=pitch_deg)
            except Exception as error:
                return self._fault(f"stop_failed: {error}")
            return self._event("stopped_returning_to_default")

    def resume(self):
        with self._lock:
            if self.camera and not self.camera.health()["ok"]:
                return self._fault(f"motor_fault: {self.camera.health()['fault']}")
            self._stop_playback()
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
            self._stop_playback()
            self._hold()

    def _start_playback(self, label):
        catalog = self.catalogs.get(label)
        if catalog is None:
            return False
        self._playback_label = label
        self._playing = True
        self._play_next()
        return True

    def _play_next(self):
        """Next random clip for the same label (AudioCatalog: no repeat within one pass)."""
        self._next_clip_at = None
        try:
            file = self.catalogs[self._playback_label].next(self._playback_label)
            self.player.play(file)
        except Exception as error:
            self._fault(f"playback_failed: {error}")
            return
        self._event("sound_requested", self.last_event.target, file)

    def _poll_audio(self, now):
        if not self._playing:
            return
        status = self.player.status()
        if status.state == "failed":
            self._fault(f"playback_failed: exit={status.returncode}; {status.error}")
        elif status.state == "completed":
            if self._next_clip_at is None:
                self._next_clip_at = now + settings.PLAYBACK_GAP_SECONDS
            elif now >= self._next_clip_at:
                self._play_next()
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
                self._stop_playback()
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
            # Green status follows the camera even while paused/stopped, so the
            # on-screen green/gray boxes stay right before W starts anything.
            if now - captured_at <= settings.MAX_RESULT_AGE_SECONDS:
                self._update_green_ids(detections, captured_at)
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
            if d.class_id in self.class_map and self.is_green(d)
        ]
        torsos = [
            d
            for d in detections
            if d.class_id == self.torso_class_id
            and d.confidence >= self.torso_confidence
        ]
        # While orange the tracked torso is expected near the virtual target the
        # motor is chasing, not where the torso last was (the camera has moved).
        orange = getattr(self.camera, "orange_point", None) if self.camera else None
        reference = (
            (orange[0] * frame_size[0], orange[1] * frame_size[1])
            if orange is not None
            else None
        )
        step = self.escape_tracker.step(
            captured_at,
            animals,
            torsos,
            *frame_size,
            observe=True,  # always evaluate; the motor never freezes anymore
            reference=reference,
        )
        self.last_escape_step = step
        # Termination is authoritative; never fall through to aim or audio on this frame.
        if step.terminated:
            self._stop_playback()
            # Slow, gentle return instead of holding in place or snapping back
            # at the normal stop speed — the animal was just seen backing
            # away or lost, so a sudden motion here is the opposite of what
            # we want. A new target reaching TRACKING overrides this ramp
            # the moment camera.aim()/update() is called again.
            if self.camera:
                self.camera.home(duration=settings.TERMINATION_RETURN_SECONDS)
            self._clear_tracking_state()
            self.phase = Phase.SEARCH
            return self._event(step.termination_reason, step.selected)
        target = step.selected
        if target is None and step.matched_torso is None:
            # (no torso and no bbox to follow either)
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
            self.phase = Phase.TRACKING
        if self.camera is None:
            return self._event("motor_disabled", target)
        if step.matched_torso is None and step.followed_bbox is None:
            self.camera.missing()
            return self._event("torso_missing_hold", target)
        # Motor keeps tracking every frame regardless of playback state. With no
        # torso it follows the CENTER of the bbox instead (3-frame filtered).
        if step.matched_torso is not None:
            offset = self.camera.aim(step.matched_torso, frame_size, captured_at)
        else:
            offset = self.camera.aim(
                step.followed_bbox, frame_size, captured_at, source="bbox"
            )
        if self._playing:
            # Only evaluate the exit band while the animal bbox is actually
            # live (target is not None) — during a fallback bridge (bbox
            # momentarily missing), keep playing regardless of offset. It
            # often drifts off-center right as the bbox disappears, which
            # isn't itself a reason to stop; tracking_lost/area/height still
            # end things properly if the bridge runs out.
            if (
                target is not None
                and offset is not None
                and offset > settings.PLAYBACK_EXIT_FRACTION
            ):
                self._stop_playback()
        elif offset is not None and offset <= settings.PLAYBACK_ENTER_FRACTION:
            # Pure red-dot (aim point) condition: the animal bbox itself
            # (`target`) need not be live this frame -- a fallback-bridged
            # torso that happens to be centered still starts playback. The
            # species label comes from the tracker's locked class, which
            # stays set through a fallback bridge even when `target` is None.
            label = self.class_map.get(self.escape_tracker.target_class)
            if label is not None:
                self._start_playback(label)
        return self._event("playing" if self._playing else "tracking", target)


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
