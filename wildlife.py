"""One-frame wildlife detection, camera aiming, and audio decisions."""

from __future__ import annotations

import logging
import math
import platform
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from audio import AudioCatalog, DryRunPlayer, WavPlayer
import settings

logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class Detection:
    class_id: int
    confidence: float
    xyxy: tuple[float, float, float, float]

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.xyxy
        return (x1 + x2) / 2, (y1 + y2) / 2


@dataclass(frozen=True, slots=True)
class Event:
    target: Detection | None
    label: str | None
    audio_file: Path | None
    reason: str


def _plain(value: object):
    """Convert CPU or CUDA YOLO tensors to ordinary Python values."""
    for method in ("detach", "cpu", "tolist"):
        if hasattr(value, method):
            value = getattr(value, method)()
    return value


def read_detections(result: object) -> tuple[tuple[int, int], tuple[Detection, ...]]:
    """Read one Ultralytics Result using its named box properties."""
    shape = getattr(result, "orig_shape", None)
    if shape is None or len(shape) < 2:
        raise ValueError("YOLO Result.orig_shape is required")
    height, width = int(shape[0]), int(shape[1])
    if min(width, height) <= 0:
        raise ValueError("frame dimensions must be positive")
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return (width, height), ()
    xyxy, scores, class_ids = (
        _plain(boxes.xyxy),
        _plain(boxes.conf),
        _plain(boxes.cls),
    )
    if not (len(xyxy) == len(scores) == len(class_ids)):
        raise ValueError("YOLO box property lengths differ")
    detections = []
    for box, raw_score, raw_id in zip(xyxy, scores, class_ids, strict=True):
        if len(box) != 4:
            raise ValueError("xyxy must contain four coordinates")
        coords = tuple(float(value) for value in box)
        score, class_id = float(raw_score), float(raw_id)
        if not all(math.isfinite(value) for value in (*coords, score, class_id)):
            raise ValueError("detection values must be finite")
        if not 0 <= score <= 1 or not class_id.is_integer() or class_id < 0:
            raise ValueError("invalid YOLO confidence or class ID")
        x1, y1, x2, y2 = coords
        if x2 <= x1 or y2 <= y1:
            raise ValueError("xyxy must have positive width and height")
        clipped = (
            max(0.0, min(width, x1)),
            max(0.0, min(height, y1)),
            max(0.0, min(width, x2)),
            max(0.0, min(height, y2)),
        )
        if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
            raise ValueError("xyxy lies outside the frame")
        detections.append(Detection(int(class_id), score, clipped))
    return (width, height), tuple(detections)


def validate_model_classes(
    model_names: Mapping[int, str] | Sequence[str],
    class_map: Mapping[int, str],
    expected_names: Mapping[int, str],
) -> None:
    """Reject class IDs copied from a different model before using hardware."""
    if set(expected_names) != set(class_map):
        raise ValueError("model_names must cover exactly the configured class IDs")
    for class_id, audio_label in class_map.items():
        try:
            actual_name = model_names[class_id]
        except (KeyError, IndexError) as error:
            raise ValueError(f"model has no class ID {class_id}") from error
        if actual_name != expected_names[class_id]:
            raise ValueError(
                f"class ID {class_id}: expected {expected_names[class_id]!r} "
                f"for audio {audio_label!r}, model reports {actual_name!r}"
            )


class TrackerCameraController:
    def __init__(self, tracker: object, tracker_size: tuple[int, int]) -> None:
        if min(tracker_size) <= 0:
            raise ValueError("tracker frame dimensions must be positive")
        self.tracker = tracker
        self.tracker_size = tracker_size

    def aim(self, target: Detection, frame_size: tuple[int, int]) -> None:
        if min(frame_size) <= 0:
            raise ValueError("detection frame dimensions must be positive")
        x, y = target.center
        width, height = frame_size
        tracker_width, tracker_height = self.tracker_size
        self.tracker.update(x * tracker_width / width, y * tracker_height / height)


class TriggerPolicy:
    def __init__(self, min_confidence: float, consecutive_frames: int,
                 release_frames: int, cooldown_seconds: float) -> None:
        if not 0 <= min_confidence <= 1:
            raise ValueError("min_confidence must be within [0, 1]")
        if consecutive_frames < 1 or release_frames < 1 or cooldown_seconds < 0:
            raise ValueError("invalid frame counts or cooldown")
        self.min_confidence = min_confidence
        self.consecutive_frames = consecutive_frames
        self.release_frames = release_frames
        self.cooldown_seconds = cooldown_seconds
        self._label: str | None = None
        self._seen = 0
        self._missing = 0
        self._played_for_sighting = False
        self._last_played_at = float("-inf")

    def should_play(self, label: str | None, confidence: float, captured_at: float) -> bool:
        if label is None or confidence < self.min_confidence:
            self._missing += 1
            self._seen = 0
            if self._missing >= self.release_frames:
                self._label = None
                self._played_for_sighting = False
            return False
        self._missing = 0
        if label != self._label:
            self._label = label
            self._seen = 1
            self._played_for_sighting = False
        else:
            self._seen += 1
        return (
            self._seen >= self.consecutive_frames
            and not self._played_for_sighting
            and captured_at - self._last_played_at >= self.cooldown_seconds
        )

    def mark_attempted(self, captured_at: float) -> None:
        self._played_for_sighting = True
        self._last_played_at = captured_at


class WildlifeDeterrent:
    def __init__(self, class_map: dict[int, str], expected_names: dict[int, str],
                 min_selection_confidence: float, policy: TriggerPolicy,
                 catalog: AudioCatalog, player: object, camera: object | None = None) -> None:
        if not 0 <= min_selection_confidence <= 1:
            raise ValueError("min_selection_confidence must be within [0, 1]")
        self.class_map = class_map
        self.expected_names = expected_names
        self.min_selection_confidence = min_selection_confidence
        self.policy = policy
        self.catalog = catalog
        self.player = player
        self.camera = camera

    @classmethod
    def from_settings(cls, *, player: object | None = None,
                      camera: object | None = None) -> WildlifeDeterrent:
        if player is None:
            command = settings.AUDIO_PLAYER_COMMAND or (
                "afplay" if platform.system() == "Darwin" else "aplay"
            )
            player = DryRunPlayer() if settings.DRY_RUN else WavPlayer(command)
        return cls(
            class_map=dict(settings.CLASS_TO_AUDIO),
            expected_names=dict(settings.EXPECTED_MODEL_NAMES),
            min_selection_confidence=settings.MIN_SELECTION_CONFIDENCE,
            policy=TriggerPolicy(
                min_confidence=settings.MIN_PLAYBACK_CONFIDENCE,
                consecutive_frames=settings.CONSECUTIVE_FRAMES,
                release_frames=settings.RELEASE_FRAMES,
                cooldown_seconds=settings.COOLDOWN_SECONDS,
            ),
            catalog=AudioCatalog.from_manifest(
                settings.AUDIO_MANIFEST,
                set(settings.CLASS_TO_AUDIO.values()),
                settings.MIN_AUDIO_FILES_PER_CLASS,
            ),
            player=player,
            camera=camera,
        )

    def process_result(self, raw: object, captured_at: float) -> Event:
        frame_size, detections = read_detections(raw)
        target = max(
            (item for item in detections
             if item.class_id in self.class_map
             and item.confidence >= self.min_selection_confidence),
            key=lambda item: item.confidence,
            default=None,
        )
        if target is None:
            self.policy.should_play(None, 0.0, captured_at)
            return Event(None, None, None, "no_target")

        if self.camera is not None:
            try:
                self.camera.aim(target, frame_size)
            except Exception:
                logger.exception("camera aim failed")

        label = self.class_map[target.class_id]
        if not self.policy.should_play(label, target.confidence, captured_at):
            return Event(target, label, None, "not_triggered")
        if self.player.is_playing():
            return Event(target, label, None, "audio_busy")

        audio_file = self.catalog.next(label)
        try:
            self.player.play(audio_file)
        except Exception:
            logger.exception("audio playback failed: %s", audio_file)
            self.policy.mark_attempted(captured_at)
            return Event(target, label, audio_file, "playback_failed")
        self.policy.mark_attempted(captured_at)
        return Event(target, label, audio_file, "played")


def process_frame(model: object, service: WildlifeDeterrent,
                  frame: object, captured_at: float) -> Event:
    results = model(frame, verbose=False)
    if len(results) != 1:
        raise ValueError("wildlife model must return one result per frame")
    return service.process_result(results[0], captured_at)
