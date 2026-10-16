"""One-frame wildlife detection, camera aiming, and audio decisions."""

from __future__ import annotations

import logging
import math
import platform
from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from statistics import median

from animal.audio import AudioCatalog, DryRunPlayer, WavPlayer
from animal import settings

logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class Detection:
    class_id: int
    confidence: float
    xyxy: tuple[float, float, float, float]
    track_id: int | None = None

    @property
    def width(self) -> float:
        return max(0.0, self.xyxy[2] - self.xyxy[0])

    @property
    def height(self) -> float:
        return max(0.0, self.xyxy[3] - self.xyxy[1])

    @property
    def area(self) -> float:
        return self.width * self.height

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
    # model.track(..., persist=True) 로 호출됐을 때만 존재 — ByteTrack 이
    # 아직 ID를 못 붙인 새 박스는 None 이 섞여 나올 수 있다.
    raw_track_ids = getattr(boxes, "id", None)
    track_ids = _plain(raw_track_ids) if raw_track_ids is not None else [None] * len(xyxy)
    if not (len(xyxy) == len(scores) == len(class_ids) == len(track_ids)):
        raise ValueError("YOLO box property lengths differ")
    detections = []
    for box, raw_score, raw_class_id, raw_track_id in zip(
        xyxy, scores, class_ids, track_ids, strict=True
    ):
        if len(box) != 4:
            raise ValueError("xyxy must contain four coordinates")
        coords = tuple(float(value) for value in box)
        score, class_id = float(raw_score), float(raw_class_id)
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
        track_id = None if raw_track_id is None else int(raw_track_id)
        detections.append(Detection(int(class_id), score, clipped, track_id))
    return (width, height), tuple(detections)


def validate_model_classes(
    model_names: Mapping[int, str] | Sequence[str],
    class_map: Mapping[int, str],
    expected_names: Mapping[int, str],
    torso_class_id: int | None = None,
    torso_class_name: str = "torso",
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
    if torso_class_id is not None:
        try:
            actual_torso_name = model_names[torso_class_id]
        except (KeyError, IndexError) as error:
            raise ValueError(f"model has no torso class ID {torso_class_id}") from error
        if actual_torso_name != torso_class_name:
            raise ValueError(
                f"torso class ID {torso_class_id}: expected {torso_class_name!r}, "
                f"model reports {actual_torso_name!r}"
            )


def torso_matches(
    animals: Sequence[Detection], torsos: Sequence[Detection]
) -> dict[int, int]:
    """Best one-to-one pairing of torso boxes to animal boxes (by index).

    A torso is only eligible for an animal whose bbox contains the torso's
    center. Among eligible pairs, prefer high containment (how much of the
    torso box overlaps the animal box) and a torso center close to the
    animal's own center. Animals may be left unmatched (not every animal has
    a confidently detected torso this frame).
    """
    if not animals or not torsos:
        return {}
    n, torso_count = len(animals), len(torsos)
    # Each animal gets a private dummy column so it may remain unmatched.
    scores = [[-1_000_000.0] * torso_count + [0.0] * n for _ in animals]
    for ai, animal in enumerate(animals):
        ax1, ay1, ax2, ay2 = animal.xyxy
        acx, acy = animal.center
        for ti, torso in enumerate(torsos):
            if torso.area <= 0 or animal.area <= 0:
                continue
            tcx, tcy = torso.center
            if not (ax1 <= tcx <= ax2 and ay1 <= tcy <= ay2):
                continue
            tx1, ty1, tx2, ty2 = torso.xyxy
            intersection = (max(0.0, min(ax2, tx2) - max(ax1, tx1))
                            * max(0.0, min(ay2, ty2) - max(ay1, ty1)))
            containment = intersection / torso.area
            if containment <= 0:
                continue
            center_distance = math.hypot(tcx - acx, tcy - acy) / max(
                math.hypot(animal.width, animal.height), 1.0)
            # The large base rewards the number of valid pairs first.
            scores[ai][ti] = 100.0 + 10.0 * containment - center_distance

    # Rectangular Hungarian assignment: n rows, torso_count + n columns.
    column_count = torso_count + n
    u = [0.0] * (n + 1)
    v = [0.0] * (column_count + 1)
    assigned_row = [0] * (column_count + 1)
    predecessor = [0] * (column_count + 1)
    for row in range(1, n + 1):
        assigned_row[0] = row
        column = 0
        minimum = [float("inf")] * (column_count + 1)
        used = [False] * (column_count + 1)
        while True:
            used[column] = True
            current_row = assigned_row[column]
            change = float("inf")
            next_column = 0
            for candidate_column in range(1, column_count + 1):
                if used[candidate_column]:
                    continue
                reduced_cost = (-scores[current_row - 1][candidate_column - 1]
                                 - u[current_row] - v[candidate_column])
                if reduced_cost < minimum[candidate_column]:
                    minimum[candidate_column] = reduced_cost
                    predecessor[candidate_column] = column
                if minimum[candidate_column] < change:
                    change = minimum[candidate_column]
                    next_column = candidate_column
            for candidate_column in range(column_count + 1):
                if used[candidate_column]:
                    u[assigned_row[candidate_column]] += change
                    v[candidate_column] -= change
                elif candidate_column:
                    minimum[candidate_column] -= change
            column = next_column
            if assigned_row[column] == 0:
                break
        while True:
            previous = predecessor[column]
            assigned_row[column] = assigned_row[previous]
            column = previous
            if column == 0:
                break
    matches: dict[int, int] = {}
    for column in range(1, torso_count + 1):
        row = assigned_row[column]
        if row and scores[row - 1][column - 1] > 0:
            matches[row - 1] = column - 1
    return matches


class AreaMonitor:
    """Tracks one locked target's bbox-area trend to detect it backing away.

    Times are in the same unit the caller passes as ``timestamp`` (seconds,
    any monotonically increasing clock). Invalid/missing samples reset
    smoothing and the hold timer. A completed baseline stays fixed until the
    monitor is replaced (new target locked).
    """

    def __init__(self, *, baseline_seconds: float, smooth_seconds: float,
                 hold_seconds: float, ratio_threshold: float,
                 edge_margin_px: float) -> None:
        self.baseline_seconds = baseline_seconds
        self.smooth_seconds = smooth_seconds
        self.hold_seconds = hold_seconds
        self.ratio_threshold = ratio_threshold
        self.edge_margin_px = edge_margin_px
        self.baseline: float | None = None
        self.baseline_samples: list[tuple[float, float]] = []
        self.samples: deque[tuple[float, float]] = deque()
        self.smoothed: float | None = None
        self.ratio: float | None = None
        self.below_since: float | None = None
        self.below_seconds = 0.0
        self.valid = False

    def update(self, timestamp: float, animal: Detection | None,
               width: int, height: int) -> bool:
        """Feed one frame's sample; returns True once the escape hold fires."""
        self.valid = (animal is not None and animal.area > 0
                      and animal.xyxy[0] > self.edge_margin_px
                      and animal.xyxy[1] > self.edge_margin_px
                      and animal.xyxy[2] < width - self.edge_margin_px
                      and animal.xyxy[3] < height - self.edge_margin_px)
        self.smoothed = self.ratio = None
        if not self.valid:
            self.samples.clear()
            self.baseline_samples.clear()
            self.below_since = None
            self.below_seconds = 0.0
            return False
        area = animal.area
        if self.baseline is None:
            self.baseline_samples.append((timestamp, area))
            if timestamp - self.baseline_samples[0][0] + 1e-9 >= self.baseline_seconds:
                self.baseline = float(median(a for _, a in self.baseline_samples))
                self.baseline_samples.clear()
            return False
        self.samples.append((timestamp, area))
        # Retain one sample at/before the window boundary to cover its duration.
        while len(self.samples) > 1 and self.samples[1][0] <= timestamp - self.smooth_seconds:
            self.samples.popleft()
        if timestamp - self.samples[0][0] + 1e-9 < self.smooth_seconds:
            return False
        self.smoothed = float(median(a for _, a in self.samples))
        self.ratio = self.smoothed / self.baseline
        if self.ratio <= self.ratio_threshold:
            if self.below_since is None:
                self.below_since = timestamp
            self.below_seconds = timestamp - self.below_since
        else:
            self.below_since = None
            self.below_seconds = 0.0
        return self.below_since is not None and self.below_seconds + 1e-9 >= self.hold_seconds


AREA_REASON = "bbox_area_reduced"
RAPID_MOVEMENT_REASON = "rapid_movement_detected"

STATE_SEARCH = "동물 탐지중"
STATE_BASELINE = "기준 면적 측정중"
STATE_ACTIVE = "추적중"
STATE_EXIT = "추적종료, 다시 탐지중"
STATE_LOST = "추적 실패"
STATE_TORSO_MISSING = "몸통 미검출"


@dataclass(frozen=True, slots=True)
class EscapeStep:
    selected: Detection | None
    matched_torso: Detection | None
    state: str
    terminated: bool
    termination_reason: str


class EscapeTracker:
    """Locks onto one track ID and releases it once it looks like it fled.

    Picks the largest eligible animal (track ID required) when no target is
    locked. While locked, feeds the target's bbox into an AreaMonitor (bbox
    area shrinking = backing away) and tracks its matched torso's movement
    over a short window (fast movement = fleeing). Either condition firing
    releases the lock and starts a short grace period before a new target
    can be picked, mirroring the original offline analysis script
    (animal/animal_escape_tracking_combined.py) but driven by wall-clock
    timestamps instead of a fixed video frame rate.
    """

    def __init__(self, *, track_lost_seconds: float, reselect_grace_seconds: float,
                 area_baseline_seconds: float, area_smooth_seconds: float,
                 area_hold_seconds: float, area_ratio_threshold: float,
                 edge_margin_px: float, torso_window_size: int,
                 torso_speed_threshold: float) -> None:
        if track_lost_seconds <= 0:
            raise ValueError("track_lost_seconds must be > 0")
        if torso_window_size < 2:
            raise ValueError("torso_window_size must be >= 2")
        if torso_speed_threshold <= 0:
            raise ValueError("torso_speed_threshold must be > 0")
        self.track_lost_seconds = track_lost_seconds
        self.reselect_grace_seconds = reselect_grace_seconds
        self.torso_window_size = torso_window_size
        self.torso_speed_threshold = torso_speed_threshold
        self._area_kwargs = dict(
            baseline_seconds=area_baseline_seconds,
            smooth_seconds=area_smooth_seconds,
            hold_seconds=area_hold_seconds,
            ratio_threshold=area_ratio_threshold,
            edge_margin_px=edge_margin_px,
        )

        self.target_id: int | None = None
        self.target_class: int | None = None
        self._lost_since: float | None = None  # 타겟이 안 보이기 시작한 시각
        self._area_monitor = AreaMonitor(**self._area_kwargs)
        self._history: deque[tuple[int, float, float, float]] = deque(
            maxlen=torso_window_size)
        # 해제된 ID -> 마지막으로 보였던(= 계속 보이는 중이면 매 프레임 갱신)
        # 이후 안 보이기 시작한 시각. None 이면 아직 계속 보이는 중(= 여전히
        # 재선정 금지).
        self._excluded: dict[int, float | None] = {}
        self._frame_number = 0
        self._reselect_at: float | None = None

    def step(self, timestamp: float, animals: Sequence[Detection],
             torsos: Sequence[Detection], width: int, height: int) -> EscapeStep:
        self._frame_number += 1
        visible_ids = {d.track_id for d in animals if d.track_id is not None}
        for banned_id, absent_since in list(self._excluded.items()):
            if banned_id in visible_ids:
                self._excluded[banned_id] = None  # 아직 화면에 있음 — 재선정 계속 금지
            else:
                if absent_since is None:
                    self._excluded[banned_id] = timestamp
                elif timestamp - absent_since >= self.track_lost_seconds:
                    del self._excluded[banned_id]

        selected = (next((d for d in animals if d.track_id == self.target_id), None)
                    if self.target_id is not None else None)

        if (self.target_id is None
                and (self._reselect_at is None or timestamp >= self._reselect_at)):
            eligible = [d for d in animals
                        if d.track_id is not None and d.track_id not in self._excluded]
            if eligible:
                selected = max(eligible, key=lambda d: d.area)
                self.target_id = selected.track_id
                self.target_class = selected.class_id
                self._lost_since = None
                self._area_monitor = AreaMonitor(**self._area_kwargs)
                self._history.clear()
                self._reselect_at = None

        if self.target_id is None:
            self._area_monitor = AreaMonitor(**self._area_kwargs)
            self._history.clear()
            state = STATE_EXIT if (self._reselect_at is not None
                                    and timestamp < self._reselect_at) else STATE_SEARCH
            return EscapeStep(None, None, state, False, "")

        area_cancel = self._area_monitor.update(timestamp, selected, width, height)
        matched_torso: Detection | None = None
        speed: float | None = None
        rapid_movement = False

        if selected is None:
            # AI 탐지가 몇 프레임 빠지는 수준의 일시적 누락은 여기서 흡수한다
            # — Tracker 는 마지막으로 받은 목표 위치를 계속 토크로 유지하고
            # 있으므로(motor/dm4310_tracker.py 의 제어 스레드), 여기서 좌표를
            # 따로 기억해 다시 보낼 필요는 없다. track_lost_seconds 를 넘겨서
            # 계속 안 보이면 그때 실제로 lock 을 해제한다.
            self._history.clear()
            if self._lost_since is None:
                self._lost_since = timestamp
            elif timestamp - self._lost_since >= self.track_lost_seconds:
                self.target_id = None
                self.target_class = None
                self._lost_since = None
                return EscapeStep(None, None, STATE_LOST, True, "tracking_lost")
            return EscapeStep(None, None, STATE_LOST, False, "")

        self._lost_since = None
        ai = next(i for i, animal in enumerate(animals) if animal is selected)
        matches = torso_matches(animals, torsos)
        ti = matches.get(ai)
        if ti is None:
            self._history.clear()
            state = STATE_TORSO_MISSING
        else:
            matched_torso = torsos[ti]
            if self._history and self._history[-1][0] != self._frame_number - 1:
                self._history.clear()
            cx, cy = matched_torso.center
            self._history.append((self._frame_number, cx, cy, selected.width))
            if len(self._history) == self.torso_window_size:
                first, last = self._history[0], self._history[-1]
                mean_width = sum(item[3] for item in self._history) / self.torso_window_size
                if mean_width > 0:
                    speed = (math.hypot(last[1] - first[1], last[2] - first[2])
                             / ((self.torso_window_size - 1) * mean_width))
                    rapid_movement = speed >= self.torso_speed_threshold
            state = STATE_ACTIVE if self._area_monitor.baseline is not None else STATE_BASELINE

        if self._area_monitor.baseline is None:
            state = STATE_BASELINE

        reasons = [name for name, fired in
                   ((AREA_REASON, area_cancel), (RAPID_MOVEMENT_REASON, rapid_movement))
                   if fired]
        if reasons:
            reason = "+".join(reasons)
            self._excluded[self.target_id] = None  # 아직 화면에 보이는 중 — 재선정 금지 시작
            self._reselect_at = timestamp + self.reselect_grace_seconds
            self.target_id = None
            self.target_class = None
            self._history.clear()
            return EscapeStep(selected, matched_torso, STATE_EXIT, True, reason)

        return EscapeStep(selected, matched_torso, state, False, "")


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
                 catalog: AudioCatalog, player: object,
                 escape_tracker: EscapeTracker,
                 torso_class_id: int, torso_confidence: float,
                 tracker_config: str, camera: object | None = None) -> None:
        if not 0 <= min_selection_confidence <= 1:
            raise ValueError("min_selection_confidence must be within [0, 1]")
        if not 0 <= torso_confidence <= 1:
            raise ValueError("torso_confidence must be within [0, 1]")
        self.class_map = class_map
        self.expected_names = expected_names
        self.min_selection_confidence = min_selection_confidence
        self.policy = policy
        self.catalog = catalog
        self.player = player
        self.escape_tracker = escape_tracker
        self.torso_class_id = torso_class_id
        self.torso_confidence = torso_confidence
        self.tracker_config = tracker_config
        self.camera = camera
        self.last_escape_step: EscapeStep | None = None  # 호출부 디버그 표시용

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
            escape_tracker=EscapeTracker(
                track_lost_seconds=settings.TRACK_LOST_SECONDS,
                reselect_grace_seconds=settings.RESELECT_GRACE_SECONDS,
                area_baseline_seconds=settings.AREA_BASELINE_SECONDS,
                area_smooth_seconds=settings.AREA_SMOOTH_SECONDS,
                area_hold_seconds=settings.AREA_HOLD_SECONDS,
                area_ratio_threshold=settings.AREA_RATIO_THRESHOLD,
                edge_margin_px=settings.EDGE_MARGIN_PX,
                torso_window_size=settings.TORSO_WINDOW_SIZE,
                torso_speed_threshold=settings.TORSO_SPEED_THRESHOLD,
            ),
            torso_class_id=settings.TORSO_CLASS_ID,
            torso_confidence=settings.TORSO_TRACK_CONFIDENCE,
            tracker_config=settings.TRACKER_CONFIG,
            camera=camera,
        )

    def process_result(self, raw: object, captured_at: float) -> Event:
        """Lock one track ID via self.escape_tracker and only aim/play for
        that locked, not-yet-fled target — not simply this frame's highest
        confidence detection (see EscapeTracker for why)."""
        frame_size, detections = read_detections(raw)
        width, height = frame_size
        animals = [d for d in detections
                   if d.class_id in self.class_map
                   and d.confidence >= self.min_selection_confidence]
        torsos = [d for d in detections
                  if d.class_id == self.torso_class_id
                  and d.confidence >= self.torso_confidence]
        step = self.escape_tracker.step(captured_at, animals, torsos, width, height)
        self.last_escape_step = step
        target = step.selected

        if target is None:
            self.policy.should_play(None, 0.0, captured_at)
            return Event(None, None, None, step.termination_reason or "no_target")

        # 몸통(torso) 중심을 조준한다 — 동물 전체 bbox 중심은 다리·꼬리 등
        # 멀리 뻗은 부위까지 포함해 흔들리므로, torso가 매칭된 동안은 그쪽이
        # 더 안정적인 조준점이다. torso를 못 찾은 프레임(STATE_TORSO_MISSING)
        # 에는 동물 bbox 중심으로 폴백한다.
        aim_target = step.matched_torso if step.matched_torso is not None else target
        if self.camera is not None:
            try:
                self.camera.aim(aim_target, frame_size)
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
    # model.track(persist=True) 로 프레임마다 같은 개체에 같은 track ID를
    # 유지시킨다 — EscapeTracker 가 그 ID로 타겟을 lock 한다. conf 는 두
    # 클래스(동물/torso) 중 더 낮은 쪽으로 넉넉히 잡아 두고, 실제 채택
    # 기준은 WildlifeDeterrent.process_result() 에서 클래스별로 따로 건다.
    conf = min(service.min_selection_confidence, service.torso_confidence)
    results = model.track(frame, persist=True, tracker=service.tracker_config,
                          conf=conf, verbose=False)
    if len(results) != 1:
        raise ValueError("wildlife model must return one result per frame")
    return service.process_result(results[0], captured_at)
