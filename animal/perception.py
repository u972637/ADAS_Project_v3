"""Detection parsing, torso association, target locking and provisional exit detection."""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from statistics import median


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
    track_ids = (
        _plain(raw_track_ids) if raw_track_ids is not None else [None] * len(xyxy)
    )
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


def _containing_animal(
    animals: Sequence[Detection], torso: Detection
) -> Detection | None:
    """A bbox whose area holds the torso's center (any, not a one-to-one pairing)."""
    tcx, tcy = torso.center
    for animal in animals:
        x1, y1, x2, y2 = animal.xyxy
        if x1 <= tcx <= x2 and y1 <= tcy <= y2:
            return animal
    return None


def _match_by_position(
    last_torso: tuple[float, float, float],
    reference: tuple[float, float] | None,
    target_class: int | None,
    animals: Sequence[Detection],
    torsos: Sequence[Detection],
    pairing: Mapping[int, int],
    width_multiplier: float,
    min_radius_px: float,
) -> tuple[Detection | None, Detection | None]:
    """Pick the torso that is "the same one" as the tracked torso, by position.

    Candidates are torsos within a CIRCLE (center = `reference`, the previous
    frame's tracking point -- the last torso center while red, the current
    orange point while orange; radius = max(min_radius_px, width_multiplier *
    last torso width)). The nearest wins; ids and confidence are not looked at.
    If the candidate sits inside a green bbox of a DIFFERENT species it is
    skipped; a torso with no bbox around it carries no species, so it is
    accepted (a known, accepted risk). Returns (torso, owner_bbox_or_None).
    """
    cx0, cy0 = reference if reference is not None else last_torso[:2]
    radius = max(min_radius_px, last_torso[2] * width_multiplier)
    owner_of = {ti: animals[ai] for ai, ti in pairing.items()}
    candidates = sorted(
        (math.hypot(t.center[0] - cx0, t.center[1] - cy0), ti)
        for ti, t in enumerate(torsos)
        if t.area > 0
    )
    for distance, ti in candidates:
        if distance > radius:
            break
        owner = owner_of.get(ti)
        if owner is None:
            owner = _containing_animal(animals, torsos[ti])
        if owner is not None and owner.class_id != target_class:
            continue
        return torsos[ti], owner
    return None, None


def _rect_distance(point: tuple[float, float], xyxy) -> float:
    """0 if the point is inside the box, else the distance to its nearest edge."""
    x, y = point
    x1, y1, x2, y2 = xyxy
    return math.hypot(max(x1 - x, 0.0, x - x2), max(y1 - y, 0.0, y - y2))


def _find_bbox_near(
    last_torso: tuple[float, float, float],
    reference: tuple[float, float] | None,
    target_class: int | None,
    animals: Sequence[Detection],
    width_multiplier: float,
    min_radius_px: float,
) -> Detection | None:
    """The green bbox to follow by its center when no torso is matched.

    Same species only (a bbox tells us the species). It must lie within the
    same circle used for torsos (center = previous tracking point, radius =
    max(min_radius_px, width_multiplier * last torso width)), measured to the
    bbox RECTANGLE (0 if the point is inside it): a big bbox's center can be
    far from where its torso is. Nearest wins, ties broken by center distance.
    """
    cx0, cy0 = reference if reference is not None else last_torso[:2]
    radius = max(min_radius_px, last_torso[2] * width_multiplier)
    best = None
    for animal in animals:
        if animal.class_id != target_class or animal.area <= 0:
            continue
        distance = _rect_distance((cx0, cy0), animal.xyxy)
        if distance > radius:
            continue
        key = (distance, math.hypot(animal.center[0] - cx0, animal.center[1] - cy0))
        if best is None or key < best[0]:
            best = (key, animal)
    return None if best is None else best[1]


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
            intersection = max(0.0, min(ax2, tx2) - max(ax1, tx1)) * max(
                0.0, min(ay2, ty2) - max(ay1, ty1)
            )
            containment = intersection / torso.area
            if containment <= 0:
                continue
            center_distance = math.hypot(tcx - acx, tcy - acy) / max(
                math.hypot(animal.width, animal.height), 1.0
            )
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
                reduced_cost = (
                    -scores[current_row - 1][candidate_column - 1]
                    - u[current_row]
                    - v[candidate_column]
                )
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


DISTANCE_METRICS = ("and", "area", "height")


class AreaMonitor:
    """Tracks one locked target's bbox size trend to detect it backing away.

    Area and height share the baseline, smoothing window, validity rule and
    hold timer; ``distance_metric`` decides which ratios must be at or below
    their thresholds:
      "and"    area AND height (rejects width-only shrinkage when a turn
               preserves height; posture and occlusion still need evaluation)
      "area"   area only (the original offline script)
      "height" height only

    Times are in the same unit the caller passes as ``timestamp`` (seconds,
    any monotonically increasing clock). Invalid/missing samples reset
    smoothing and the hold timer. A completed baseline stays fixed until the
    monitor is replaced (new target locked).
    """

    def __init__(
        self,
        *,
        baseline_seconds: float,
        smooth_seconds: float,
        hold_seconds: float,
        ratio_threshold: float,
        edge_margin_px: float,
        height_ratio_threshold: float = 1.0,
        distance_metric: str = "area",
    ) -> None:
        if distance_metric not in DISTANCE_METRICS:
            raise ValueError(f"distance_metric must be one of {DISTANCE_METRICS}")
        self.baseline_seconds = baseline_seconds
        self.smooth_seconds = smooth_seconds
        self.hold_seconds = hold_seconds
        self.ratio_threshold = ratio_threshold
        self.height_ratio_threshold = height_ratio_threshold
        self.distance_metric = distance_metric
        self.edge_margin_px = edge_margin_px
        self.baseline: float | None = None  # area
        self.baseline_height: float | None = None
        self.baseline_samples: list[tuple[float, float, float]] = []
        self.samples: deque[tuple[float, float, float]] = deque()
        self.smoothed: float | None = None  # area
        self.ratio: float | None = None  # area
        self.height_ratio: float | None = None
        self.below_since: float | None = None
        self.below_seconds = 0.0
        self.valid = False

    def _shrunk(self) -> bool:
        area_ok = self.ratio <= self.ratio_threshold
        height_ok = self.height_ratio <= self.height_ratio_threshold
        if self.distance_metric == "area":
            return area_ok
        if self.distance_metric == "height":
            return height_ok
        return area_ok and height_ok

    def update(
        self, timestamp: float, animal: Detection | None, width: int, height: int
    ) -> bool:
        """Feed one frame's sample; returns True once the escape hold fires."""
        self.valid = (
            animal is not None
            and animal.area > 0
            and animal.xyxy[0] > self.edge_margin_px
            and animal.xyxy[1] > self.edge_margin_px
            and animal.xyxy[2] < width - self.edge_margin_px
            and animal.xyxy[3] < height - self.edge_margin_px
        )
        self.smoothed = self.ratio = self.height_ratio = None
        if not self.valid:
            self.samples.clear()
            self.baseline_samples.clear()
            self.below_since = None
            self.below_seconds = 0.0
            return False
        sample = (timestamp, animal.area, animal.height)
        if self.baseline is None:
            self.baseline_samples.append(sample)
            if timestamp - self.baseline_samples[0][0] + 1e-9 >= self.baseline_seconds:
                self.baseline = float(median(s[1] for s in self.baseline_samples))
                self.baseline_height = float(
                    median(s[2] for s in self.baseline_samples)
                )
                self.baseline_samples.clear()
            return False
        self.samples.append(sample)
        # Retain one sample at/before the window boundary to cover its duration.
        while (
            len(self.samples) > 1
            and self.samples[1][0] <= timestamp - self.smooth_seconds
        ):
            self.samples.popleft()
        if timestamp - self.samples[0][0] + 1e-9 < self.smooth_seconds:
            return False
        self.smoothed = float(median(s[1] for s in self.samples))
        self.ratio = self.smoothed / self.baseline
        self.height_ratio = (
            float(median(s[2] for s in self.samples)) / self.baseline_height
        )
        if self._shrunk():
            if self.below_since is None:
                self.below_since = timestamp
            self.below_seconds = timestamp - self.below_since
        else:
            self.below_since = None
            self.below_seconds = 0.0
        return (
            self.below_since is not None
            and self.below_seconds + 1e-9 >= self.hold_seconds
        )


AREA_REASON = "bbox_area_reduced"
TOO_SMALL_REASON = "bbox_too_small"

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
    # A green bbox followed by its CENTER because no torso could be matched
    # this frame (it is also `selected`). None whenever a torso is matched.
    followed_bbox: Detection | None = None


class EscapeTracker:
    """Locks onto one track ID and releases it once it looks like it fled.

    Picks the largest eligible animal (track ID required) when no target is
    locked. While locked, feeds the target's bbox into an AreaMonitor (bbox
    area/height shrinking = provisional backing away); that firing releases
    the lock and starts a short grace period before a new target can be
    picked, driven by wall-clock timestamps instead of a fixed video frame
    rate.
    """

    def __init__(
        self,
        *,
        track_lost_seconds: float,
        reselect_grace_seconds: float,
        area_baseline_seconds: float,
        area_smooth_seconds: float,
        area_hold_seconds: float,
        area_ratio_threshold: float,
        edge_margin_px: float,
        height_ratio_threshold: float = 1.0,
        distance_metric: str = "area",
        min_bbox_height_px: float = 0.0,
        torso_bridge_radius_multiplier: float = 1.0,
        torso_bridge_min_radius_px: float = 0.0,
    ) -> None:
        if track_lost_seconds <= 0:
            raise ValueError("track_lost_seconds must be > 0")
        if min_bbox_height_px < 0:
            raise ValueError("min_bbox_height_px must be >= 0")
        if torso_bridge_radius_multiplier <= 0:
            raise ValueError("torso_bridge_radius_multiplier must be > 0")
        if torso_bridge_min_radius_px < 0:
            raise ValueError("torso_bridge_min_radius_px must be >= 0")
        self.track_lost_seconds = track_lost_seconds
        self.reselect_grace_seconds = reselect_grace_seconds
        # Re-matching the tracked torso after a gap is by POSITION: the nearest
        # torso inside a circle around the previous tracking point, radius
        # max(torso_bridge_min_radius_px, torso_bridge_radius_multiplier *
        # last torso width). The fixed floor matters for small/far torsos,
        # whose width-based radius would be tiny next to how far a target (or
        # the camera) moves in a few frames.
        self.torso_bridge_radius_multiplier = torso_bridge_radius_multiplier
        self.torso_bridge_min_radius_px = torso_bridge_min_radius_px
        # Absolute floor, independent of the AreaMonitor's baseline-relative
        # ratio check: too far away / too small to track reliably, regardless
        # of what size it started at. 0 disables this check.
        self.min_bbox_height_px = min_bbox_height_px
        self._area_kwargs = dict(
            baseline_seconds=area_baseline_seconds,
            smooth_seconds=area_smooth_seconds,
            hold_seconds=area_hold_seconds,
            ratio_threshold=area_ratio_threshold,
            height_ratio_threshold=height_ratio_threshold,
            distance_metric=distance_metric,
            edge_margin_px=edge_margin_px,
        )

        self.target_id: int | None = None
        self.target_class: int | None = None
        self._lost_since: float | None = None  # 타겟이 안 보이기 시작한 시각
        self._area_monitor = AreaMonitor(**self._area_kwargs)
        # After a release (fled / lost) NO new lock is allowed until this time,
        # whatever the ids are (re-locking the same animal is allowed after).
        self._reselect_at: float | None = None
        # The bbox id the area/height baseline was measured on. When the target
        # bbox changes to another id the baseline no longer describes it (it
        # may be a different animal), so it is re-measured from scratch.
        self._monitored_id: int | None = None
        # Last known (center_x, center_y, width) of the locked target's
        # matched torso, updated every frame it's found (live match or
        # bridged). Lets a momentary ByteTrack gap (animal box flickering
        # off) keep following the torso near where it actually is now,
        # rather than a frozen box from the last real sighting.
        self._last_torso: tuple[float, float, float] | None = None

    def step(
        self,
        timestamp: float,
        animals: Sequence[Detection],
        torsos: Sequence[Detection],
        width: int,
        height: int,
        *,
        observe: bool = True,
        reference: tuple[float, float] | None = None,
    ) -> EscapeStep:
        """One frame. `reference` (pixels) overrides the center used to find
        the tracked torso again: the caller passes the current orange point
        while the motor is chasing the virtual target; None = the last torso."""
        selected = (
            next(
                (
                    d
                    for d in animals
                    if d.track_id == self.target_id and d.class_id == self.target_class
                ),
                None,
            )
            if self.target_id is not None
            else None
        )

        if self.target_id is None and (
            self._reselect_at is None or timestamp >= self._reselect_at
        ):
            eligible = [d for d in animals if d.track_id is not None]
            # A lock starts only on a bbox that ALREADY has a matched torso:
            # the aim point is the torso, so a bbox alone is not trackable.
            # Among those, the largest wins (not the largest overall -- a
            # bigger bbox with no torso must not block a trackable animal).
            with_torso = [
                eligible[i] for i in torso_matches(eligible, torsos)
            ] if eligible else []
            if with_torso:
                selected = max(with_torso, key=lambda d: d.area)
                self.target_id = selected.track_id
                self.target_class = selected.class_id
                self._lost_since = None
                self._area_monitor = AreaMonitor(**self._area_kwargs)
                self._monitored_id = selected.track_id
                self._reselect_at = None

        if self.target_id is None:
            self._area_monitor = AreaMonitor(**self._area_kwargs)
            state = (
                STATE_EXIT
                if (self._reselect_at is not None and timestamp < self._reselect_at)
                else STATE_SEARCH
            )
            return EscapeStep(None, None, state, False, "")

        # --- find the tracked torso in this frame ---------------------------
        by_id = selected  # the locked id's own bbox this frame, if it is visible
        matched_torso: Detection | None = None
        owner: Detection | None = None
        pairing = torso_matches(animals, torsos)  # animal index -> torso index
        if by_id is not None:
            ti = pairing.get(next(i for i, a in enumerate(animals) if a is by_id))
            if ti is not None:
                # Same id and the torso is inside its bbox: no distance check, so
                # a big camera move between two frames cannot lose a target that
                # is plainly in view.
                matched_torso, owner = torsos[ti], by_id
        if matched_torso is None and self._last_torso is not None:
            matched_torso, owner = _match_by_position(
                self._last_torso,
                reference,
                self.target_class,
                animals,
                torsos,
                pairing,
                self.torso_bridge_radius_multiplier,
                self.torso_bridge_min_radius_px,
            )
        if owner is not None:
            # The id is not the identity: whichever green bbox holds the matched
            # torso is the target bbox now (same animal re-id'd by ByteTrack, or
            # simply whoever is there). Species stays what the lock started with.
            selected = owner
            if owner.track_id is not None:
                self.target_id = owner.track_id
        # No torso anywhere: follow a bbox by its center instead (never to START
        # a lock -- that needs a torso -- only to keep one). The locked id's own
        # bbox if it is visible, else a green bbox of the same species near the
        # previous tracking point. A torso, when there is one, always wins.
        followed: Detection | None = None
        if matched_torso is None:
            if by_id is not None:
                followed = by_id
            elif self._last_torso is not None:
                followed = _find_bbox_near(
                    self._last_torso,
                    reference,
                    self.target_class,
                    animals,
                    self.torso_bridge_radius_multiplier,
                    self.torso_bridge_min_radius_px,
                )
            if followed is not None:
                selected = followed
                if followed.track_id is not None:
                    self.target_id = followed.track_id
        # A different bbox id than the one the baseline was measured on may be
        # a different animal: measure the baseline again.
        if selected is not None and selected.track_id != self._monitored_id:
            self._area_monitor = AreaMonitor(**self._area_kwargs)
            self._monitored_id = selected.track_id
        area_cancel = (
            self._area_monitor.update(timestamp, selected, width, height)
            if observe
            else False
        )

        # Single lost-timer, driven purely by whether a torso was actually
        # matched this frame -- NOT by whether the animal bbox happened to
        # be present. A bbox that keeps appearing every frame while its
        # torso repeatedly fails to match (occlusion, bad angle, a
        # genuinely gone torso) no longer stalls forever in
        # STATE_TORSO_MISSING with no timeout; it releases on the same
        # track_lost_seconds clock as a full bbox gap.
        if matched_torso is None and followed is None:
            if self._lost_since is None:
                self._lost_since = timestamp
            elif timestamp - self._lost_since >= self.track_lost_seconds:
                self.target_id = None
                self.target_class = None
                self._lost_since = None
                self._last_torso = None
                self._monitored_id = None
                self._reselect_at = timestamp + self.reselect_grace_seconds
                return EscapeStep(None, None, STATE_LOST, True, "tracking_lost")
        else:
            self._lost_since = None

        if matched_torso is not None:
            # Torsos are detected class-agnostic (always TORSO_CLASS_ID,
            # track_id=None — see animal/inference.py) since they never enter
            # ByteTrack themselves. Once one is confirmed as THIS locked
            # individual's torso (by whichever match path above), tag it
            # with that individual's track_id/species so callers (logging,
            # display) don't have to look the lock up separately.
            matched_torso = replace(
                matched_torso, track_id=self.target_id, class_id=self.target_class
            )
            state = (
                STATE_ACTIVE
                if self._area_monitor.baseline is not None
                else STATE_BASELINE
            )
            # 다음 프레임(특히 동물 bbox가 빠지는 경우)의 탐색 기준점을 매
            # 프레임 갱신 — 고정된 박스가 아니라 "마지막으로 본 torso 위치"가
            # 대상과 같이 움직이게 한다.
            self._last_torso = (*matched_torso.center, matched_torso.width)
        elif followed is not None:
            # The bbox center is the tracking point now (the next frame looks
            # for the torso around it); keep the last torso width for the radius.
            last_width = self._last_torso[2] if self._last_torso else followed.width
            self._last_torso = (*followed.center, last_width)
            state = STATE_TORSO_MISSING
        else:
            state = STATE_TORSO_MISSING

        if self._area_monitor.baseline is None:
            state = STATE_BASELINE

        if not observe:
            self.reset_observation()

        # Absolute floor, independent of the baseline-relative AreaMonitor
        # ratio and of `observe` — a gimbal pan doesn't change how far away
        # the animal actually is, so this is meaningful even while aiming.
        too_small = (
            selected is not None
            and self.min_bbox_height_px > 0
            and selected.height < self.min_bbox_height_px
        )

        reasons = [
            name
            for name, fired in (
                (AREA_REASON, area_cancel),
                (TOO_SMALL_REASON, too_small),
            )
            if fired
        ]
        if reasons:
            reason = "+".join(reasons)
            self._reselect_at = timestamp + self.reselect_grace_seconds
            self.target_id = None
            self.target_class = None
            self._last_torso = None
            self._monitored_id = None
            return EscapeStep(selected, matched_torso, STATE_EXIT, True, reason)

        return EscapeStep(selected, matched_torso, state, False, "", followed)

    def reset_observation(self) -> None:
        """Discard samples collected while the camera was moving."""
        self._area_monitor = AreaMonitor(**self._area_kwargs)

    @property
    def baseline_ready(self) -> bool:
        return self._area_monitor.baseline is not None

    def reset(self) -> None:
        self.target_id = self.target_class = None
        self._lost_since = self._reselect_at = None
        self._last_torso = None
        self._monitored_id = None
        self.reset_observation()
