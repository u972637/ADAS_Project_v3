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


def _nearest_torso(
    center: tuple[float, float], max_distance: float, torsos: Sequence[Detection]
) -> Detection | None:
    """Closest torso to center within max_distance pixels.

    Used to bridge a momentary animal-bbox gap by distance to the last
    KNOWN TORSO POSITION (updated every frame), not a frozen animal bbox —
    a moving target keeps a moving search point instead of drifting outside
    a stale window."""
    cx0, cy0 = center
    best, best_dist = None, max_distance
    for torso in torsos:
        if torso.area <= 0:
            continue
        tcx, tcy = torso.center
        dist = math.hypot(tcx - cx0, tcy - cy0)
        if dist <= best_dist:
            best, best_dist = torso, dist
    return best


def _reidentify_by_torso(
    last_torso: tuple[float, float, float] | None,
    target_class: int,
    excluded: Mapping[int, float | None],
    animals: Sequence[Detection],
    torsos: Sequence[Detection],
    radius_multiplier: float = 1.0,
    *,
    unrestricted: bool = False,
) -> tuple[Detection, Detection] | None:
    """Recover from a ByteTrack ID switch on the same real individual: a
    bbox (possibly under a NEW track_id) directly contains a torso
    (ordinary bbox-containment matching via torso_matches(), never a bare
    position-only torso -- this only ever considers a torso BACKED BY SOME
    BBOX this frame). Among qualifying candidates, the BIGGEST torso wins
    (not the nearest) -- a bigger torso is the more confident, closer-to-
    camera read on "this is probably the real individual."

    Normally (unrestricted=False) a candidate must also sit within
    radius_multiplier torso-widths of the last confirmed torso position
    (last_torso), for a quick same-spot ID-switch recovery. Once the
    locked individual's own bbox has been missing long enough
    (EscapeTracker.bbox_retarget_grace_seconds), the caller passes
    unrestricted=True to drop that radius limit entirely and search the
    whole frame -- giving up on "probably the same exact individual" in
    favor of "whichever same-species animal is most confidently visible
    right now." Always restricted to the same species as the current lock
    -- never bridges a species change."""
    matches = torso_matches(animals, torsos)
    if not unrestricted:
        if last_torso is None:
            return None
        cx0, cy0 = last_torso[:2]
        max_distance = last_torso[2] * radius_multiplier
    best, best_area = None, -1.0
    for ai, animal in enumerate(animals):
        if (
            animal.track_id is None
            or animal.track_id in excluded
            or animal.class_id != target_class
            or ai not in matches
        ):
            continue
        torso = torsos[matches[ai]]
        if not unrestricted:
            tcx, tcy = torso.center
            if math.hypot(tcx - cx0, tcy - cy0) > max_distance:
                continue
        if torso.area > best_area:
            best, best_area = (animal, torso), torso.area
    return best


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
        bbox_retarget_grace_seconds: float = 1.0,
    ) -> None:
        if track_lost_seconds <= 0:
            raise ValueError("track_lost_seconds must be > 0")
        if min_bbox_height_px < 0:
            raise ValueError("min_bbox_height_px must be >= 0")
        if torso_bridge_radius_multiplier <= 0:
            raise ValueError("torso_bridge_radius_multiplier must be > 0")
        if bbox_retarget_grace_seconds <= 0:
            raise ValueError("bbox_retarget_grace_seconds must be > 0")
        self.track_lost_seconds = track_lost_seconds
        self.reselect_grace_seconds = reselect_grace_seconds
        # The position-based bridge (gap fallback / ID-switch reidentify)
        # searches within one torso-WIDTH of the last confirmed torso by
        # default (multiplier=1.0) -- fine for a single dropped frame, but a
        # real animal can easily move more than its own body-width during a
        # half-second-plus gap, which made a re-appearing torso stay
        # unmatched (yellow, never red again) until the full
        # track_lost_seconds timeout released and re-selected it from
        # scratch. Widen the search radius by this multiplier to tolerate
        # realistic movement during short gaps.
        self.torso_bridge_radius_multiplier = torso_bridge_radius_multiplier
        # Once the locked individual's own bbox (not just its torso) has
        # been absent for this long, ID-switch reidentify stops requiring
        # the candidate to be near the last torso position: it searches the
        # WHOLE frame for any same-species torso that's confirmed by some
        # bbox this frame, preferring the biggest. Below this grace period,
        # reidentify stays radius-limited (quick recovery of the SAME
        # individual nearby); the plain position-only bridge (no bbox
        # anywhere in frame) is unaffected either way.
        self.bbox_retarget_grace_seconds = bbox_retarget_grace_seconds
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
        # 해제된 ID -> 마지막으로 보였던(= 계속 보이는 중이면 매 프레임 갱신)
        # 이후 안 보이기 시작한 시각. None 이면 아직 계속 보이는 중(= 여전히
        # 재선정 금지).
        self._excluded: dict[int, float | None] = {}
        self._reselect_at: float | None = None
        # Last known (center_x, center_y, width) of the locked target's
        # matched torso, updated every frame it's found (live match or
        # bridged). Lets a momentary ByteTrack gap (animal box flickering
        # off) keep following the torso near where it actually is now,
        # rather than a frozen box from the last real sighting.
        self._last_torso: tuple[float, float, float] | None = None
        # Time since the locked individual's own bbox (selected) was last
        # actually present -- independent of self._lost_since, which is
        # purely about torso-match success (bbox or not). Drives the
        # reidentify widening above; never releases the lock by itself.
        self._bbox_lost_since: float | None = None

    def step(
        self,
        timestamp: float,
        animals: Sequence[Detection],
        torsos: Sequence[Detection],
        width: int,
        height: int,
        *,
        observe: bool = True,
    ) -> EscapeStep:
        visible_ids = {d.track_id for d in animals if d.track_id is not None}
        for banned_id, absent_since in list(self._excluded.items()):
            if banned_id in visible_ids:
                self._excluded[banned_id] = None  # 아직 화면에 있음 — 재선정 계속 금지
            else:
                if absent_since is None:
                    self._excluded[banned_id] = timestamp
                elif timestamp - absent_since >= self.track_lost_seconds:
                    del self._excluded[banned_id]

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
            eligible = [
                d
                for d in animals
                if d.track_id is not None and d.track_id not in self._excluded
            ]
            if eligible:
                selected = max(eligible, key=lambda d: d.area)
                self.target_id = selected.track_id
                self.target_class = selected.class_id
                self._lost_since = None
                self._bbox_lost_since = None
                self._area_monitor = AreaMonitor(**self._area_kwargs)
                self._reselect_at = None

        if self.target_id is None:
            self._area_monitor = AreaMonitor(**self._area_kwargs)
            state = (
                STATE_EXIT
                if (self._reselect_at is not None and timestamp < self._reselect_at)
                else STATE_SEARCH
            )
            return EscapeStep(None, None, state, False, "")

        area_cancel = (
            self._area_monitor.update(timestamp, selected, width, height)
            if observe
            else False
        )
        matched_torso: Detection | None = None

        if selected is None:
            # Only bridge a true ByteTrack gap (target_id absent entirely this
            # frame). If target_id reappears with a different class, that is
            # an identity conflict, not a flicker — never paper over it with
            # position.
            id_conflicted = self.target_id is not None and any(
                d.track_id == self.target_id for d in animals
            )
            # The locked individual's own bbox has been gone long enough
            # that "same exact spot" is no longer a meaningful constraint --
            # widen reidentify to the whole frame (still same species,
            # still bbox-confirmed, biggest torso wins).
            bbox_grace_expired = (
                self._bbox_lost_since is not None
                and timestamp - self._bbox_lost_since
                >= self.bbox_retarget_grace_seconds
            )
            reidentified = (
                _reidentify_by_torso(
                    self._last_torso,
                    self.target_class,
                    self._excluded,
                    animals,
                    torsos,
                    self.torso_bridge_radius_multiplier,
                    unrestricted=bbox_grace_expired,
                )
                if not id_conflicted and (self._last_torso is not None or bbox_grace_expired)
                else None
            )
            if reidentified is not None:
                # Same individual, new ByteTrack ID — a fresh bbox directly
                # matched a torso right where we last confirmed one. Adopt
                # the new ID and continue as if never lost, instead of
                # falling back to a torso-only position bridge.
                selected, matched_torso = reidentified
                self.target_id = selected.track_id
                self.target_class = selected.class_id
            elif self._last_torso is not None and not id_conflicted:
                # 동물 bbox만 이번 프레임에 빠졌다(ByteTrack 순간 누락) — torso는
                # 마지막 위치 근처에서 여전히 보이면 그 torso를 계속 따라간다.
                # 동물 bbox가 없어 면적 기반 판정은 이번 프레임엔 평가하지 않는다
                # (area_cancel은 이미 False).
                matched_torso = _nearest_torso(
                    self._last_torso[:2],
                    self._last_torso[2] * self.torso_bridge_radius_multiplier,
                    torsos,
                )
        else:
            ai = next(i for i, animal in enumerate(animals) if animal is selected)
            matches = torso_matches(animals, torsos)
            ti = matches.get(ai)
            if ti is not None:
                matched_torso = torsos[ti]
            elif self._last_torso is not None:
                # The animal's bbox is tracked fine this frame, but its
                # torso failed the strict bbox-containment match (occlusion,
                # jitter, or fast motion nudging the torso center just
                # outside the bbox edge for one frame) -- bridge with the
                # same position-based fallback used for a full bbox gap,
                # instead of treating one missed containment check as
                # "torso missing" and flipping red -> yellow despite the
                # lock itself being perfectly healthy.
                matched_torso = _nearest_torso(
                    self._last_torso[:2],
                    self._last_torso[2] * self.torso_bridge_radius_multiplier,
                    torsos,
                )

        # Tracks how long the locked individual's own bbox (not just its
        # torso) has been absent -- feeds bbox_grace_expired above, next
        # time. Resets on ANY bbox presence this frame, original track_id
        # or reidentified.
        if selected is None:
            if self._bbox_lost_since is None:
                self._bbox_lost_since = timestamp
        else:
            self._bbox_lost_since = None

        # Single lost-timer, driven purely by whether a torso was actually
        # matched this frame -- NOT by whether the animal bbox happened to
        # be present. A bbox that keeps appearing every frame while its
        # torso repeatedly fails to match (occlusion, bad angle, a
        # genuinely gone torso) no longer stalls forever in
        # STATE_TORSO_MISSING with no timeout; it releases on the same
        # track_lost_seconds clock as a full bbox gap.
        if matched_torso is None:
            if self._lost_since is None:
                self._lost_since = timestamp
            elif timestamp - self._lost_since >= self.track_lost_seconds:
                self.target_id = None
                self.target_class = None
                self._lost_since = None
                self._last_torso = None
                self._bbox_lost_since = None
                return EscapeStep(None, None, STATE_LOST, True, "tracking_lost")
        else:
            self._lost_since = None

        if matched_torso is None:
            state = STATE_TORSO_MISSING
        else:
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
            self._excluded[self.target_id] = (
                None  # 아직 화면에 보이는 중 — 재선정 금지 시작
            )
            self._reselect_at = timestamp + self.reselect_grace_seconds
            self.target_id = None
            self.target_class = None
            self._last_torso = None
            self._bbox_lost_since = None
            return EscapeStep(selected, matched_torso, STATE_EXIT, True, reason)

        return EscapeStep(selected, matched_torso, state, False, "")

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
        self._bbox_lost_since = None
        self._excluded.clear()
        self.reset_observation()
