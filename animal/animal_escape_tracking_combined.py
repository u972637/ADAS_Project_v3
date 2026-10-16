r"""Lock one animal ID and end tracking when EITHER escape condition fires:
  1) bbox area: smoothed area <= 50% of the initial baseline for one second
     (same rule as animal_escape_tracking.py)
  2) torso movement: 5-frame torso displacement / (4 x mean bbox width) >= 0.15
     (same rule as animal_escape_tracking_pixel_015.py)
Each condition keeps its original behavior; termination_reason records which
one fired. Both are image-based rules, not proof of distance, speed, or
successful deterrence. Keep camera zoom fixed. Audio state is logged only;
no hardware is driven.

Run in the existing YOLO11 environment with MODEL_PATH and VIDEO_PATH below:
    C:\ProgramData\anaconda3\envs\yolo11\python.exe animal_escape_tracking_combined.py
"""

from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from math import hypot
from pathlib import Path
from statistics import median

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO
from ultralytics.utils import YAML
from ultralytics.utils.checks import check_yaml


# Editable settings. Change VIDEO_PATH to run another Downloads video.
DOWNLOADS = Path.home() / "Downloads"
MODEL_PATH = DOWNLOADS / "weights.pt"
VIDEO_PATH = DOWNLOADS / "boar_testnir5.webm"
OUTPUT_DIR = DOWNLOADS / "tracking_results"
OUTPUT_VIDEO = OUTPUT_DIR / "tracking_result.mp4"
OUTPUT_CSV = OUTPUT_DIR / "tracking_result.csv"

ANIMAL_CONFIDENCE = 0.3
TORSO_CONFIDENCE = 0.2
CONFIDENCE = min(ANIMAL_CONFIDENCE, TORSO_CONFIDENCE)
TRACKER = "bytetrack.yaml"
PROCESSING_FPS = 25.0
TRACK_LOST_FRAMES = 10
EXIT_MESSAGE_DURATION = 1.0

# Condition 1: bbox area reduction.
AREA_RATIO_THRESHOLD = 0.50  # Remaining area: 50% means a 50% reduction.
BASELINE_SECONDS = 1.0
AREA_SMOOTH_SECONDS = 0.5
AREA_HOLD_SECONDS = 1.0
EDGE_MARGIN_PX = 2  # Reject potentially clipped animal bboxes.

# Condition 2: torso movement.
WINDOW_SIZE = 5
NORMALIZED_THRESHOLD = 0.15

STATE_SEARCH = "동물 탐지중"
STATE_ACTIVE = "음향 발사중"
STATE_EXIT = "추적종료, 다시 탐지중"
STATE_LOST = "추적 실패"
STATE_TORSO_MISSING = "몸통 미검출, 음향 정지"
STATE_BASELINE = "기준 면적 측정중"
AREA_REASON = "bbox_area_reduced"
RAPID_MOVEMENT_REASON = "rapid_movement_detected"
EVENT_MEANINGS = {
    AREA_REASON: "bbox 면적 감소",
    RAPID_MOVEMENT_REASON: "급속 이동 감지",
}

CSV_FIELDS = [
    "frame", "source_frame", "timestamp", "animal_class", "animal_track_id",
    "animal_detected", "torso_detected", "torso_matched", "audio_active",
    "animal_confidence", "animal_bbox_width", "animal_bbox_height",
    "torso_confidence", "torso_center_x", "torso_center_y", "torso_bbox_width",
    "torso_bbox_height", "torso_match_success", "animal_bbox_area",
    "baseline_area", "smoothed_area", "area_ratio", "area_below_seconds",
    "area_sample_valid", "area_ratio_threshold", "normalized_speed",
    "speed_threshold", "system_state", "tracking_terminated", "termination_reason",
]
EVENT_FIELDS = [
    "frame", "timestamp", "animal_class", "animal_track_id", "termination_reason",
    "baseline_area", "smoothed_area", "area_ratio", "area_ratio_threshold",
    "normalized_speed", "speed_threshold", "event_meaning",
]


@dataclass(frozen=True)
class Detection:
    cls_name: str
    confidence: float
    box: tuple[float, float, float, float]
    track_id: int | None

    @property
    def width(self) -> float:
        return max(0.0, self.box[2] - self.box[0])

    @property
    def height(self) -> float:
        return max(0.0, self.box[3] - self.box[1])

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


class AreaMonitor:
    """One instance per selected ID; times are source-video seconds.

    Invalid/missing samples reset smoothing and the hold timer. A completed
    baseline stays fixed until a new target is selected. Partial baseline
    samples are discarded after a gap. No torso detection is required.
    """

    def __init__(self) -> None:
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
        self.valid = (animal is not None and animal.area > 0
                      and animal.box[0] > EDGE_MARGIN_PX
                      and animal.box[1] > EDGE_MARGIN_PX
                      and animal.box[2] < width - EDGE_MARGIN_PX
                      and animal.box[3] < height - EDGE_MARGIN_PX)
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
            if timestamp - self.baseline_samples[0][0] + 1e-9 >= BASELINE_SECONDS:
                self.baseline = float(median(a for _, a in self.baseline_samples))
                self.baseline_samples.clear()
            return False
        self.samples.append((timestamp, area))
        # Retain one sample at/before the window boundary to cover its duration.
        while len(self.samples) > 1 and self.samples[1][0] <= timestamp - AREA_SMOOTH_SECONDS:
            self.samples.popleft()
        if timestamp - self.samples[0][0] + 1e-9 < AREA_SMOOTH_SECONDS:
            return False
        self.smoothed = float(median(a for _, a in self.samples))
        self.ratio = self.smoothed / self.baseline
        if self.ratio <= AREA_RATIO_THRESHOLD:
            if self.below_since is None:
                self.below_since = timestamp
            self.below_seconds = timestamp - self.below_since
        else:
            self.below_since = None
            self.below_seconds = 0.0
        return self.below_since is not None and self.below_seconds + 1e-9 >= AREA_HOLD_SECONDS


def validate_inputs() -> None:
    for path in (MODEL_PATH, VIDEO_PATH):
        if not path.is_file():
            raise FileNotFoundError(f"입력 파일이 없습니다: {path}")
    if TRACK_LOST_FRAMES < 1 or not 0 < AREA_RATIO_THRESHOLD < 1:
        raise ValueError("TRACK_LOST_FRAMES >= 1, 0 < AREA_RATIO_THRESHOLD < 1 필요")
    if min(BASELINE_SECONDS, AREA_SMOOTH_SECONDS, AREA_HOLD_SECONDS) <= 0 or EDGE_MARGIN_PX < 0:
        raise ValueError("면적 판단 시간은 양수, 가장자리 여백은 0 이상이어야 합니다")
    if WINDOW_SIZE < 2 or NORMALIZED_THRESHOLD <= 0:
        raise ValueError("WINDOW_SIZE >= 2, NORMALIZED_THRESHOLD > 0이어야 합니다")


def new_run_directory() -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    for index in range(1000):
        run_dir = OUTPUT_DIR / (stamp if index == 0 else f"{stamp}_{index:03d}")
        try:
            run_dir.mkdir(exist_ok=False)
            return run_dir
        except FileExistsError:
            continue
    raise RuntimeError("고유한 결과 폴더를 만들 수 없습니다")


def parse_detections(result: object, names: dict[int, str]) -> list[Detection]:
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    coords = boxes.xyxy.cpu().tolist()
    classes = boxes.cls.int().cpu().tolist()
    confidences = boxes.conf.cpu().tolist()
    ids = boxes.id.int().cpu().tolist() if boxes.id is not None else [None] * len(boxes)
    return [
        Detection(str(names[int(cls)]).lower(), float(conf), tuple(map(float, xyxy)),
                  None if track_id is None else int(track_id))
        for xyxy, cls, conf, track_id in zip(coords, classes, confidences, ids)
    ]


def torso_matches(animals: list[Detection], torsos: list[Detection]) -> dict[int, int]:
    """Find the maximum-quality one-to-one pairing, allowing unmatched animals."""
    if not animals or not torsos:
        return {}
    n, torso_count = len(animals), len(torsos)
    # Each animal gets a private dummy column so it may remain unmatched.
    scores = [[-1_000_000.0] * torso_count + [0.0] * n for _ in animals]
    for ai, animal in enumerate(animals):
        ax1, ay1, ax2, ay2 = animal.box
        acx, acy = animal.center
        for ti, torso in enumerate(torsos):
            if torso.area <= 0 or animal.area <= 0:
                continue
            tcx, tcy = torso.center
            if not (ax1 <= tcx <= ax2 and ay1 <= tcy <= ay2):
                continue
            tx1, ty1, tx2, ty2 = torso.box
            intersection = (max(0.0, min(ax2, tx2) - max(ax1, tx1))
                            * max(0.0, min(ay2, ty2) - max(ay1, ty1)))
            containment = intersection / torso.area
            if containment <= 0:
                continue
            center_distance = hypot(tcx - acx, tcy - acy) / max(hypot(animal.width, animal.height), 1.0)
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
                reduced_cost = -scores[current_row - 1][candidate_column - 1] - u[current_row] - v[candidate_column]
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


def korean_font(size: int) -> ImageFont.FreeTypeFont:
    for path in (Path("C:/Windows/Fonts/malgun.ttf"), Path("C:/Windows/Fonts/NanumGothic.ttf")):
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    raise FileNotFoundError("한글 표시용 폰트가 없습니다 (맑은 고딕 또는 나눔고딕 필요)")


def render(frame_image, animals: list[Detection], torsos: list[Detection],
           matches: dict[int, int], selected: Detection | None,
           state: str, target_id: int | None, target_class: str | None,
           area_monitor: AreaMonitor, speed: float | None, frame_number: int,
           fps: float, lost_count: int) -> object:
    selected_index = None
    for ai, animal in enumerate(animals):
        if selected is animal:
            selected_index = ai
        x1, y1, x2, y2 = map(int, animal.box)
        is_selected = selected is animal
        color = (0, 220, 255) if is_selected else (40, 200, 40)
        cv2.rectangle(frame_image, (x1, y1), (x2, y2), color, 3 if is_selected else 2)
        # Show each animal's tracker ID independently of torso detection
        # and target selection. Never invent an ID when tracking is pending.
        id_text = str(animal.track_id) if animal.track_id is not None else "pending"
        label = f"{animal.cls_name} {animal.confidence:.2f} ID:{id_text}"
        cv2.putText(frame_image, label, (max(0, x1), max(16, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.53, color, 2, cv2.LINE_AA)
    selected_torso_index = matches.get(selected_index) if selected_index is not None else None
    if selected_torso_index is not None:
        torso = torsos[selected_torso_index]
        x1, y1, x2, y2 = map(int, torso.box)
        color = (255, 255, 0)
        cv2.rectangle(frame_image, (x1, y1), (x2, y2), color, 3)
        cv2.circle(frame_image, tuple(map(int, torso.center)), 5, color, -1)
        cv2.putText(frame_image, f"TARGET TORSO {torso.confidence:.2f}", (max(0, x1), max(16, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

    lines = [
        f"상태: {state}",
        f"Target ID: {target_id if target_id is not None else 'N/A'}",
        f"Class: {target_class or 'N/A'}",
        (f"[면적] ratio: {area_monitor.ratio:.3f} (<= {AREA_RATIO_THRESHOLD:.2f}, {AREA_HOLD_SECONDS:.1f}s)"
         if area_monitor.ratio is not None
         else f"[면적] ratio: N/A (<= {AREA_RATIO_THRESHOLD:.2f}, {AREA_HOLD_SECONDS:.1f}s)"),
        f"[면적] Baseline: {area_monitor.baseline:.0f} px2" if area_monitor.baseline is not None else "[면적] Baseline: collecting",
        f"[면적] Below timer: {area_monitor.below_seconds:.2f}s / sample: {'valid' if area_monitor.valid else 'missing / edge'}",
        (f"[이동] Normalized Speed: {speed:.3f} (>= {NORMALIZED_THRESHOLD:.3f})"
         if speed is not None
         else f"[이동] Normalized Speed: N/A (>= {NORMALIZED_THRESHOLD:.3f})"),
        f"Frame: {frame_number}  FPS: {fps:.2f}",
    ]
    if lost_count:
        lines.append(f"대상 미검출: {lost_count}/{TRACK_LOST_FRAMES}")
    panel_height = 12 + 27 * len(lines)
    overlay = frame_image.copy()
    cv2.rectangle(overlay, (0, 0), (min(frame_image.shape[1], 560), panel_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.69, frame_image, 0.31, 0, frame_image)
    image = Image.fromarray(cv2.cvtColor(frame_image, cv2.COLOR_BGR2RGB))
    drawer = ImageDraw.Draw(image)
    font = korean_font(19)
    for index, line in enumerate(lines):
        color = "#ff8585" if index == 0 and state in (STATE_EXIT, STATE_LOST) else "white"
        drawer.text((10, 7 + 27 * index), line, font=font, fill=color)
    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)


def main() -> None:
    validate_inputs()
    model = YOLO(str(MODEL_PATH))
    names = {int(k): str(v) for k, v in model.names.items()}
    print(f"모델 클래스: {names}", flush=True)
    animal_classes = {idx for idx, name in names.items() if name.lower() in {"boar", "deer"}}
    torso_classes = {idx for idx, name in names.items() if name.lower() == "torso"}
    if not animal_classes or not torso_classes:
        raise RuntimeError("모델에 boar/deer 및 torso 클래스가 모두 있어야 합니다. 임의 Torso BBox는 만들지 않습니다.")

    capture = cv2.VideoCapture(str(VIDEO_PATH))
    if not capture.isOpened():
        raise RuntimeError(f"영상을 열 수 없습니다: {VIDEO_PATH}")
    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if source_fps <= 0 or width <= 0 or height <= 0:
        capture.release()
        raise RuntimeError("입력 영상의 FPS 또는 해상도를 읽을 수 없습니다")
    if PROCESSING_FPS <= 0 or PROCESSING_FPS > source_fps:
        capture.release()
        raise ValueError(f"PROCESSING_FPS는 0보다 크고 원본 FPS({source_fps:.2f}) 이하여야 합니다")
    fps = PROCESSING_FPS

    run_dir = new_run_directory()
    tracker_settings = YAML.load(check_yaml(TRACKER))
    if tracker_settings.get("tracker_type") != "bytetrack":
        raise ValueError("TRACKER는 ByteTrack 설정이어야 합니다")
    tracker_settings["track_high_thresh"] = min(tracker_settings["track_high_thresh"], TORSO_CONFIDENCE)
    tracker_settings["new_track_thresh"] = min(tracker_settings["new_track_thresh"], TORSO_CONFIDENCE)
    tracker_path = run_dir / "bytetrack_confidence.yaml"
    YAML.save(str(tracker_path), tracker_settings)
    video_path = run_dir / OUTPUT_VIDEO.name
    csv_path = run_dir / OUTPUT_CSV.name
    events_path = run_dir / "termination_events.csv"
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"출력 영상을 열 수 없습니다: {video_path}")
    print(f"입력: {VIDEO_PATH} / 원본 {source_fps:.2f} FPS / 처리·출력 {fps:.2f} FPS / {width}x{height}", flush=True)
    print(f"결과 폴더: {run_dir}", flush=True)

    target_id: int | None = None
    target_class: str | None = None
    lost_count = 0
    area_monitor = AreaMonitor()
    history: deque[tuple[int, float, float, float]] = deque(maxlen=WINDOW_SIZE)
    excluded: dict[int, int] = {}  # Terminated ID -> consecutive frames absent.
    exit_through_frame = 0
    frame_number = 0
    source_frame_index = -1
    area_event_count = 0
    rapid_event_count = 0

    try:
        with csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file, \
             events_path.open("w", newline="", encoding="utf-8-sig") as events_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            csv_writer.writeheader()
            event_writer = csv.DictWriter(events_file, fieldnames=EVENT_FIELDS)
            event_writer.writeheader()
            while True:
                ok, frame_image = capture.read()
                if not ok:
                    break
                source_frame_index += 1
                # Sample real source frames at PROCESSING_FPS.
                if source_frame_index != round(frame_number * source_fps / fps):
                    continue
                frame_number += 1
                timestamp = source_frame_index / source_fps
                result = model.track(frame_image, persist=True, tracker=str(tracker_path),
                                     conf=CONFIDENCE, verbose=False)[0]
                detections = parse_detections(result, names)
                animals = [d for d in detections if d.cls_name in {"boar", "deer"}
                           and d.confidence >= ANIMAL_CONFIDENCE]
                torsos = [d for d in detections if d.cls_name == "torso"
                          and d.confidence >= TORSO_CONFIDENCE]
                matches = torso_matches(animals, torsos)
                visible_ids = {d.track_id for d in animals if d.track_id is not None}
                for banned_id in list(excluded):
                    excluded[banned_id] = 0 if banned_id in visible_ids else excluded[banned_id] + 1
                    if excluded[banned_id] >= TRACK_LOST_FRAMES:
                        del excluded[banned_id]

                selected = next((d for d in animals if d.track_id == target_id), None) if target_id is not None else None
                state = STATE_SEARCH
                reason = ""
                speed: float | None = None
                matched_torso: Detection | None = None
                row_id = target_id
                row_class = target_class
                terminated = False
                audio_active = False

                if target_id is None and frame_number > exit_through_frame:
                    eligible = [d for d in animals if d.track_id is not None and d.track_id not in excluded]
                    if eligible:
                        selected = max(eligible, key=lambda d: d.area)
                        target_id = selected.track_id
                        target_class = selected.cls_name
                        row_id, row_class = target_id, target_class
                        lost_count = 0
                        area_monitor = AreaMonitor()
                        history.clear()

                if target_id is not None:
                    # Condition 1 is evaluated whether or not the torso is matched.
                    area_cancel = area_monitor.update(timestamp, selected, width, height)
                    rapid_movement = False
                    if selected is None:
                        lost_count += 1
                        history.clear()
                        if lost_count >= TRACK_LOST_FRAMES:
                            state = STATE_LOST
                            reason = "tracking_lost"
                            terminated = True
                            target_id = None
                            target_class = None
                            lost_count = 0
                    else:
                        lost_count = 0
                        ai = next(i for i, animal in enumerate(animals) if animal is selected)
                        ti = matches.get(ai)
                        if ti is None:
                            history.clear()
                        else:
                            # Condition 2 needs the matched torso on consecutive frames.
                            matched_torso = torsos[ti]
                            if history and history[-1][0] != frame_number - 1:
                                history.clear()
                            cx, cy = matched_torso.center
                            history.append((frame_number, cx, cy, selected.width))
                            if len(history) == WINDOW_SIZE:
                                first, last = history[0], history[-1]
                                mean_width = sum(item[3] for item in history) / WINDOW_SIZE
                                if mean_width > 0:
                                    speed = hypot(last[1] - first[1], last[2] - first[2]) / ((WINDOW_SIZE - 1) * mean_width)
                                    rapid_movement = speed >= NORMALIZED_THRESHOLD
                        # Collect the initial area reference before beginning audio.
                        if area_monitor.baseline is None:
                            state = STATE_BASELINE
                        elif matched_torso is not None:
                            state = STATE_ACTIVE
                            audio_active = True
                        else:
                            state = STATE_TORSO_MISSING

                        reasons = [name for name, fired in ((AREA_REASON, area_cancel),
                                                            (RAPID_MOVEMENT_REASON, rapid_movement)) if fired]
                        if reasons:
                            state = STATE_EXIT
                            audio_active = False
                            reason = "+".join(reasons)
                            terminated = True
                            area_event_count += area_cancel
                            rapid_event_count += rapid_movement
                            excluded[target_id] = 0
                            exit_through_frame = frame_number + max(1, round(EXIT_MESSAGE_DURATION * fps)) - 1
                            event_writer.writerow({
                                "frame": frame_number, "timestamp": f"{timestamp:.6f}",
                                "animal_class": target_class, "animal_track_id": target_id,
                                "termination_reason": reason,
                                "baseline_area": f"{area_monitor.baseline:.3f}" if area_monitor.baseline is not None else "",
                                "smoothed_area": f"{area_monitor.smoothed:.3f}" if area_monitor.smoothed is not None else "",
                                "area_ratio": f"{area_monitor.ratio:.6f}" if area_monitor.ratio is not None else "",
                                "area_ratio_threshold": AREA_RATIO_THRESHOLD,
                                "normalized_speed": f"{speed:.6f}" if speed is not None else "",
                                "speed_threshold": f"{NORMALIZED_THRESHOLD:.6f}",
                                "event_meaning": " + ".join(EVENT_MEANINGS[name] for name in reasons)
                                                 + " (퇴치 성공 판정 아님)",
                            })
                            history.clear()
                            target_id = None
                            target_class = None
                            lost_count = 0
                else:
                    area_monitor = AreaMonitor()
                    history.clear()
                    if frame_number <= exit_through_frame:
                        state = STATE_EXIT

                row = {field: "" for field in CSV_FIELDS}
                row.update({
                    "frame": frame_number, "source_frame": source_frame_index + 1,
                    "timestamp": f"{timestamp:.6f}",
                    "animal_class": row_class or "", "animal_track_id": row_id if row_id is not None else "",
                    "animal_detected": selected is not None,
                    "torso_detected": bool(torsos),
                    "torso_matched": matched_torso is not None,
                    "audio_active": audio_active,
                    "torso_match_success": bool(matched_torso) if row_id is not None else "",
                    "animal_bbox_area": f"{selected.area:.3f}" if selected is not None else "",
                    "baseline_area": area_monitor.baseline if area_monitor.baseline is not None else "",
                    "smoothed_area": area_monitor.smoothed if area_monitor.smoothed is not None else "",
                    "area_ratio": area_monitor.ratio if area_monitor.ratio is not None else "",
                    "area_below_seconds": f"{area_monitor.below_seconds:.3f}",
                    "area_sample_valid": area_monitor.valid,
                    "area_ratio_threshold": AREA_RATIO_THRESHOLD,
                    "normalized_speed": f"{speed:.6f}" if speed is not None else "",
                    "speed_threshold": f"{NORMALIZED_THRESHOLD:.6f}",
                    "system_state": state,
                    "tracking_terminated": terminated, "termination_reason": reason,
                })
                if selected is not None:
                    row.update({
                        "animal_confidence": f"{selected.confidence:.6f}",
                        "animal_bbox_width": f"{selected.width:.3f}",
                        "animal_bbox_height": f"{selected.height:.3f}",
                    })
                if matched_torso is not None:
                    cx, cy = matched_torso.center
                    row.update({
                        "torso_confidence": f"{matched_torso.confidence:.6f}",
                        "torso_center_x": f"{cx:.3f}", "torso_center_y": f"{cy:.3f}",
                        "torso_bbox_width": f"{matched_torso.width:.3f}",
                        "torso_bbox_height": f"{matched_torso.height:.3f}",
                    })
                csv_writer.writerow(row)
                display_id = row_id if state in (STATE_EXIT, STATE_LOST) and row_id is not None else target_id
                display_class = row_class if state in (STATE_EXIT, STATE_LOST) and row_class else target_class
                annotated = render(frame_image, animals, torsos, matches,
                                   selected if target_id is not None else None,
                                   state, display_id, display_class, area_monitor, speed,
                                   frame_number, fps, lost_count)
                writer.write(annotated)
                if frame_number % 100 == 0:
                    print(f"처리: {frame_number}프레임 / 면적 감소 종료: {area_event_count}회"
                          f" / 급속 이동 종료: {rapid_event_count}회", flush=True)
    finally:
        capture.release()
        writer.release()
    print(f"완료: {frame_number}프레임 / 면적 감소 종료: {area_event_count}회"
          f" / 급속 이동 종료: {rapid_event_count}회", flush=True)
    print(f"MP4: {video_path}\nCSV: {csv_path}\n이벤트: {events_path}", flush=True)


if __name__ == "__main__":
    main()
