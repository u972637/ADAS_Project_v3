"""Feed a YOLO pose model's people into the wildlife pipeline as if animals.

start.sh 2번(사람 대역 추적) 전용. 동물 모델은 동물 박스와 함께 torso 클래스
박스를 내주지만 COCO YOLO 에는 torso 클래스가 없으므로, 어깨·골반 키포인트로
torso 박스를 만들어 붙인다. 그 뒤로는 WildlifeDeterrent.process_detections()
— 즉 1번과 같은 EscapeTracker·조준·재생 정책 — 를 그대로 탄다.
"""

from __future__ import annotations

import math

from animal import settings
from animal.wildlife import (
    Detection, Event, WildlifeDeterrent, _plain, read_detections,
)

# 모델이 내는 클래스가 아니라 키포인트로 만든 합성 클래스 — 실제 모델 클래스
# ID 와 절대 겹치지 않도록 음수를 쓴다.
TORSO_CLASS_ID = -1

# COCO 17-keypoint: 왼/오른 어깨, 왼/오른 골반
TORSO_KEYPOINTS = (5, 6, 11, 12)


def torso_from_keypoints(points: list, scores: list,
                         frame_size: tuple[int, int]) -> Detection | None:
    """어깨·골반 4점을 감싸는 박스. 신뢰도는 4점 중 최저값 — 실제 채택 여부는
    WildlifeDeterrent.torso_confidence 로 동물 torso 와 같은 자리에서 거른다."""
    if len(points) <= max(TORSO_KEYPOINTS) or len(scores) <= max(TORSO_KEYPOINTS):
        return None
    width, height = frame_size
    xs = [float(points[i][0]) for i in TORSO_KEYPOINTS]
    ys = [float(points[i][1]) for i in TORSO_KEYPOINTS]
    confidence = min(float(scores[i]) for i in TORSO_KEYPOINTS)
    if not all(math.isfinite(v) for v in (*xs, *ys, confidence)):
        return None
    x1, x2 = max(0.0, min(xs)), min(float(width), max(xs))
    y1, y2 = max(0.0, min(ys)), min(float(height), max(ys))
    if x2 <= x1 or y2 <= y1:
        return None
    return Detection(TORSO_CLASS_ID, max(0.0, min(1.0, confidence)), (x1, y1, x2, y2))


def read_person_detections(result: object) -> tuple[tuple[int, int], tuple[Detection, ...]]:
    """사람 박스(read_detections 그대로, track_id 포함) + 키포인트 torso 박스."""
    frame_size, people = read_detections(result)
    keypoints = getattr(result, "keypoints", None)
    if not people or keypoints is None or keypoints.conf is None:
        return frame_size, people
    all_points, all_scores = _plain(keypoints.xy), _plain(keypoints.conf)
    # model.track() 이 박스를 track 순서로 재정렬할 때 keypoints 도 같은
    # 인덱스로 함께 재정렬되므로 사람 박스와 1:1 로 대응한다.
    if not (len(all_points) == len(all_scores) == len(people)):
        raise ValueError("pose keypoints do not match person boxes")
    torsos = [torso for points, scores in zip(all_points, all_scores, strict=True)
              if (torso := torso_from_keypoints(points, scores, frame_size)) is not None]
    return frame_size, (*people, *torsos)


def person_deterrent_from_settings(**kwargs) -> WildlifeDeterrent:
    return WildlifeDeterrent.from_settings(
        class_map=settings.PERSON_CLASS_TO_AUDIO,
        expected_names=settings.PERSON_EXPECTED_MODEL_NAMES,
        torso_class_id=TORSO_CLASS_ID,
        torso_confidence=settings.PERSON_KEYPOINT_CONFIDENCE,
        **kwargs,
    )


def process_person_frame(model: object, service: WildlifeDeterrent,
                         frame: object, captured_at: float) -> Event:
    # animal/wildlife.py 의 process_frame() 과 같은 model.track(persist=True)
    # — EscapeTracker 가 그 track ID 로 사람 한 명을 lock 한다. torso 는 박스
    # 탐지가 아니라 키포인트에서 나오므로 conf 는 사람 박스 기준만 건다.
    results = model.track(frame, persist=True, tracker=service.tracker_config,
                          conf=service.min_selection_confidence, verbose=False)
    if len(results) != 1:
        raise ValueError("person model must return one result per frame")
    frame_size, detections = read_person_detections(results[0])
    return service.process_detections(frame_size, detections, captured_at)
