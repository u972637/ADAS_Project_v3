"""Feed a YOLO pose model's people into the wildlife pipeline as if animals.

start.sh 2번(사람 대역 추적) 전용. 동물 모델은 동물 박스와 함께 torso 클래스
박스를 내주지만 COCO YOLO 에는 torso 클래스가 없으므로, 어깨·골반 키포인트로
torso 박스를 만들어 붙인다. 그 뒤로는 WildlifeDeterrent.process_detections()
— 즉 1번과 같은 EscapeTracker·조준·재생 정책 — 를 그대로 탄다.
"""

from __future__ import annotations

import math

from animal import settings
from animal.wildlife import WildlifeDeterrent
from animal.perception import Detection, _plain, read_detections

PERSON_MODEL_PATH = settings.PROJECT_ROOT / "assets/models/yolov8n-pose.pt"
PERSON_CLASS_TO_AUDIO = {0: "boar"}
PERSON_EXPECTED_MODEL_NAMES = {0: "person"}
PERSON_KEYPOINT_CONFIDENCE = 0.5

# 모델이 내는 클래스가 아니라 키포인트로 만든 합성 클래스 — 실제 모델 클래스
# ID 와 절대 겹치지 않도록 음수를 쓴다.
TORSO_CLASS_ID = -1

# COCO 17-keypoint: 왼/오른 어깨, 왼/오른 골반
TORSO_KEYPOINTS = (5, 6, 11, 12)


def torso_from_keypoints(
    points: list, scores: list, frame_size: tuple[int, int]
) -> Detection | None:
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


def read_person_detections(
    result: object,
) -> tuple[tuple[int, int], tuple[Detection, ...]]:
    """Raw pose boxes and keypoints retain their original shared indices."""
    frame_size, people = read_detections(result)
    keypoints = getattr(result, "keypoints", None)
    if not people or keypoints is None or keypoints.conf is None:
        return frame_size, people
    all_points, all_scores = _plain(keypoints.xy), _plain(keypoints.conf)
    # Use raw predict() results before animal identity tracking reorders boxes.
    if not (len(all_points) == len(all_scores) == len(people)):
        raise ValueError("pose keypoints do not match person boxes")
    torsos = [
        torso
        for points, scores in zip(all_points, all_scores, strict=True)
        if (torso := torso_from_keypoints(points, scores, frame_size)) is not None
    ]
    return frame_size, (*people, *torsos)


def person_deterrent_from_settings(**kwargs) -> WildlifeDeterrent:
    return WildlifeDeterrent.from_settings(
        class_map=PERSON_CLASS_TO_AUDIO,
        expected_names=PERSON_EXPECTED_MODEL_NAMES,
        torso_class_id=TORSO_CLASS_ID,
        torso_confidence=PERSON_KEYPOINT_CONFIDENCE,
        **kwargs,
    )


def infer_person_detections(model, service, frame):
    from animal.inference import infer_detections

    return infer_detections(model, service, frame, torso_reader=read_person_detections)


def run_person_test():
    from animal.runtime import run_session

    return run_session(
        model_path=PERSON_MODEL_PATH,
        service_factory=person_deterrent_from_settings,
        inference=infer_person_detections,
        enable_motor=True,
        start_paused=True,
    )
