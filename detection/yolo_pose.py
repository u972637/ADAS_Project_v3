#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ultralytics YOLOv8 pose 기반 PoseDetector 구현.

나중에 파인튜닝한 커스텀 YOLO pose 모델로 교체할 때는 model_path 인자만
바꿔서 새 YoloPoseDetector(model_path="my_finetuned.pt") 를 만들면 된다.
COCO 17-keypoint 포맷(nose=0 ... 어깨/손목 인덱스 동일)을 유지하는 모델이면
이 클래스를 그대로 재사용할 수 있다.

만약 키포인트 레이아웃이 다른 모델(다른 관절 수/순서)을 쓴다면 이 파일을
참고해 PoseDetector 를 새로 구현하면 되고, main.py 등 상위 코드는
detection.base.PersonPose 형태만 맞으면 변경할 필요가 없다.
"""

from typing import List, Optional

import torch
from ultralytics import YOLO

from detection.base import PersonPose, PoseDetector

# YOLOv8 pose keypoint 인덱스 (COCO 기준)
KP_NOSE           = 0
KP_LEFT_EYE       = 1
KP_RIGHT_EYE      = 2
KP_LEFT_EAR       = 3
KP_RIGHT_EAR      = 4
KP_LEFT_SHOULDER  = 5
KP_RIGHT_SHOULDER = 6
KP_LEFT_WRIST     = 9
KP_RIGHT_WRIST    = 10

FACE_BOX_PAD = 30


def _is_arm_up(kp) -> bool:
    """keypoints 배열(N×2)에서 손목이 어깨보다 위에 있으면 True."""
    if len(kp) <= KP_RIGHT_WRIST:
        return False
    left_up  = kp[KP_LEFT_WRIST][1]  < kp[KP_LEFT_SHOULDER][1]
    right_up = kp[KP_RIGHT_WRIST][1] < kp[KP_RIGHT_SHOULDER][1]
    return left_up or right_up


def _estimate_face_box(kp, pad=FACE_BOX_PAD):
    """코·눈·귀 keypoint(0~4)로 얼굴 영역 박스를 추정."""
    head_kp = kp[:5]
    xs = [p[0] for p in head_kp if p[0] > 0]
    ys = [p[1] for p in head_kp if p[1] > 0]
    if not xs or not ys:
        return None
    return (int(min(xs) - pad), int(min(ys) - pad),
            int(max(xs) + pad), int(max(ys) + pad))


def _select_device(requested: Optional[str]) -> str:
    if requested:
        return requested
    return "cuda:0" if torch.cuda.is_available() else "cpu"


class YoloPoseDetector(PoseDetector):
    def __init__(self, model_path: str, device: Optional[str] = None):
        self._device = _select_device(device)
        self._model = YOLO(model_path)

    @property
    def device(self) -> str:
        return self._device

    def detect(self, frame) -> List[PersonPose]:
        results = self._model(frame, verbose=False, device=self._device)
        people: List[PersonPose] = []

        for r in results:
            if r.keypoints is None or r.boxes is None:
                continue

            kp_all = r.keypoints.xy   # (N, 17, 2)
            boxes  = r.boxes.xyxy     # (N, 4)

            for i in range(len(kp_all)):
                kp = kp_all[i].cpu().numpy()
                x1, y1, x2, y2 = boxes[i].cpu().numpy()

                nose = None
                if len(kp) > KP_NOSE and kp[KP_NOSE][0] > 0 and kp[KP_NOSE][1] > 0:
                    nose = (int(kp[KP_NOSE][0]), int(kp[KP_NOSE][1]))

                people.append(PersonPose(
                    track_index=i,
                    nose=nose,
                    arm_up=_is_arm_up(kp),
                    box=(float(x1), float(y1), float(x2), float(y2)),
                    face_box=_estimate_face_box(kp),
                ))

        return people
