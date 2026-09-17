#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
사람 검출기 추상 인터페이스.

YOLOv8 pose를 파인튜닝한 모델이나 완전히 다른 검출기로 교체되어도
main.py 등 상위 추적 로직은 이 인터페이스(PersonPose 목록)만 맞으면
코드를 바꾸지 않고 그대로 동작한다.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class PersonPose:
    """한 사람에 대한 검출 결과 — 상위 추적 로직이 필요로 하는 최소 정보만 담는다."""

    track_index: int                          # 이번 프레임 내 순번 (프레임 간 지속되는 id 아님)
    nose: Optional[Tuple[int, int]]            # 코 픽셀 좌표 (x, y), 검출 못하면 None
    arm_up: bool                               # 손목이 어깨보다 위에 있는지
    box: Tuple[float, float, float, float]     # 사람 박스 (x1, y1, x2, y2)
    face_box: Optional[Tuple[int, int, int, int]]  # 얼굴 영역 박스, 추정 불가면 None


class PoseDetector(ABC):
    """프레임 한 장을 넣으면 사람별 PersonPose 목록을 반환하는 검출기 인터페이스."""

    @abstractmethod
    def detect(self, frame) -> List[PersonPose]:
        """BGR 프레임(np.ndarray)을 받아 검출된 사람 목록을 반환."""
        raise NotImplementedError

    @property
    @abstractmethod
    def device(self) -> str:
        """추론에 사용 중인 디바이스 문자열 (예: 'cuda:0', 'cpu')."""
        raise NotImplementedError
