"""사건 로그: 매 프레임이 아니라 추적 상태가 바뀔 때만 한 줄씩 남긴다.

runtime 루프가 반복마다 observe(service, now) 를 부르면, 직전 호출과 달라진
것만 로그로 나간다 (같은 프레임을 여러 번 봐도 중복되지 않는다).
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

# 락이 있을 때의 추적 방식
TORSO, BBOX, ORANGE = "torso", "bbox", "orange"
_MODE_TEXT = {TORSO: "torso 중심(빨강)", BBOX: "bbox 중심(빨강)", ORANGE: "주황점"}
_END_TEXT = {
    "tracking_lost": "도망 판정: 주황 상태가 제한 시간 넘게 지속",
    "bbox_area_reduced": "도망 판정: bbox 면적·높이 감소",
    "bbox_too_small": "도망 판정: bbox 높이가 최소값 미만",
    "camera_timeout": "카메라 프레임 끊김",
    "perception_timeout": "추론 결과 끊김",
}


class EventLogger:
    def __init__(self, emit=logger.info):
        self._emit = emit
        self._mode = None            # None = 락 없음
        self._target_id = None
        self._orange_since = None
        self._orange_frames_from = None
        self._playing = False
        self._phase = None
        self._error = None
        self._last_step_id = None

    @staticmethod
    def _mode_of(service):
        step = service.last_escape_step
        if service.escape_tracker.target_id is None or step is None or step.terminated:
            return None
        if step.matched_torso is not None:
            return TORSO
        if step.followed_bbox is not None:
            return BBOX
        return ORANGE

    def observe(self, service, now: float) -> None:
        phase = service.phase.value
        if phase != self._phase:
            if self._phase is not None:
                self._emit(f"[상태] {self._phase} -> {phase}")
            self._phase = phase
        error = service.last_error
        if error and error != self._error:
            self._emit(f"[오류] {error}")
        self._error = error

        # 같은 프레임 결과를 반복 처리하지 않는다 (스텝 객체가 같으면 건너뜀).
        step = service.last_escape_step
        if step is not None and id(step) == self._last_step_id:
            self._observe_playing(service)
            return
        self._last_step_id = id(step) if step is not None else None

        mode = self._mode_of(service)
        tracker = service.escape_tracker
        if mode != self._mode:
            self._transition(service, self._mode, mode, now)
            self._mode = mode
        elif mode is not None and tracker.target_id != self._target_id:
            self._emit(f"[대상 변경] bbox id {self._target_id} -> {tracker.target_id}")
        self._target_id = tracker.target_id if mode is not None else None
        self._observe_playing(service)

    def _observe_playing(self, service) -> None:
        playing = service.playing
        if playing != self._playing:
            label = service.class_map.get(service.escape_tracker.target_class)
            self._emit(
                f"[경계음] 재생 시작 ({label})" if playing else "[경계음] 재생 종료"
            )
            self._playing = playing

    def _transition(self, service, old, new, now) -> None:
        tracker = service.escape_tracker
        step = service.last_escape_step
        if old is None and new is not None:
            target = step.selected
            label = service.class_map.get(tracker.target_class, "?")
            conf = f"{target.confidence:.2f}" if target is not None else "-"
            torso = step.matched_torso
            where = (
                f" torso=({torso.center[0]:.0f},{torso.center[1]:.0f})"
                if torso is not None
                else ""
            )
            self._emit(
                f"[추적 시작] {label} id={tracker.target_id} 신뢰도={conf}{where}"
            )
        elif new is None:
            reason = service.last_event.reason
            if self._mode == ORANGE:
                gone = f", 주황 {now - self._orange_since:.1f}초 지속"
            else:
                gone = ""
            text = _END_TEXT.get(reason)
            if text is None and reason.startswith(("stopped", "camera_", "perception_")):
                text = "정지/타임아웃으로 추적 해제"
            if text is None:
                # 다른 이유(정지·재개 등)는 [상태] 줄이 따로 말해 준다.
                parts = [p for p in reason.split("+") if p in _END_TEXT]
                text = " + ".join(_END_TEXT[p] for p in parts) or f"추적 종료 ({reason})"
            self._emit(f"[추적 종료] {text}{gone}")
        elif new == ORANGE:
            self._orange_since = now
            self._emit(
                f"[추적 실패(주황)] {_MODE_TEXT[old]} 을(를) 잃음 — "
                "주황점으로 모터 이동 시작"
            )
        elif old == ORANGE:
            what = "torso" if new == TORSO else "bbox"
            self._emit(
                f"[주황 해제] {what} 이(가) 먼저 다시 나타나 빨간점 복귀 "
                f"(주황 {now - self._orange_since:.1f}초, id={tracker.target_id})"
            )
        elif old == TORSO and new == BBOX:
            self._emit("[추적 전환] torso 유실 -> bbox 중심 추적")
        elif old == BBOX and new == TORSO:
            self._emit("[추적 전환] bbox 안에서 torso 다시 매칭 -> torso 추적")
