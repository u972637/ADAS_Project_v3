#!/usr/bin/env bash
# Yaw/Pitch 모터 + 카메라 통합 테스트 메뉴 (tools/motor_manual_control.py 실행).
#
# 메뉴:
#   1. 동물 추적 + 종별 음향 재생 (멧돼지/고라니 탐지 시 모터 조준 + WAV 재생)
#   2. YOLO 코 좌표 추적 (손을 ARM_UP_THRESHOLD(2초) 이상 들어야 모터 추적
#      시작, 추적이 확정되는 순간 audio/track_start2.mp3 안내음 재생)
#   3. 방향키로 위치 제어 (모터 각도 직접 조그)
#   0. 종료
#
# 1·2·3번 모두 q/ESC 로 정상 종료하면 "숙인위치"(yaw=0°, pitch=-4°)로
# 1초에 걸쳐 복귀한 뒤 메뉴로 돌아간다 (Ctrl+C 강제중지 시엔 복귀 없이 바로 종료).
#
# 사용법: ./camera_moter_test.sh

set -uo pipefail
cd "$(dirname "$0")"

uv run python tools/motor_manual_control.py
