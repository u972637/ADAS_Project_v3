#!/usr/bin/env bash
# Yaw/Pitch 수동 제어 + 카메라 위 가상 코 좌표 테스트 메뉴 (test/motor_manual_control.py 실행).
#
# 메뉴:
#   1. START (아직 구현되지 않음 — 추후 main.py 자동 추적(Tracker)과 연결 예정)
#   2. 방향키로 위치 제어 (모터 각도 직접 조그)
#   3. 가상 코 좌표 테스트 (실제 카메라 화면 위에서 wasd로 노란 점 이동 → 실제 Tracker)
#   0. 종료
#
# 사용법: ./camera_moter_test.sh

set -uo pipefail
cd "$(dirname "$0")"

uv run python test/motor_manual_control.py
