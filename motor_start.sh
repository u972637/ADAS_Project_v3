#!/usr/bin/env bash
# 지정한 픽셀 좌표 (x, y)로 Yaw/Pitch 짐벌을 이동시키고 그 자리를 유지한다.
# main.py(카메라+YOLO) 없이, 좌표만 직접 줘서 실제 운영 Tracker를 단독으로
# 구동해볼 때 쓴다. 내부적으로 test/motor_goto.py 를 실행한다.
#
# 사용법:
#   ./motor_start.sh 1200 900   # 그 좌표로 이동 후 유지 (Ctrl+C로 종료)
#   ./motor_start.sh            # 대화형으로 좌표 입력받음
#
# 좌표계: motor/dm4310_tracker.py 의 FRAME_W/FRAME_H(기본 2592x1944,
# 카메라 해상도) 기준 — 좌측 상단이 (0,0).

set -uo pipefail
cd "$(dirname "$0")"

uv run python test/motor_goto.py "$@"
