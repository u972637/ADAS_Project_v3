#!/usr/bin/env bash
# 메뉴 1(동물 모델 통합 실험)과 같은 동작(`main.py --start-paused`)을 실행하면서
# 터미널에 나오는 모든 출력(logging 포함)을 log/menu1_<시각>.log 에 남긴다.
# 실행 전에 카메라/모터 포트 상태를, 끝난 뒤에 종료 코드와 커널 USB 로그를
# 같이 기록하므로 "안 열린다" 같은 문제가 생기면 이 로그 파일 하나만 보면 된다.
#
#   tools/menu1_logged.sh                      # 메뉴 1 그대로
#   tools/menu1_logged.sh --no-motor --headless   # 추가 옵션은 main.py 로 전달
#
# 키 입력(R 정지 / W 재개 / Q 종료, 터미널 stop/resume/quit)은 평소와 같다.
set -uo pipefail
cd "$(dirname "$0")/.."

mkdir -p log
STAMP=$(date +%Y%m%d_%H%M%S)
START_AT=$(date '+%Y-%m-%d %H:%M:%S')
LOG="log/menu1_${STAMP}.log"

{
    echo "===== 메뉴 1 실행 로그 ${STAMP} ====="
    echo "[git] $(git rev-parse --abbrev-ref HEAD 2>/dev/null) $(git rev-parse --short HEAD 2>/dev/null) (미커밋 파일 $(git status --short 2>/dev/null | wc -l)개)"
    echo "[명령] uv run python -u main.py --start-paused $*"
    echo
    echo "--- 카메라 ---"
    ls -l /dev/v4l/by-id/ 2>&1 | sed 's/^.* \(usb-\)/  \1/'
    found=""
    for d in /sys/bus/usb/devices/*; do
        [ -f "$d/product" ] || continue
        grep -qi arducam "$d/product" || continue
        found=1
        speed=$(cat "$d/speed")
        echo "  Arducam: $(cat "$d/product")  speed=${speed}Mbps  ($(basename "$d"))"
        if [ "$speed" != "5000" ] && [ "$speed" != "10000" ]; then
            echo "  [경고] USB 3(5000Mbps)이 아닙니다 -> 2592x1944 영상이 약 2fps로 느려지고 추적이 끊깁니다. USB 3 포트에 다시 꽂으세요."
        fi
    done
    [ -z "$found" ] && echo "  [경고] Arducam 이 USB 에서 보이지 않습니다."
    echo
    echo "--- 모터(USB-CAN) ---"
    ls -l /dev/ttyACM* 2>&1 | sed 's/^/  /'
    echo
    echo "--- 카메라/모터 포트를 이미 잡고 있는 프로세스 ---"
    holders=""
    for p in $(pgrep -u "$USER" -f python); do
        open=$(ls -l /proc/"$p"/fd 2>/dev/null | grep -E "video|ttyACM" | sed 's/^.*-> /      -> /')
        if [ -n "$open" ]; then
            holders=1
            echo "  pid $p: $(ps -o cmd= -p "$p" | cut -c1-80)"
            echo "$open"
        fi
    done
    [ -z "$holders" ] && echo "  없음"
    echo
    echo "--- 실행 ($(date +%T)) ---"
} 2>&1 | tee "$LOG"

uv run python -u main.py --start-paused "$@" 2>&1 | tee -a "$LOG"
code=${PIPESTATUS[0]}

{
    echo
    echo "--- 종료 ($(date +%T)) exit=${code} ---"
    echo "--- 실행 중 커널 USB/UVC/시리얼 로그 ---"
    journalctl -k --since "$START_AT" --no-pager 2>/dev/null \
        | grep -iE "uvc|usb|xhci|ttyACM|cdc_acm" | tail -20 \
        | sed 's/^/  /'
    echo "(로그 파일: ${LOG})"
} 2>&1 | tee -a "$LOG"

exit "$code"
