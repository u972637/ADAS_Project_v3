#!/usr/bin/env bash
# DM-J4310-2EC 액추에이터 테스트 — 메뉴에서 번호를 선택해 단계별로 실행한다.
#
# 1. CAN ID 찾기/할당 : CAN ID 1~N번을 스캔해 찾고, 원하면 그 자리에서 새 ID로 변경
# 2. CAN ID 설정     : 이미 아는 CAN ID를 직접 입력 (스캔 없이)
# 3. 신호 조회       : 토크 없이 상태만 반복 조회 — 통신 생존 확인
# 4. 위치 신호       : 짧게 활성화해 MIT 모드로 작게 왕복 — 실제 제어 확인
# 5. 위치 이동       : 지정한 각도로 이동한 뒤 그 자리를 계속 유지 (Ctrl+C로 종료)
# 6. 제로잉         : 토크 없이 손으로 0도 위치를 맞춘 뒤 영점 저장
# 7. 게인 설정       : 4/5번에 쓸 Kp/Kd/이동각/유지시간을 이 실행 동안만 바꿈
#
# 포트(시리얼 장치 경로)는 이 스크립트에서 따로 설정하지 않는다 —
# test/actuator_conn_test.py 가 motor/dm4310_tracker.py 의 DEVICENAME 상수를
# 그대로 기본값으로 쓰기 때문에, 실제 사용 중인 포트가 바뀌면
# motor/dm4310_tracker.py 의 DEVICENAME 한 곳만 고치면 이 스크립트와
# main.py 운영 코드 모두에 일괄 적용된다.
#
# 1번(또는 2번)으로 CAN ID 가 설정돼야만 3~6번을 실행할 수 있다.
# 7번에서 바꾼 값은 이 스크립트를 실행하는 동안만 유지되는 임시값이다 —
# test/actuator_conn_test.py 맨 위 DEFAULT_KP/DEFAULT_KD/... 상수 자체를
# 바꾸는 게 아니므로, 다음에 스크립트를 새로 실행하면 다시 그 파일의
# 기본값(8.0 / 0.6 / 0.1rad≈5.73° / 1.0)으로 돌아간다. "항상 이 값으로 시작하고
# 싶다"면 test/actuator_conn_test.py 상단의 DEFAULT_* 상수를 직접 고칠 것.
#
# 사용법: ./actuator_test.sh

# -e(실패 즉시 종료)는 안 쓴다 — 동글 미연결/잘못된 CAN ID 등으로 파이썬
# 호출이 실패해도(exit code 1) 그 선택 하나만 실패 처리하고 메뉴로 돌아와야
# 하는 대화형 루프라서, -e 를 쓰면 그런 흔한 실패 한 번에 스크립트 전체가
# 꺼져버린다.
set -uo pipefail
cd "$(dirname "$0")"

CAN_ID=""

# motor/dm4310_tracker.py 의 DEVICENAME 을 그대로 읽어와 메뉴에 표시만 한다
# (여기서 바꾸는 게 아니라 "지금 어떤 포트를 쓰고 있는지" 확인용).
CURRENT_DEVICE="$(uv run python3 -c "from motor.dm4310_tracker import DEVICENAME; print(DEVICENAME)" 2>/dev/null || echo "(확인 불가)")"

# 4번(위치 신호)에 쓸 게인/이동각/유지시간 — test/actuator_conn_test.py 의
# DEFAULT_KP/DEFAULT_KD/DEFAULT_DELTA_RAD(=60도)/DEFAULT_HOLD_SEC 와 같은
# 기본값으로 시작한다. 7번 메뉴에서 바꾸면 이 변수만 바뀌고(세션 한정),
# 파이썬 파일의 상수는 그대로다.
# 이동각은 사람이 보기 편하게 "도(degree)" 단위로 들고 있다가, 파이썬 호출
# 시 --delta-deg 로 그대로 넘긴다(내부에서 라디안으로 변환해 CAN에 실림).
KP="8.0"
KD="0.6"
DELTA_DEG="60"
HOLD="1.0"

print_menu() {
    echo
    echo "======================================"
    echo " DM-J4310-2EC 액추에이터 테스트"
    echo "======================================"
    echo " 포트   : $CURRENT_DEVICE  (motor/dm4310_tracker.py 의 DEVICENAME)"
    if [[ -n "$CAN_ID" ]]; then
        echo " CAN ID : $CAN_ID"
    else
        echo " CAN ID : (지정 안 됨 — 1번 또는 2번 필요)"
    fi
    echo " 게인   : Kp=$KP Kd=$KD / 이동각=${DELTA_DEG}° / 유지=${HOLD}s"
    echo "--------------------------------------"
    echo " 1. CAN ID 찾기/할당 (스캔)"
    echo " 2. CAN ID 설정 (직접 입력)"
    echo " 3. 신호 조회 (통신 생존 확인)"
    echo " 4. 위치 신호 (MIT 모드 테스트)"
    echo " 5. 위치 이동 (특정 각도로 이동 후 유지)"
    echo " 6. 제로잉 (영점 설정)"
    echo " 7. 게인 설정 (Kp/Kd/이동각/유지시간, 이번 실행만 적용)"
    echo " 0. 종료"
    echo "======================================"
}

require_ready() {
    if [[ -z "$CAN_ID" ]]; then
        echo "[!] 먼저 1번(CAN ID 찾기/할당) 또는 2번(CAN ID 설정)을 완료하세요."
        return 1
    fi
    return 0
}

while true; do
    print_menu
    read -rp "선택 > " choice

    case "$choice" in
        1)
            # CAN ID를 몰라도 된다 — 1~N번을 스캔해서 찾고, 원하면 새 ID로 변경.
            # 최종적으로 응답하는 ID를 파이썬 스크립트가 tmpfile에 적어주면
            # 그 값을 그대로 CAN_ID로 받아온다.
            tmpfile="$(mktemp)"
            uv run python test/actuator_conn_test.py --step id --write-id-to "$tmpfile"
            if [[ -s "$tmpfile" ]]; then
                CAN_ID="$(cat "$tmpfile")"
                echo
                echo "[OK] CAN ID = $CAN_ID 로 지정됨"
            else
                echo
                echo "[!] ID를 확인하지 못했습니다 — CAN_ID가 지정되지 않았습니다."
            fi
            rm -f "$tmpfile"
            ;;
        2)
            read -rp "CAN ID 입력 (예: 0x01=Yaw, 0x02=Pitch): " input_id
            if [[ -z "$input_id" ]]; then
                echo "[!] 입력이 없습니다."
                continue
            fi
            CAN_ID="$input_id"
            echo "[OK] CAN ID = $CAN_ID 로 지정됨"
            ;;
        3)
            require_ready || continue
            uv run python test/actuator_conn_test.py --id "$CAN_ID" --step ping
            ;;
        4)
            require_ready || continue
            uv run python test/actuator_conn_test.py --id "$CAN_ID" --step mit \
                --kp "$KP" --kd "$KD" --delta-deg "$DELTA_DEG" --hold "$HOLD"
            ;;
        5)
            require_ready || continue
            read -rp "목표 각도(도) 입력 (예: 10, -5): " input_target
            if [[ -z "$input_target" ]]; then
                echo "[!] 입력이 없습니다."
                continue
            fi
            echo "[안내] 목표 위치에 도달한 뒤 계속 유지합니다 — 멈추려면 Ctrl+C"
            uv run python test/actuator_conn_test.py --id "$CAN_ID" --step goto \
                --target-deg "$input_target" --kp "$KP" --kd "$KD" --hold "$HOLD"
            ;;
        6)
            require_ready || continue
            uv run python test/actuator_conn_test.py --id "$CAN_ID" --step zero
            ;;
        7)
            read -rp "Kp 입력 [현재: $KP]: " input_kp
            [[ -n "$input_kp" ]] && KP="$input_kp"
            read -rp "Kd 입력 [현재: $KD]: " input_kd
            [[ -n "$input_kd" ]] && KD="$input_kd"
            read -rp "이동각(도) 입력 [현재: $DELTA_DEG]: " input_delta
            [[ -n "$input_delta" ]] && DELTA_DEG="$input_delta"
            read -rp "유지시간 초 입력 [현재: $HOLD]: " input_hold
            [[ -n "$input_hold" ]] && HOLD="$input_hold"
            echo "[OK] 게인 = Kp=$KP Kd=$KD / 이동각=${DELTA_DEG}° / 유지=${HOLD}s (이번 실행에만 적용)"
            ;;
        0)
            echo "종료합니다."
            break
            ;;
        *)
            echo "[!] 0~7 중에서 선택하세요."
            ;;
    esac
done
