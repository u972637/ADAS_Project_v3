#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DM-J4310-2EC 액추에이터 연결 테스트 — 신규/단독 모터 1개를 Yaw/Pitch 역할에
배선하기 전에 CAN 통신·MIT 모드 제어·영점 설정이 되는지 확인하는 용도.

../actuator_test.sh 메뉴 기준으로 4단계로 구성된다 (1번부터 CAN ID를 몰라도
된다 — 스캔으로 찾기 때문).

  1) 포트 지정(ID 스캔) : CAN ID 1~N번을 순서대로 찔러봐서(스캔) 응답하는
                 ID를 찾고, 원하면 그 자리에서 새 ID로 변경 후 플래시에 저장
  2) 신호 조회   : 토크를 걸지 않은 채로 상태 조회만 반복 — 응답이 오는지,
                 축을 손으로 돌렸을 때 읽히는 값이 바뀌는지로 통신 생존을 확인
  3) 위치 신호(MIT 모드 테스트): 짧게 활성화해서 작은 각도만 안전하게
                 왕복시켜 실제 위치 제어(Kp/Kd 임피던스)가 동작하는지 확인
  4) 제로잉     : 토크 없이 손으로 원하는 0도 위치로 맞춘 뒤 모터 내부
                 비휘발성 메모리에 영점으로 저장
  5) 위치 이동(goto): 지정한 각도로 이동한 뒤 그 자리를 계속 유지
                 (왕복하지 않음 — Ctrl+C 로 멈출 때까지 유지)

3단계는 실제로 축이 움직이므로 진행 전 Enter 확인을 받는다 — 손가락 끼임 등
안전사고가 없는지 확인한 뒤 진행할 것.

--step 으로 한 단계만 실행할 수 있다 (actuator_test.sh 가 이 방식으로 호출함).
생략하면 1→2→3 을 이어서 한 번에 실행하는 기존 방식대로 동작한다.
"id" 단계는 CAN ID를 아직 모른다는 전제이므로 --id 없이 실행한다.

실행 예시:
  # 메뉴 없이 전체(연결→신호조회→MIT테스트)를 바로 실행
  uv run python tools/actuator_conn_test.py --id 0x01

  # 신호 조회만
  uv run python tools/actuator_conn_test.py --id 0x01 --step ping

  # 영점 설정만
  uv run python tools/actuator_conn_test.py --id 0x01 --step zero

  # CAN ID를 모를 때: 1~10번 스캔 후 원하는 ID로 변경
  uv run python tools/actuator_conn_test.py --step id

  # 특정 각도(도)로 이동해서 그 자리 유지 (Ctrl+C로 종료)
  uv run python tools/actuator_conn_test.py --id 0x01 --step goto --target-deg 15
"""

import argparse
import math
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import serial

from motor.DM_CAN import Motor, MotorControl, DM_variable, Control_Type
from motor.dm4310_tracker import DEVICENAME, BAUDRATE, MOTOR_TYPE

# ============================================================
# 사용자 설정값(기본값) — 필요하면 아래 값들을 직접 수정해도 된다.
#
# 여기 값들은 "CLI 인자(--kp, --kd, --delta, --hold 등)를 안 줬을 때" 쓰이는
# 기본값이다. actuator_test.sh 메뉴의 "게인 설정"에서 바꾼 값은 그 실행에만
# 적용되는 임시값(세션 한정)이며, 이 상수들은 바꾸지 않는다 — 즉 메뉴에서
# 설정 → 다음에 또 메뉴 없이 CLI로 직접 실행하면 다시 이 기본값으로
# 돌아간다. "항상 이 값으로 시작하고 싶다"면 아래 상수 자체를 고쳐야 한다.
# ============================================================

# MIT 모드 테스트용 기본 게인(torque = Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff).
# motor/dm4310_tracker.py 의 실제 운용값과 동일한 안전한 저값으로 맞춰둠.
#   Kp: 위치 오차에 대한 강성(스프링) — DM4310 범위 0~500. 너무 높이면
#       목표 도달 시 튕기거나 진동할 수 있음. 낮은 값부터 올릴 것.
DEFAULT_KP = 500.0
#   Kd: 속도 오차에 대한 감쇠(댐퍼) — DM4310 범위 0~5. 진동이 보이면 올릴 것.
DEFAULT_KD = 5.0
#   tau_ff: 피드포워드 토크(N·m). 중력보상 등에 쓰는 값이라 이 테스트에선 0 고정.
DEFAULT_TAU_FF = 0.0

# 기본 테스트 이동각(rad). 60도 ≈ 1.0472rad — 꽤 크게 움직이는 값이므로
# "위치 신호" 테스트 실행 전 Enter 확인 단계에서 축 주변 끼임 위험을
# 반드시 다시 확인할 것. 더 작게/크게 쓰려면 --delta 또는 --delta-deg로
# 그때그때 덮어쓸 수 있다.
DEFAULT_DELTA_RAD = math.radians(60.0)

# 목표까지 램프 이동 + 목표에서 정지 유지하는 시간(초). 너무 짧으면 급격히
# 움직이고, 너무 길면 테스트가 느려짐.
DEFAULT_HOLD_SEC = 1.0

RAMP_FREQ = 200          # 왕복 램프 스트리밍 주파수(Hz) — 커질수록 더 부드럽게 보간
PING_COUNT = 5           # 신호 테스트(2단계) 반복 횟수
PING_INTERVAL = 0.3      # 신호 테스트 간격(초)

DEFAULT_SCAN_MAX = 10    # ID 스캔(1단계) 시 1~이 값까지 탐색
SCAN_RETRY_INTERVAL = 0.05  # read_motor_param() 내부 재시도 간격(초) — 후보 1개당 대략 20회 재시도


def parse_can_id(text: str) -> int:
    """'0x01' 또는 '1' 둘 다 허용."""
    return int(text, 0)


def fmt_rad(rad: float) -> str:
    """라디안 값을 'rad(도)' 형태로 함께 보여주는 포맷터.

    CAN 프로토콜/모터 내부는 전부 라디안 기준이라 계산은 라디안으로 하지만,
    사람이 읽을 땐 도(degree) 가 더 직관적이라 항상 같이 표시한다.
    """
    return f"{rad:+.4f}rad ({math.degrees(rad):+.1f}°)"


def read_state(ctrl: MotorControl, motor: Motor):
    """토크 없이 상태만 조회 (위치, 속도, 토크)."""
    ctrl.refresh_motor_status(motor)
    return motor.getPosition(), motor.getVelocity(), motor.getTorque()


def open_connection(device: str, baud: int):
    print("=" * 60)
    print("[연결] 시리얼 포트 열기")
    print(f"  포트: {device} (baud={baud})")
    print("=" * 60)
    ser = serial.Serial(device, baud, timeout=0.001)
    ctrl = MotorControl(ser)
    print("[OK] 시리얼 포트 연결 완료\n")
    return ser, ctrl


def step1_connect(device: str, baud: int, can_id: int, master_id: int):
    print("=" * 60)
    print("[1단계] ID 지정 / 연결")
    print(f"  포트        : {device} (baud={baud})")
    print(f"  CAN ID      : 0x{can_id:02X}")
    print(f"  Master ID   : 0x{master_id:02X}")
    print("=" * 60)

    ser = serial.Serial(device, baud, timeout=0.001)
    ctrl = MotorControl(ser)
    motor = Motor(MOTOR_TYPE, can_id, master_id)
    ctrl.addMotor(motor)
    print("[OK] 시리얼 포트 연결 및 모터 객체 등록 완료\n")
    return ser, ctrl, motor


def step2_ping(ctrl: MotorControl, motor: Motor):
    print("[2단계] 신호 테스트 (토크 없음 — 손으로 축을 돌려도 안전)")
    print(f"  {PING_INTERVAL}s 간격으로 {PING_COUNT}회 상태 조회. 이 사이에 축을 손으로")
    print("  살짝 돌려보면 값이 변하는지로 통신 생존을 눈으로도 확인할 수 있다.\n")

    positions = []
    for i in range(PING_COUNT):
        pos, vel, tau = read_state(ctrl, motor)
        positions.append(pos)
        print(f"  [{i + 1}/{PING_COUNT}] pos={fmt_rad(pos)}  vel={vel:+.4f}rad/s  tau={tau:+.4f}Nm")
        time.sleep(PING_INTERVAL)

    # 전부 정확히 0.0 이면 응답이 안 오고 있을 가능성 — 그래도 실제 0 rad 일 수
    # 있으니 단정은 못 하지만, 경고는 해준다.
    if all(p == 0.0 for p in positions):
        print("\n[WARN] 5회 모두 pos=0.0 — 응답이 안 오고 있을 수 있다.")
        print("       (실제로 0 rad 위치일 수도 있으니, 축을 손으로 돌려서 값이")
        print("        바뀌는지 다시 한번 확인해볼 것)")
    else:
        print("\n[OK] 값이 갱신되고 있음 — CAN 통신 응답 확인됨")
    print()


def _ramp_to(ctrl: MotorControl, motor: Motor, start: float, target: float,
             duration: float, kp: float, kd: float):
    """start → target 까지 선형 램프로 서서히 이동 (MIT 모드)."""
    steps = max(int(duration * RAMP_FREQ), 1)
    dt = duration / steps
    for i in range(1, steps + 1):
        q_des = start + (target - start) * (i / steps)
        ctrl.controlMIT(motor, kp, kd, q_des, 0.0, DEFAULT_TAU_FF)
        time.sleep(dt)


def step3_mit_test(ctrl: MotorControl, motor: Motor, delta: float, kp: float, kd: float, hold: float):
    print("[3단계] MIT 모드 테스트 (실제로 축이 움직입니다)")
    print(f"  게인: Kp={kp}, Kd={kd} / 이동각: ±{fmt_rad(delta)}")
    input("  → 축 주변에 손가락 등 끼임 위험이 없는지 확인했으면 Enter...")

    # CTRL_MODE(RID=10) 레지스터가 MIT(1)가 아닌 다른 모드(위치-속도/속도/
    # 힘-위치)로 저장돼 있으면 MIT 프레임을 보내도 반응이 없을 수 있다.
    # 새 모터는 보통 기본값이 MIT지만, 이전에 다른 모드로 쓰고 저장된 적이
    # 있을 수 있으니 안전하게 매번 명시적으로 MIT로 전환한다(세션 한정 —
    # save_motor_param() 을 따로 호출하지 않으면 플래시에는 저장되지 않음).
    if ctrl.switchControlMode(motor, Control_Type.MIT):
        print("  [OK] 제어 모드 = MIT 확인/전환됨")
    else:
        print("  [WARN] 제어 모드를 MIT로 전환했는지 확인할 수 없음 — 응답이 없을 수 있음")

    ctrl.enable(motor)
    time.sleep(0.1)

    base_pos, _, _ = read_state(ctrl, motor)
    target_pos = base_pos + delta
    print(f"\n  시작 위치: {fmt_rad(base_pos)} → 목표: {fmt_rad(target_pos)} 로 이동")

    _ramp_to(ctrl, motor, base_pos, target_pos, hold, kp, kd)
    reached_pos, _, _ = read_state(ctrl, motor)
    error = reached_pos - target_pos
    print(f"  도달 위치: {fmt_rad(reached_pos)}  (목표 대비 오차 {fmt_rad(error)})")

    time.sleep(hold)  # 목표 위치에서 잠깐 정지 유지

    print(f"\n  원위치 {fmt_rad(base_pos)} 로 복귀")
    _ramp_to(ctrl, motor, reached_pos, base_pos, hold, kp, kd)
    final_pos, _, _ = read_state(ctrl, motor)
    print(f"  최종 위치: {fmt_rad(final_pos)}")

    ctrl.disable(motor)
    print("\n[OK] 모터 비활성화(토크 OFF) 완료")

    if abs(error) < 0.05:
        print("[PASS] 목표 위치에 잘 도달함 — MIT 모드 위치 제어 정상 동작")
    else:
        print("[WARN] 목표 위치와 오차가 큼 — 배선/CAN ID/Kp·Kd 값을 다시 확인할 것")


def step_goto(ctrl: MotorControl, motor: Motor, target: float, kp: float, kd: float, ramp_time: float):
    """지정한 각도로 램프 이동한 뒤, 그 자리를 계속 유지한다 (Ctrl+C로 종료).

    step3_mit_test() 와 달리 왕복하지 않고 목표에 도달한 뒤 그대로 머문다 —
    "특정 위치로 보내고 싶다"는 요청에 맞춘 기능. 유지하는 동안도 계속
    controlMIT() 을 스트리밍해야 모터가 그 자리를 버틴다(한 번만 보내고
    멈추면 통신 끊김으로 판단해 타임아웃 후 풀릴 수 있음).
    """
    print("[위치 이동] 지정한 각도로 이동 후 유지 (실제로 축이 움직입니다)")
    print(f"  게인: Kp={kp}, Kd={kd} / 목표: {fmt_rad(target)}")
    input("  → 축 주변에 손가락 등 끼임 위험이 없는지 확인했으면 Enter...")

    if ctrl.switchControlMode(motor, Control_Type.MIT):
        print("  [OK] 제어 모드 = MIT 확인/전환됨")
    else:
        print("  [WARN] 제어 모드를 MIT로 전환했는지 확인할 수 없음 — 응답이 없을 수 있음")

    ctrl.enable(motor)
    time.sleep(0.1)

    start_pos, _, _ = read_state(ctrl, motor)
    print(f"\n  시작 위치: {fmt_rad(start_pos)} → 목표: {fmt_rad(target)} 로 이동")

    _ramp_to(ctrl, motor, start_pos, target, ramp_time, kp, kd)
    reached_pos, _, _ = read_state(ctrl, motor)
    error = reached_pos - target
    print(f"  도달 위치: {fmt_rad(reached_pos)}  (목표 대비 오차 {fmt_rad(error)})")

    print("\n  목표 위치를 계속 유지합니다 — 멈추려면 Ctrl+C...")
    while True:
        ctrl.controlMIT(motor, kp, kd, target, 0.0, DEFAULT_TAU_FF)
        time.sleep(0.01)   # 약 100Hz 로 유지 스트리밍


def step4_zero(ctrl: MotorControl, motor: Motor):
    print("[4단계] 제로잉 (영점 0도 설정) — 토크 없음, 손으로 자유롭게 돌릴 수 있다")

    before, _, _ = read_state(ctrl, motor)
    print(f"  현재 각도: {fmt_rad(before)}")
    input("  → 이 축을 원하는 0도 위치(정면/수평 등)로 손으로 맞춘 뒤 Enter...")

    ctrl.set_zero_position(motor)
    time.sleep(0.1)

    # set_zero_position() 만으로는 플래시에 저장되지 않고 전원이 꺼지면
    # 되돌아간다(공식 문서 기준) — save_motor_param() 으로 반드시 플래시에
    # 써야 영구적으로 유지된다.
    ctrl.save_motor_param(motor)
    time.sleep(0.1)

    after, _, _ = read_state(ctrl, motor)
    print(f"  [DONE] 영점 설정 완료 → 새 각도 = {fmt_rad(after)} (0에 가까워야 정상)")
    print("  전원을 꺼도 유지됩니다.\n")


def scan_for_id(ctrl: MotorControl, scan_max: int):
    """
    CAN ID를 모를 때 1~scan_max 번을 순서대로 찔러봐서 응답하는 ID를 찾는다.

    CAN 버스는 ID로 주소를 매기는 구조라 ID를 모르면 애초에 말을 걸 수 없다 —
    그래서 모터 1대만 연결된 상태를 전제로 후보 ID를 하나씩 시도해보는
    브루트포스 스캔이다. 후보 1개당 응답이 없으면 read_motor_param() 내부
    재시도(약 20회 × 0.05s ≈ 1초)만큼 기다리므로 범위가 넓을수록 느려진다.
    """
    print(f"[스캔] CAN ID 1~{scan_max} 범위에서 현재 연결된 액추에이터를 찾는 중...")
    print("       (후보 1개당 무응답이면 최대 약 1초씩 걸림)\n")

    for candidate in range(1, scan_max + 1):
        print(f"  ID 0x{candidate:02X} 시도 중...", end="", flush=True)
        motor = Motor(MOTOR_TYPE, candidate, candidate + 0x10)
        ctrl.addMotor(motor)
        found = ctrl.read_motor_param(motor, DM_variable.ESC_ID)
        if found is not None:
            print(f" 응답! (ESC_ID={found})")
            return candidate, motor
        print(" 무응답")

    return None, None


def step_id_manage(ctrl: MotorControl, scan_max: int, write_id_to: str = None):
    """
    CAN ID를 스캔해서 찾고, 원하면 새 ID로 변경한다.

    write_id_to 가 주어지면 최종적으로 이 액추에이터가 응답하는 ID(스캔된
    원래 ID, 또는 변경에 성공했을 때는 새 ID)를 그 경로에 "0xNN" 형식으로
    저장한다 — actuator_test.sh 가 이 값을 읽어 메뉴의 CAN_ID 로 바로 쓴다.
    """
    print("[ID 확인/변경]")
    current_id, motor = scan_for_id(ctrl, scan_max)

    if current_id is None:
        print(f"\n[FAIL] 1~{scan_max} 범위에서 응답하는 액추에이터를 찾지 못했습니다.")
        print("       배선(CAN_H/CAN_L)·전원·종단저항을 확인하거나,")
        print("       --scan-max 로 스캔 범위를 넓혀서 다시 시도하세요.")
        return

    print(f"\n[OK] 현재 CAN ID = 0x{current_id:02X}")
    final_id = current_id

    answer = input("새 CAN ID로 바꾸시겠습니까? 그대로 두려면 Enter만 누르세요 "
                    "(바꾸려면 예: 0x01): ").strip()
    if not answer:
        print("[SKIP] ID 변경 없이 종료합니다.")
    else:
        new_id = parse_can_id(answer)
        new_master_id = new_id + 0x10
        print(f"\n  새 CAN ID=0x{new_id:02X}, Master ID=0x{new_master_id:02X} 로 변경 시도...")

        ok_id = ctrl.change_motor_param(motor, DM_variable.ESC_ID, new_id)
        if ok_id:
            # ESC_ID(CAN ID) 변경은 레지스터 쓰기라 즉시 적용된다 — 이 순간부터
            # 모터는 new_id 로만 응답한다. 다음 줄(MST_ID 변경)이 여전히 옛
            # SlaveID 로 모터를 찾으면 모터가 무시해버려 항상 실패하므로,
            # 파이썬 쪽 Motor 객체도 바로 new_id 로 갱신해 계속 통신되게 한다.
            motor.SlaveID = new_id
            ctrl.addMotor(motor)

        ok_master = ctrl.change_motor_param(motor, DM_variable.MST_ID, new_master_id)
        if ok_master:
            motor.MasterID = new_master_id
            ctrl.addMotor(motor)

        if ok_id and ok_master:
            ctrl.save_motor_param(motor)
            print("[OK] ID 변경 및 플래시 저장 완료.")
            print(f"     이제부터 이 액추에이터는 CAN ID 0x{new_id:02X} 로 응답합니다.")
            print("     전원을 껐다 켠 뒤(또는 재연결 후) 새 ID로 신호 조회해서 확인하세요.")
            final_id = new_id
        else:
            print("[WARN] 변경 확인에 실패했습니다 — 안전을 위해 플래시 저장은 하지 않았습니다.")
            print(f"       (ESC_ID 변경: {'OK' if ok_id else 'FAIL'}, "
                  f"MST_ID 변경: {'OK' if ok_master else 'FAIL'})")
            print("       배선 상태를 확인하고 다시 시도하세요. (기존 ID 그대로 유지됨)")

    if write_id_to:
        with open(write_id_to, "w") as f:
            f.write(f"0x{final_id:02X}")


def main():
    parser = argparse.ArgumentParser(description="DM-J4310-2EC 액추에이터 연결 테스트")
    parser.add_argument("--id", type=parse_can_id, default=None,
                         help="테스트할 CAN ID (예: 0x01 또는 1). --step id 일 때는 불필요")
    parser.add_argument("--master-id", type=parse_can_id, default=None,
                         help="Master ID (기본값: CAN ID + 0x10, 프로젝트 관례와 동일)")
    parser.add_argument("--device", default=DEVICENAME, help=f"시리얼 포트 (기본: {DEVICENAME})")
    parser.add_argument("--baud", type=int, default=BAUDRATE, help=f"시리얼 보율 (기본: {BAUDRATE})")
    parser.add_argument("--delta", type=float, default=DEFAULT_DELTA_RAD,
                         help=f"MIT 모드 테스트 이동각, 라디안 단위 (기본: {DEFAULT_DELTA_RAD}rad)")
    parser.add_argument("--delta-deg", type=float, default=None,
                         help="MIT 모드 테스트 이동각, 도(degree) 단위 — 주면 --delta 대신 이 값을 "
                              "라디안으로 변환해서 씀 (예: --delta-deg 10)")
    parser.add_argument("--kp", type=float, default=DEFAULT_KP, help=f"테스트용 Kp (기본: {DEFAULT_KP})")
    parser.add_argument("--kd", type=float, default=DEFAULT_KD, help=f"테스트용 Kd (기본: {DEFAULT_KD})")
    parser.add_argument("--hold", type=float, default=DEFAULT_HOLD_SEC,
                         help=f"목표 위치까지 램프 이동 + 정지 유지 시간(초) (기본: {DEFAULT_HOLD_SEC})")
    parser.add_argument("--target", type=float, default=None,
                         help="--step goto 의 목표 각도, 라디안 단위 (중앙 0rad 기준)")
    parser.add_argument("--target-deg", type=float, default=None,
                         help="--step goto 의 목표 각도, 도(degree) 단위 — 주면 --target 대신 "
                              "이 값을 라디안으로 변환해서 씀 (예: --target-deg 15)")
    parser.add_argument("--step", choices=["ping", "mit", "zero", "id", "goto"], default=None,
                         help="이 단계 하나만 실행 (생략하면 신호조회→MIT테스트를 이어서 실행)")
    parser.add_argument("--scan-max", type=int, default=DEFAULT_SCAN_MAX,
                         help=f"--step id 에서 1~이 값까지 스캔 (기본 {DEFAULT_SCAN_MAX})")
    parser.add_argument("--write-id-to", default=None,
                         help="--step id 결과(최종 CAN ID)를 이 파일 경로에 저장 (actuator_test.sh 용)")
    args = parser.parse_args()

    if args.delta_deg is not None:
        args.delta = math.radians(args.delta_deg)
    if args.target_deg is not None:
        args.target = math.radians(args.target_deg)

    if args.step != "id" and args.id is None:
        parser.error("--id 는 필수입니다 (CAN ID를 모른다면 --step id 로 먼저 스캔하세요)")
    if args.step == "goto" and args.target is None:
        parser.error("--step goto 에는 --target(rad) 또는 --target-deg(도) 가 필요합니다")

    ser = None
    motor = None
    try:
        if args.step == "id":
            # CAN ID를 아직 모르는 상태이므로 Motor 객체 없이 포트만 연다
            ser, ctrl = open_connection(args.device, args.baud)
            step_id_manage(ctrl, args.scan_max, args.write_id_to)
        else:
            master_id = args.master_id if args.master_id is not None else args.id + 0x10
            ser, ctrl, motor = step1_connect(args.device, args.baud, args.id, master_id)

            if args.step == "ping":
                step2_ping(ctrl, motor)
            elif args.step == "mit":
                input("Enter 를 누르면 MIT 모드 테스트로 진행합니다 (Ctrl+C로 중단 가능)...")
                step3_mit_test(ctrl, motor, args.delta, args.kp, args.kd, args.hold)
            elif args.step == "zero":
                step4_zero(ctrl, motor)
            elif args.step == "goto":
                step_goto(ctrl, motor, args.target, args.kp, args.kd, args.hold)
            else:
                # --step 생략: 기존처럼 신호조회 → (확인 후) MIT 테스트를 이어서 실행
                step2_ping(ctrl, motor)
                input("Enter 를 누르면 MIT 모드 테스트로 진행합니다 (Ctrl+C로 중단 가능)...")
                step3_mit_test(ctrl, motor, args.delta, args.kp, args.kd, args.hold)

    except (EOFError, KeyboardInterrupt):
        print("\n[중단] 테스트가 중단되었습니다.")
        if ser is not None and motor is not None:
            try:
                ctrl.disable(motor)
            except Exception:
                pass
    except serial.SerialException as e:
        # 동글이 안 꽂혀있거나 경로가 다를 때 여기로 옴 — 트레이스백 대신
        # 한 줄로 안내하고 exit code 1로 종료. actuator_test.sh 쪽은 이
        # 비정상 종료 하나로 메뉴 전체가 꺼지지 않도록 set -e 를 쓰지 않음.
        print(f"\n[ERROR] 포트를 열 수 없습니다: {e}")
        print("        동글이 USB에 꽂혀 있는지, 경로(--device)가 맞는지 확인하세요.")
        sys.exit(1)
    finally:
        if ser is not None:
            ser.close()
            print("[OK] 시리얼 포트 닫힘")


if __name__ == "__main__":
    main()
