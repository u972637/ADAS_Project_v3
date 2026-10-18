#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arducam B0538C 카메라 단독 테스트 — YOLO/모터 없이 카메라만 열어 화면에 띄운다.

main.py와 같은 카메라 설정(해상도/FOURCC)을 쓰지만, YOLO(ultralytics/torch)
등 무거운 의존성은 import하지 않아 훨씬 가볍고 빠르게 뜬다. 카메라가 실제로
켜지는지, 어떤 /dev/video* 인덱스로 잡히는지, 실측 fps가 얼마나 나오는지만
빠르게 확인하고 싶을 때 쓴다. actuator_test.sh의 "카메라 테스트" 메뉴가
이 스크립트를 호출한다.

카메라를 아직 안 써봐서 실제 /dev/video* 인덱스를 모르는 상태라면
--index 로 바꿔가며 시도해볼 것 (예: 0, 1, 2 ...).

실행:
  uv run python tools/camera_test.py              # 기본 인덱스(4)로 시도
  uv run python tools/camera_test.py --index 0    # 다른 인덱스로 시도
  uv run python tools/camera_test.py --list       # 0~9번 인덱스를 순서대로 열어만 보고 결과 출력

키: q 또는 ESC = 종료
"""

import argparse
import time

import cv2

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from animal.camera import FRAME_W, FRAME_H, CAMERA_FOURCC, open_camera, read_camera_frame as read_frame
from animal.settings import CAMERA_INDEX as DEFAULT_CAMERA_IDX

# 2592x1944 그대로 띄우면 화면보다 커서 축소 표시만 한다
DISPLAY_SCALE = 0.4

FPS_PRINT_INTERVAL = 1.0  # 실측 fps 콘솔 출력 간격(초)


def list_indices(max_index: int = 9):
    print(f"[검색] 인덱스 0~{max_index} 를 순서대로 열어봅니다...\n")
    found = []
    for i in range(max_index + 1):
        cap = open_camera(i)
        if cap is None:
            print(f"  index {i}: 열기 실패")
            continue
        ok, frame = read_frame(cap)
        if ok:
            print(f"  index {i}: 열림 + 프레임 수신 성공 (shape={frame.shape})")
            found.append(i)
        else:
            print(f"  index {i}: 열리긴 했으나 프레임 수신 실패 (다른 장치일 수 있음)")
        cap.release()
    print()
    if found:
        print(f"[결과] 프레임 수신 성공한 인덱스: {found}")
    else:
        print("[결과] 프레임을 수신한 인덱스가 없습니다 — 배선/권한(video 그룹)을 확인하세요.")


def run_preview(index: int):
    print(f"[..] 카메라 index={index} 여는 중 (해상도 {FRAME_W}x{FRAME_H}, FOURCC={CAMERA_FOURCC})...")
    cap = open_camera(index)
    if cap is None:
        print(f"[ERROR] index={index} 카메라를 열 수 없습니다.")
        print("        다른 인덱스를 시도하려면 --index, 어느 인덱스가 되는지 모르면 --list 를 쓰세요.")
        return

    print("[OK] 카메라 열림 — 창이 뜨면 'q' 또는 ESC 로 종료")

    win_name = "Camera Test (q/ESC: quit)"
    cv2.namedWindow(win_name)

    frame_count = 0
    last_fps_t = time.time()
    measured_fps = 0.0

    try:
        while True:
            ok, frame = read_frame(cap)
            if not ok or frame is None:
                print("[WARN] 프레임 읽기 실패 — 재시도")
                time.sleep(0.05)
                continue

            frame_count += 1
            now = time.time()
            if now - last_fps_t >= FPS_PRINT_INTERVAL:
                measured_fps = frame_count / (now - last_fps_t)
                frame_count = 0
                last_fps_t = now

            h, w = frame.shape[:2]
            disp = cv2.resize(frame, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE)))
            dh, dw = disp.shape[:2]
            cx, cy = dw // 2, dh // 2
            cv2.line(disp, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 1)
            cv2.line(disp, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 1)
            cv2.putText(disp, f"index={index}  {w}x{h}  {measured_fps:.1f}fps",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.imshow(win_name, disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[OK] 카메라 해제 완료")


def main():
    parser = argparse.ArgumentParser(description="Arducam B0538C 카메라 단독 테스트")
    parser.add_argument("--index", type=int, default=DEFAULT_CAMERA_IDX,
                         help=f"카메라 장치 인덱스 (기본: {DEFAULT_CAMERA_IDX})")
    parser.add_argument("--list", action="store_true",
                         help="0~9번 인덱스를 순서대로 열어보고 결과만 출력 (미리보기 없음)")
    args = parser.parse_args()

    if args.list:
        list_indices()
    else:
        run_preview(args.index)


if __name__ == "__main__":
    main()
