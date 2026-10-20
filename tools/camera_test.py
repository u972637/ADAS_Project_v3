#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arducam B0538C 카메라 단독 테스트 — YOLO/모터 없이 카메라만 열어 화면에 띄운다.

main.py와 같은 카메라 설정(해상도/FOURCC)을 쓰지만, YOLO(ultralytics/torch)
등 무거운 의존성은 import하지 않아 훨씬 가볍고 빠르게 뜬다. 카메라가 실제로
켜지는지, 어떤 /dev/video* 인덱스로 잡히는지, 실측 fps가 얼마나 나오는지만
빠르게 확인하고 싶을 때 쓴다. actuator_test.sh의 "카메라 테스트" 메뉴가
이 스크립트를 호출한다.

카메라는 animal/settings.py 의 CAMERA_INDEX(/dev/videoN)로 연다. 번호는 USB를
다시 꽂거나 다른 카메라(노트북 웹캠 등)가 먼저 잡히면 바뀔 수 있으니, 검은
화면이 나오면 --list 로 어느 번호가 Arducam 인지 확인한다. 다른 장치를
시험하려면 --camera 에 번호나 경로를 직접 넘긴다.

실행:
  uv run python tools/camera_test.py                      # 설정된 고정 경로로 시도
  uv run python tools/camera_test.py --camera 0           # 번호로 시도
  uv run python tools/camera_test.py --camera /dev/v4l/by-id/<...>-video-index0
  uv run python tools/camera_test.py --list               # 연결된 카메라 + 0~9번 인덱스 열어보기

키: q 또는 ESC = 종료
"""

import argparse
import time

import cv2

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from animal.camera import (
    FRAME_W,
    FRAME_H,
    CAMERA_FOURCC,
    camera_hint,
    configured_camera,
    open_camera,
    read_camera_frame as read_frame,
)

# animal/settings.py 의 CAMERA_DEVICE(by-id 경로, 설정했을 때) 또는 숫자
# CAMERA_INDEX(/dev/videoN). tools/motor_manual_control.py(메뉴 3)도 이 값을 쓴다.
DEFAULT_CAMERA_IDX = configured_camera()

# 2592x1944 그대로 띄우면 화면보다 커서 축소 표시만 한다 (창 첫 크기 = 2592x1944 의
# 이 비율, 창은 메뉴 1처럼 드래그로 자유롭게 줄이고 늘릴 수 있다)
DISPLAY_SCALE = 0.3

FPS_PRINT_INTERVAL = 1.0  # 실측 fps 콘솔 출력 간격(초)


def list_indices(max_index: int = 9):
    print(f"[연결된 카메라] {camera_hint()}\n")
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


def run_preview(index):
    print(f"[..] 카메라 {index} 여는 중 (해상도 {FRAME_W}x{FRAME_H}, FOURCC={CAMERA_FOURCC})...")
    cap = open_camera(index)
    if cap is None:
        print(f"[ERROR] 카메라 {index} 를 열 수 없습니다.")
        print(f"        {camera_hint()}")
        print("        다른 장치를 시도하려면 --camera, 연결 상태를 보려면 --list 를 쓰세요.")
        return

    print("[OK] 카메라 열림 — 창이 뜨면 'q' 또는 ESC 로 종료")

    win_name = "Camera Test (q/ESC: quit)"
    # 메뉴 1(animal/runtime.py)과 같은 WINDOW_NORMAL: 크기를 마우스로 조절할 수
    # 있다. 기본값 WINDOW_AUTOSIZE 는 영상 크기에 고정돼 줄일 수 없다.
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, int(FRAME_W * DISPLAY_SCALE), int(FRAME_H * DISPLAY_SCALE))

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
    parser.add_argument("--camera", "--index", dest="camera",
                         type=lambda text: int(text) if text.isdigit() else text,
                         default=DEFAULT_CAMERA_IDX,
                         help=f"카메라 번호 또는 /dev/v4l/by-id 경로 (기본: {DEFAULT_CAMERA_IDX})")
    parser.add_argument("--list", action="store_true",
                         help="연결된 카메라와 0~9번 인덱스를 열어보고 결과만 출력 (미리보기 없음)")
    args = parser.parse_args()

    if args.list:
        list_indices()
    else:
        run_preview(args.camera)


if __name__ == "__main__":
    main()
