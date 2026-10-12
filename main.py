#!/usr/bin/env python3
"""Wildlife detection, camera tracking, and class-based audio playback."""

from __future__ import annotations

import time

import cv2
from ultralytics import YOLO

from camera import CameraCapture, open_camera, read_camera_frame
import settings
from wildlife import (
    TrackerCameraController,
    WildlifeDeterrent,
    process_frame,
    validate_model_classes,
)


def run() -> None:
    model_path = settings.MODEL_PATH
    if not model_path.is_file():
        raise FileNotFoundError(f"동물 탐지 모델이 없습니다: {model_path}")

    model = YOLO(str(model_path))
    print(f"[OK] 동물 탐지 모델 로드: {model_path}")
    tracker = None
    camera = None
    service = None
    try:
        service = WildlifeDeterrent.from_settings()
        validate_model_classes(model.names, service.class_map, service.expected_names)
        for class_id in sorted(service.class_map):
            print(
                f"[CLASS] model {class_id}={model.names[class_id]} "
                f"-> audio {service.class_map[class_id]}"
            )

        if settings.ENABLE_MOTOR:
            from motor.dm4310_tracker import FRAME_H, FRAME_W, Tracker

            tracker = Tracker()
            service.camera = TrackerCameraController(tracker, (FRAME_W, FRAME_H))
            print("[OK] 동물 bbox 중심으로 Damiao 모터 추적")

        cap = open_camera(settings.CAMERA_INDEX)
        if cap is None:
            raise RuntimeError(f"카메라를 열 수 없습니다: index={settings.CAMERA_INDEX}")
        try:
            for _ in range(20):
                read_camera_frame(cap)
            camera = CameraCapture(cap, settings.CAMERA_INDEX)
        except Exception:
            cap.release()
            raise

        last_sequence = 0
        print(f"[OK] 동물 탐지 시작: camera={settings.CAMERA_INDEX}, dry_run={settings.DRY_RUN}")
        while True:
            ready, frame, last_sequence, captured_at = camera.read_new(last_sequence)
            if not ready:
                time.sleep(0.01)
                continue

            frame = cv2.flip(frame, 1)  # DM4310 좌표계에 맞춘 좌우 반전
            started_at = time.monotonic()
            event = process_frame(model, service, frame, captured_at)
            inference_ms = (time.monotonic() - started_at) * 1000
            if event.audio_file is not None:
                print(f"[AUDIO] {event.reason}: {event.audio_file}")

            if settings.HEADLESS:
                continue
            label = "No target"
            if event.target is not None:
                x1, y1, x2, y2 = (int(v) for v in event.target.xyxy)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = event.label or "Unknown"
                label += f" {event.target.confidence:.2f}"
            cv2.putText(
                frame,
                f"{label} | {event.reason} | YOLO {inference_ms:.1f}ms",
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                "[Q] Quit [E] Center",
                (20, frame.shape[0] - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (180, 180, 180),
                2,
            )
            cv2.imshow("Wildlife Deterrent", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("e") and tracker is not None:
                tracker.go_center()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if tracker is not None:
                tracker.close()
        finally:
            try:
                if service is not None:
                    service.player.stop()
            finally:
                if camera is not None:
                    camera.release()
                if not settings.HEADLESS:
                    cv2.destroyAllWindows()


def main() -> None:
    run()


if __name__ == "__main__":
    main()
