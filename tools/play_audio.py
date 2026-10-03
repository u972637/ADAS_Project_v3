#!/usr/bin/env python3
"""Play one random WAV using a simulated wildlife classification result."""

import time
from types import SimpleNamespace

import settings
from wildlife import WildlifeDeterrent


CLASS_ID = 0  # Change to 1 for water_deer with the example class mapping.


def main() -> None:
    if settings.DRY_RUN:
        raise RuntimeError("Set DRY_RUN = False in settings.py to hear audio")
    if CLASS_ID not in settings.CLASS_TO_AUDIO:
        raise ValueError(f"No audio mapping for class ID {CLASS_ID}")

    service = WildlifeDeterrent.from_settings()
    result = SimpleNamespace(
        orig_shape=(720, 1280),
        boxes=SimpleNamespace(
            xyxy=[[100, 100, 300, 300]],
            conf=[0.9],
            cls=[CLASS_ID],
        ),
    )

    try:
        start = time.monotonic()
        for index in range(settings.CONSECUTIVE_FRAMES):
            event = service.process_result(result, start + index * 0.04)
        if event.reason != "played":
            raise RuntimeError(f"Audio was not started: {event.reason}")
        print(f"{event.label}: {event.reason} -> {event.audio_file}")
        while service.player.is_playing():
            time.sleep(0.1)
    finally:
        service.player.stop()


if __name__ == "__main__":
    main()
