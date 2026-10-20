#!/usr/bin/env python3
"""Audition a warning clip directly, without camera or motor decisions."""

import argparse
import platform
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from animal import settings
from animal.audio import AudioCatalog, DryRunPlayer, WavPlayer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--label", choices=sorted(set(settings.CLASS_TO_AUDIO.values())), default="boar"
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    catalog = AudioCatalog.from_directory(settings.WARNING_AUDIO_DIR, {args.label}, 1)
    command = settings.AUDIO_PLAYER_COMMAND or (
        "afplay" if platform.system() == "Darwin" else "aplay"
    )
    player = DryRunPlayer() if args.dry_run or settings.DRY_RUN else WavPlayer(command)
    file = catalog.next(args.label)
    try:
        print(f"{args.label}: {file}")
        player.play(file)
        while player.is_playing():
            time.sleep(0.05)
        status = player.status()
        if status.state != "completed":
            raise RuntimeError(
                f"{status.state}: exit={status.returncode} {status.error}"
            )
        print("completed")
    finally:
        player.stop()


if __name__ == "__main__":
    main()
