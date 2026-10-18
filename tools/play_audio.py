#!/usr/bin/env python3
"""Audition a warning/threat clip directly, without camera or motor decisions."""

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
    parser.add_argument("--stage", choices=("warning", "threat"), default="threat")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    warning = args.stage == "warning"
    root = settings.WARNING_AUDIO_DIR if warning else settings.AUDIO_DIR
    catalog = AudioCatalog.from_directory(
        root, {args.label}, 1 if warning else settings.MIN_AUDIO_FILES_PER_CLASS
    )
    command = settings.AUDIO_PLAYER_COMMAND or (
        "afplay" if platform.system() == "Darwin" else "aplay"
    )
    player = DryRunPlayer() if args.dry_run or settings.DRY_RUN else WavPlayer(command)
    file = catalog.next(args.label)
    try:
        print(f"{args.stage}: {file}")
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
