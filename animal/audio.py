"""Class-specific WAV catalog and playback."""

from __future__ import annotations

import json
import random
import shutil
import subprocess
import wave
from pathlib import Path


class AudioCatalog:
    def __init__(
        self,
        files_by_label: dict[str, list[Path]],
        min_per_class: int,
        rng: random.Random | None = None,
    ) -> None:
        if min_per_class < 1:
            raise ValueError("min_per_class must be positive")
        self._files: dict[str, tuple[Path, ...]] = {}
        for label, files in files_by_label.items():
            if len(files) < min_per_class or len(set(files)) != len(files):
                raise ValueError(f"{label}: requires {min_per_class} distinct clips")
            for file in files:
                if not file.is_file() or file.suffix.lower() != ".wav":
                    raise ValueError(f"not a playable WAV file: {file}")
                try:
                    with wave.open(str(file), "rb") as reader:
                        if reader.getnframes() <= 0 or reader.getframerate() <= 0:
                            raise ValueError(f"empty WAV file: {file}")
                except (EOFError, wave.Error) as error:
                    raise ValueError(f"invalid WAV file: {file}") from error
            self._files[label] = tuple(files)
        self._bags: dict[str, list[Path]] = {}
        self._last: dict[str, Path] = {}
        self._rng = rng or random.Random()

    @classmethod
    def from_manifest(
        cls, manifest: Path, labels: set[str], min_per_class: int
    ) -> AudioCatalog:
        data = json.loads(manifest.read_text(encoding="utf-8"))
        files_by_label: dict[str, list[Path]] = {label: [] for label in labels}
        for entry in data["clips"]:
            label = entry["class"]
            if label in files_by_label:
                files_by_label[label].append((manifest.parent / entry["file"]).resolve())
        return cls(files_by_label, min_per_class)

    def next(self, label: str) -> Path:
        if not self._bags.get(label):
            bag = list(self._files[label])
            self._rng.shuffle(bag)
            if len(bag) > 1 and bag[-1] == self._last.get(label):
                bag[0], bag[-1] = bag[-1], bag[0]
            self._bags[label] = bag
        selected = self._bags[label].pop()
        self._last[label] = selected
        return selected


class WavPlayer:
    def __init__(self, command: str) -> None:
        if shutil.which(command) is None:
            raise FileNotFoundError(f"WAV player command not found: {command}")
        self.command = command
        self._process: subprocess.Popen[bytes] | None = None

    def play(self, file: Path) -> None:
        if self.is_playing():
            raise RuntimeError("audio is already playing")
        self._process = subprocess.Popen(
            [self.command, str(file)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def is_playing(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def stop(self) -> None:
        if self.is_playing():
            assert self._process is not None
            self._process.terminate()
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1)
        self._process = None


class DryRunPlayer:
    def play(self, file: Path) -> None:
        pass

    def is_playing(self) -> bool:
        return False

    def stop(self) -> None:
        pass
