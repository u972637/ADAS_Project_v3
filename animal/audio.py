"""Class-specific WAV catalog and playback."""

from __future__ import annotations

import random
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass
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
    def from_directory(
        cls, root: Path, labels: set[str], min_per_class: int
    ) -> AudioCatalog:
        """Load <root>/<label>/*.wav — the folder name is the audio label."""
        for label in labels:
            if not (root / label).is_dir():
                raise FileNotFoundError(f"audio folder not found: {root / label}")
        files_by_label = {
            label: sorted(path.resolve() for path in (root / label).glob("*.wav"))
            for label in labels
        }
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


@dataclass(frozen=True)
class PlaybackStatus:
    state: str  # idle, playing, completed, failed, stopped
    file: Path | None = None
    returncode: int | None = None
    error: str | None = None


class WavPlayer:
    def __init__(self, command: str) -> None:
        if shutil.which(command) is None:
            raise FileNotFoundError(f"WAV player command not found: {command}")
        self.command = command
        self._process = None
        self._stderr = None
        self._status = PlaybackStatus("idle")

    def play(self, file: Path) -> None:
        if self.is_playing():
            raise RuntimeError("audio is already playing")
        self._stderr = tempfile.TemporaryFile()
        try:
            self._process = subprocess.Popen(
                [self.command, str(file)],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
            )
            self._status = PlaybackStatus("playing", file)
        except Exception as error:
            self._stderr.close()
            self._stderr = None
            self._status = PlaybackStatus("failed", file, error=str(error))
            raise

    def status(self) -> PlaybackStatus:
        if self._process is not None and self._status.state == "playing":
            code = self._process.poll()
            if code is not None:
                self._stderr.seek(0)
                error = self._stderr.read(4096).decode(errors="replace").strip()
                self._stderr.close()
                self._stderr = None
                self._status = PlaybackStatus(
                    "completed" if code == 0 else "failed",
                    self._status.file,
                    code,
                    error or None,
                )
                self._process = None
        return self._status

    def is_playing(self) -> bool:
        return self.status().state == "playing"

    def stop(self) -> None:
        if self.is_playing():
            self._process.terminate()
            try:
                self._process.wait(timeout=0.3)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=0.3)
            self._stderr.close()
            self._stderr = self._process = None
            self._status = PlaybackStatus("stopped", self._status.file)


class DryRunPlayer:
    """Silent simulation retains WAV duration so the observation timing is real."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._status = PlaybackStatus("idle")
        self._ends_at = 0.0

    def play(self, file: Path) -> None:
        with wave.open(str(file), "rb") as reader:
            duration = reader.getnframes() / reader.getframerate()
        self._ends_at = self.clock() + duration
        self._status = PlaybackStatus("playing", file)

    def status(self) -> PlaybackStatus:
        if self._status.state == "playing" and self.clock() >= self._ends_at:
            self._status = PlaybackStatus("completed", self._status.file, 0)
        return self._status

    def is_playing(self) -> bool:
        return self.status().state == "playing"

    def stop(self) -> None:
        if self._status.state == "playing":
            self._status = PlaybackStatus("stopped", self._status.file)
