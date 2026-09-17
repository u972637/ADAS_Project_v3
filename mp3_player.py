#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MP3 재생기 — pygame.mixer 기반 (subprocess/fork 없음)

pygame 미설치 시 subprocess 방식으로 자동 폴백.
설치: pip install pygame
"""

import os
import queue
import shutil
import subprocess
import threading
from typing import Optional

try:
    import pygame
    _HAVE_PYGAME = True
except ImportError:
    _HAVE_PYGAME = False


# ──────────────────────────────────────────────
# pygame 방식: fork 없이 백그라운드 스레드에서 재생
# ──────────────────────────────────────────────

if _HAVE_PYGAME:
    class _MixerWorker:
        """pygame.mixer.music 를 담당하는 싱글턴 백그라운드 스레드.
        모든 MP3LoopPlayer 인스턴스가 공유한다.
        """
        _instance: Optional["_MixerWorker"] = None
        _lock = threading.Lock()

        @classmethod
        def get(cls) -> "_MixerWorker":
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
            return cls._instance

        def __init__(self):
            pygame.mixer.pre_init(44100, -16, 2, 2048)
            pygame.mixer.init()
            self._q: queue.SimpleQueue = queue.SimpleQueue()
            threading.Thread(target=self._loop, daemon=True).start()

        def _loop(self):
            while True:
                item = self._q.get()
                if item is None:
                    break
                cmd, *args = item
                try:
                    if cmd == "play":
                        path, loops = args
                        pygame.mixer.music.load(path)
                        pygame.mixer.music.play(loops)
                    elif cmd == "stop":
                        pygame.mixer.music.stop()
                except Exception as e:
                    print(f"[WARN] pygame 오디오 오류: {e}")

        def play(self, path: str, loops: int = 0):
            """비동기 재생 요청 (즉시 반환)."""
            self._q.put(("play", path, loops))

        def stop(self):
            self._q.put(("stop",))

        def set_volume(self, val: float):
            """볼륨 설정 (0.0 ~ 1.0). pygame.mixer.music은 스레드 안전."""
            pygame.mixer.music.set_volume(max(0.0, min(1.0, val)))

        def get_volume(self) -> float:
            return pygame.mixer.music.get_volume()


# ──────────────────────────────────────────────
# MP3LoopPlayer — 공개 인터페이스
# ──────────────────────────────────────────────

class MP3LoopPlayer:
    def __init__(self, mp3_path: str, enabled: bool = True, loop: bool = True):
        self.mp3_path = os.path.abspath(mp3_path)
        self.enabled  = enabled
        self.loop     = loop
        self._warned  = False

        if _HAVE_PYGAME:
            self._mixer = _MixerWorker.get()
            self._proc  = None          # subprocess 폴백 미사용
            self._proc_lock = None
            self._cmd   = None
        else:
            self._mixer     = None
            self._proc: Optional[subprocess.Popen] = None
            self._proc_lock = threading.Lock()
            self._cmd       = self._select_command()

    # ── subprocess 폴백용 커맨드 선택 ──

    def _select_command(self):
        if shutil.which("mpg123"):
            cmd = ["mpg123", "-q"]
            if self.loop:
                cmd.extend(["--loop", "-1"])
            cmd.append(self.mp3_path)
            return cmd

        if shutil.which("ffplay"):
            cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet"]
            if self.loop:
                cmd.extend(["-loop", "0"])
            cmd.append(self.mp3_path)
            return cmd

        if shutil.which("cvlc"):
            if self.loop:
                return ["cvlc", "-q", "--loop", self.mp3_path]
            return ["cvlc", "-q", "--play-and-exit", self.mp3_path]

        return None

    # ── 내부 파일 유효성 검사 ──

    def _check_file(self) -> bool:
        if not os.path.exists(self.mp3_path):
            if not self._warned:
                print(f"[WARN] mp3 파일 없음: {self.mp3_path}")
                self._warned = True
            return False
        return True

    # ── 공개 API ──

    def start(self, restart: bool = False):
        if not self.enabled or not self._check_file():
            return

        if _HAVE_PYGAME:
            loops = -1 if self.loop else 0
            self._mixer.play(self.mp3_path, loops)
            return

        # subprocess 폴백
        if self._cmd is None:
            if not self._warned:
                print("[WARN] mp3 재생기 없음 — pip install pygame 권장")
                self._warned = True
            return

        def _launch():
            with self._proc_lock:
                if self._proc is not None and self._proc.poll() is None:
                    if not restart:
                        return
                    self._proc.terminate()
                    try:
                        self._proc.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        self._proc.kill()
                    self._proc = None
            try:
                proc = subprocess.Popen(
                    self._cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                with self._proc_lock:
                    self._proc = proc
            except Exception as e:
                if not self._warned:
                    print(f"[WARN] mp3 재생 실패: {e}")
                    self._warned = True

        threading.Thread(target=_launch, daemon=True).start()

    def stop(self):
        if _HAVE_PYGAME:
            self._mixer.stop()
            return
        if self._proc_lock is None:
            return
        with self._proc_lock:
            if self._proc is None:
                return
            if self._proc.poll() is None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
            self._proc = None

    def play_once(self):
        """처음부터 한 번 재생 (비동기)."""
        self.start(restart=True)

    def set_volume(self, val: float):
        """볼륨 설정 (0.0 = 묵음, 1.0 = 최대)."""
        if _HAVE_PYGAME and self._mixer is not None:
            self._mixer.set_volume(val)

    def get_volume(self) -> float:
        """현재 볼륨 반환 (0.0 ~ 1.0)."""
        if _HAVE_PYGAME and self._mixer is not None:
            return self._mixer.get_volume()
        return 1.0

    def update(self, should_play: bool):
        if should_play:
            self.start()
        else:
            self.stop()

    def close(self):
        self.stop()
