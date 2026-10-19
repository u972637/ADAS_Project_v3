#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MP3 재생기 — subprocess(mpg123/ffplay/cvlc) 기반.

애니메이션 wildlife 시나리오(animal/audio.py의 WavPlayer)와 같은 방식으로,
외부 플레이어 프로세스를 fork해 재생한다. pygame.mixer는 쓰지 않는다 —
첫 재생 시 오디오 장치를 여는 pygame.mixer.init() 호출이 GIL을 오래 쥐고
있을 수 있어, 같은 프로세스의 500Hz 모터 제어 스레드를 지연시켜
MotorFault(control loop stalled)를 유발한 적이 있다.
"""

import os
import shutil
import subprocess
import threading
from typing import Optional


class MP3LoopPlayer:
    def __init__(self, mp3_path: str, enabled: bool = True, loop: bool = True):
        self.mp3_path = os.path.abspath(mp3_path)
        self.enabled  = enabled
        self.loop     = loop
        self._warned  = False
        self._proc: Optional[subprocess.Popen] = None
        self._proc_lock = threading.Lock()
        self._cmd = self._select_command()
        self._volume = 1.0

    # ── 재생 커맨드 선택 ──

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

        if self._cmd is None:
            if not self._warned:
                print("[WARN] mp3 재생기 없음 — mpg123/ffplay/cvlc 중 하나를 설치하세요")
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
        """볼륨 값만 보관한다 — 이미 띄운 외부 플레이어 프로세스의 실시간
        볼륨 변경은 지원하지 않는다(다음 재생부터 반영하려면 플레이어별
        커맨드에 볼륨 옵션을 추가해야 한다). people/main_facetrack.py 의
        +/- 볼륨 키가 예외 없이 동작하도록 값만 유지해둔다."""
        self._volume = max(0.0, min(1.0, val))

    def get_volume(self) -> float:
        return self._volume

    def update(self, should_play: bool):
        if should_play:
            self.start()
        else:
            self.stop()

    def close(self):
        self.stop()
