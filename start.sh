#!/usr/bin/env bash
# Shared wildlife runtime: menus 1/2 start paused. W/resume enables the scenario.
# R/E/Space/stop halts tracking/audio and returns to default; W resumes explicitly.
# Menus 3/4/5 retain the hand-raise/manual/acoustic-test tools.
set -uo pipefail
cd "$(dirname "$0")"

uv run python tools/motor_manual_control.py
