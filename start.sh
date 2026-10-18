#!/usr/bin/env bash
# Optional experiment menu; production runs directly through main.py.
# Menus 1/2 rehearse the shared pipeline, starting paused; 3/4/5 are bench tools.
set -uo pipefail
cd "$(dirname "$0")"

exec uv run python tools/motor_manual_control.py
