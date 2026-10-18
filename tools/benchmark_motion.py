#!/usr/bin/env python3
"""Reproducible reference-generator benchmark; never opens CAN, camera or audio."""

import argparse
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import ruckig
from motor.trajectory import MotionCommand, MotionPlanner


def distribution(values):
    values = sorted(values)
    return {
        "samples": len(values),
        "median_us": statistics.median(values),
        "p99_us": values[int(0.99 * (len(values) - 1))],
        "max_us": max(values),
    }


def benchmark(ticks):
    planner = MotionPlanner((0, 0))
    samples, replans = [], []
    goal = MotionCommand((0, 0))
    for k in range(ticks + 1000):
        now = k * 0.002
        if k % 20 == 0:
            goal = MotionCommand((0.7 * math.sin(now * 1.7), 0.3 * math.sin(now * 2.2)))
        count = planner.replans
        started = time.perf_counter_ns()
        planner.step(now, goal)
        elapsed = (time.perf_counter_ns() - started) / 1000
        if k >= 1000:
            (replans if planner.replans != count else samples).append(elapsed)
    moves = {}
    for degrees in (1, 5, 15):
        planner = MotionPlanner((0, 0))
        planner.step(0, MotionCommand((math.radians(degrees), 0)))
        moves[str(degrees)] = planner._trajectory.duration
    return {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "processor": platform.processor(),
        "ruckig": ruckig.__version__,
        "control_period_ms": 2,
        "goal_period_ms": 40,
        "sampling": distribution(samples),
        "replanning": distribution(replans),
        "rest_to_rest_yaw_degrees_duration_s": moves,
        "scope": "Local reference computation only; excludes CAN, scheduling, inference and plant response.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", type=int, default=60000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.ticks < 1000:
        parser.error("--ticks must be at least 1000")
    result = json.dumps(benchmark(args.ticks), indent=2, ensure_ascii=False)
    print(result)
    if args.output:
        args.output.write_text(result + "\n")


if __name__ == "__main__":
    main()
