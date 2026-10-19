"""Construct the Tracker implementation selected by motor.config.CONTROL_MODE.

Single place both the in-process callers (animal/runtime.py, menus) and the
IPC child worker (motor/ipc_tracker.py) go through, so "trajectory" vs "step"
only needs to be decided in one spot.
"""

from motor import config


def create_tracker(devicename=None, baudrate=None):
    kwargs = {}
    if devicename is not None:
        kwargs["devicename"] = devicename
    if baudrate is not None:
        kwargs["baudrate"] = baudrate
    if config.CONTROL_MODE == "trajectory":
        from motor.dm4310_tracker import Tracker

        return Tracker(**kwargs)
    if config.CONTROL_MODE == "step":
        from motor.dm4310_tracker_step import StepTracker

        return StepTracker(**kwargs)
    raise ValueError(f"unknown motor.config.CONTROL_MODE: {config.CONTROL_MODE!r}")
