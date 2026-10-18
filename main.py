#!/usr/bin/env python3
"""Production wildlife deterrent entrypoint; no experiment menu required."""

import argparse

from animal.runtime import run


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run the wildlife deterrent pipeline.")
    for flag, help_text in (
        ("motor", "enable real motors (default: enabled)"),
        ("start-paused", "wait for W/resume before aiming or playing audio"),
        ("dry-run", "simulate motor commands and audio without operating either"),
        ("headless", "use terminal commands without a preview window"),
    ):
        parser.add_argument(
            f"--{flag}",
            action=argparse.BooleanOptionalAction,
            default=None,
            dest="enable_motor" if flag == "motor" else flag.replace("-", "_"),
            help=help_text,
        )
    run(**vars(parser.parse_args(argv)))


if __name__ == "__main__":
    main()
