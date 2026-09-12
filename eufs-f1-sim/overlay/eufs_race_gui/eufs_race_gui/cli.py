"""Explicit GUI entrypoint; importing the package never starts Qt."""

from __future__ import annotations

import argparse

from .app import MainWindow, create_application
from .core import LiveDataSource, ReplayDataSource


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="eufs_race_gui")
    parser.add_argument("--replay", help="JSONL causal records to open offline")
    parser.add_argument("--run-dir", help="saved run bundle containing decisions/events JSONL")
    parser.add_argument("--live", action="store_true", help="connect to ROS through the threaded live adapter")
    parser.add_argument("--run-id", default="live", help="live record run identifier")
    parser.add_argument("--epoch-id", type=int, default=0, help="live epoch identifier")
    args = parser.parse_args(argv)
    app = create_application()
    if args.live:
        source = LiveDataSource()
    elif args.run_dir:
        source = ReplayDataSource.from_run_dir(args.run_dir)
    elif args.replay:
        source = ReplayDataSource.from_jsonl(args.replay)
    else:
        source = ReplayDataSource()
    window = MainWindow(source); window.show()
    bridge = None
    if args.live:
        from .ros_bridge import LiveROSBridge
        bridge = LiveROSBridge(run_id=args.run_id, epoch_id=args.epoch_id, parent=window)
        window.attach_live_bridge(bridge)
    return app.exec_()
