"""EUFS race GUI foundation.

Core data contracts can be imported on headless machines.  Qt is only loaded
when :mod:`eufs_race_gui.app` or the explicit CLI is used.
"""

from .core import (
    ChannelRegistry, ChannelSample, ChannelSpec, DataSource, DecisionPin, EventTimeline,
    LiveDataSource, Record, RecordStore, ReplayDataSource, flatten_scalars,
)

__all__ = [
    "ChannelRegistry", "ChannelSample", "ChannelSpec", "DataSource", "DecisionPin", "EventTimeline",
    "LiveDataSource", "Record", "RecordStore", "ReplayDataSource", "flatten_scalars",
]
