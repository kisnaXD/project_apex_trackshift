"""Pure Python Phase 1 race-control foundation.

The package deliberately has no ROS imports at core import time.  ROS adapters
can translate these immutable contracts to generated messages at the boundary.
"""

from .contracts import *

__all__ = [name for name in globals() if not name.startswith("_")]
