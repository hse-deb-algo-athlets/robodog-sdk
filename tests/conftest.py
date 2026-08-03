from __future__ import annotations

import sys
from pathlib import Path

# The examples are runnable scripts, not part of the distribution — put them on
# the path so tests/test_examples.py can start them like any other node.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "examples"))
