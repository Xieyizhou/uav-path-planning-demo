"""Compatibility wrapper for the A* PX4 flight runner."""

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.flight.fly_astar_path import main


if __name__ == "__main__":
    main()
