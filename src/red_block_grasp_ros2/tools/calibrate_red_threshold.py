#!/usr/bin/env python3
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from red_block_grasp_ros2.tools.calibrate_red_threshold import main

if __name__ == "__main__":
    raise SystemExit(main())
