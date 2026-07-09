#!/usr/bin/env python3
"""Print OpenEarable v2 battery voltage in millivolts."""

from __future__ import annotations

import sys

from battery_debug import main


if __name__ == "__main__":
    raise SystemExit(main(["voltage", "--raw", *sys.argv[1:]]))
