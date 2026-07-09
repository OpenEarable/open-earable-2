#!/usr/bin/env python3
"""Compatibility wrapper for the generic battery debug tool."""

from __future__ import annotations

import sys

from battery_debug import main


if __name__ == "__main__":
    raise SystemExit(main(["recover", *sys.argv[1:]]))
