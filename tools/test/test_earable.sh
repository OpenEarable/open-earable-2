#!/bin/sh
set -eu

test_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
python=${PCB_TEST_PYTHON:-"$test_dir/.venv/bin/python"}
if [ -z "${PCB_TEST_PYTHON:-}" ] && [ ! -x "$python" ]; then
    python=python3
fi

exec "$python" "$test_dir/test_earable.py" "$@"
