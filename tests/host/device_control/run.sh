#!/bin/sh
set -eu
suite_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
binary=$(mktemp /tmp/oe-device-control.XXXXXX)
trap 'rm -f "$binary"' EXIT
c++ -std=c++17 -Wall -Wextra -Wno-unused-variable -I"$suite_dir/stubs" "$suite_dir/main.cpp" -o "$binary"
"$binary"
