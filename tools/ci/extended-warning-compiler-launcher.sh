#!/usr/bin/env sh
# Apply CI-only diagnostics to this application's sources, not Zephyr/NCS code.
compiler=$1
shift

for arg in "$@"; do
    case "$arg" in
        */open-earable-v2/src/*) exec "$compiler" -Wextra -Wshadow "$@" ;;
    esac
done

exec "$compiler" "$@"
