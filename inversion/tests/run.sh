#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")"
binary=$(mktemp)
trap 'rm -f "$binary"' EXIT
${CC:-cc} -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I../main ../main/inversion_controller.c test_controller.c -lm -o "$binary"
# LeakSanitizer's /proc inspection is unavailable in some containers; this
# controller performs no heap allocations. Keep ASan and UBSan enabled.
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "$binary"
