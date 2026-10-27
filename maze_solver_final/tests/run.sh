#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
test_bin=$(mktemp)
trap 'rm -f "$test_bin"' EXIT
for rule in 0 1 2; do
    cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,address \
       -DCONFIG_FINAL_EXIT_RULE="$rule" -Itests/stubs -Imain/include \
       tests/test_navigation.c main/junction.c main/node_memory.c -o "$test_bin"
    "$test_bin"
done
