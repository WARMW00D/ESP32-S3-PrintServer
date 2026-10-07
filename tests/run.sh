#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_binary=$(mktemp)
trap 'rm -f "$test_binary"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -Itests/shim -I. tests/test_mobile_print.cpp -o "$test_binary"
"$test_binary" "$@"
