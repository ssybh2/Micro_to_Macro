#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="${TMPDIR:-/tmp}/mujoco_micro_core_self_check"
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  -I"$package_dir/include" \
  "$package_dir/src/kinematics.cpp" \
  "$package_dir/src/control_core.cpp" \
  "$package_dir/tools/core_self_check.cpp" \
  -o "$output"
"$output"
