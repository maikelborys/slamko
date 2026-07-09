#!/usr/bin/env bash
# Wrapper for bench_cuvslam_provider.py — REQUIRED because a ROS-sourced shell has
# /opt/ros/jazzy/lib (isaac_ros 4.4, old C-API libcuvslam.so) on LD_LIBRARY_PATH,
# which shadows the wheel's bundled v16 lib and breaks the import with
# "undefined symbol: _ZN7cuvslam4Slam13LocalizeInMap...".
set -euo pipefail
VENV="$HOME/.venvs/cuvslam"
PKG="$VENV/lib/python3.12/site-packages/cuvslam"
export LD_LIBRARY_PATH="$PKG${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$VENV/bin/python" "$(dirname "$0")/bench_cuvslam_provider.py" "$@"
