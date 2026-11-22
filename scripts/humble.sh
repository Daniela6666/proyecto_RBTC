#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
if [ "$#" -eq 0 ]; then set -- ros2 launch dif_bot bringup.launch.py; fi
exec env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-xterm}" DISPLAY="${DISPLAY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" PULSE_SERVER="${PULSE_SERVER:-}" bash --noprofile --norc -c '
  source /opt/ros/humble/setup.bash
  cd "$1"
  shift
  if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi
  exec "$@"
' bash "$ROOT" "$@"
